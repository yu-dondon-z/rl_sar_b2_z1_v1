// Offline guide-only characterization, NOT robot/support acceptance.
// Build without robot SDKs, Torch, Eigen, DDS, or any Inbox worker:
//   c++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread replay_v251_visual_dropout.cpp -lyaml-cpp -o /tmp/replay_v251_visual_dropout
// Run with the V2.5.1 diagnosis directory, or explicit control.csv + JSONL paths.
// Original packet contents and capture/publication timestamps are never changed.
#include "production/near_visual_io.hpp"
#include "production/trial_supervisor.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unordered_map>

namespace {
using va15_visual::Guide;
using va15_visual::Packet;
using Ns = std::uint64_t;

void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

struct Counts { std::size_t accepted=0, incomplete=0, nul=0, malformed=0; };
struct Control {
    Ns now=0;
    unsigned trial=0, visual=0, sessions=0;
    std::size_t line=0;
};
struct RecordedPacket {
    Packet packet;
    Ns publish=0, capture=0, sequence=0;
    bool declared_valid=false;
    double rejected_span=0.;
    bool rejected_look=false;
    std::string reason;
    std::size_t line=0;
};

std::vector<std::string> Split(const std::string& line) {
    // These diagnostic CSVs contain numeric, unquoted cells only.
    std::vector<std::string> cells;
    std::istringstream input(line);
    std::string cell;
    while (std::getline(input, cell, ',')) cells.push_back(cell);
    if (!line.empty() && line.back()==',') cells.emplace_back();
    return cells;
}

Ns Unsigned(const std::string& text) {
    Require(!text.empty() && text.front()!='-', "negative/empty integer");
    std::size_t end=0;
    const auto result=std::stoull(text, &end);
    Require(end==text.size(), "invalid integer cell");
    return result;
}

std::vector<Control> ReadControls(const std::filesystem::path& path, Counts& counts) {
    std::ifstream input(path, std::ios::binary);
    Require(bool(input), "cannot open "+path.string());
    std::string line;
    Require(bool(std::getline(input,line)), "missing CSV header");
    if (!line.empty() && line.back()=='\r') line.pop_back();
    const auto header=Split(line);
    std::unordered_map<std::string,std::size_t> column;
    for (std::size_t i=0; i<header.size(); ++i) column.emplace(header[i],i);
    for (const auto* key : {"control_start_ns","trial_stage","visual_stage","visual_session_count"})
        Require(column.count(key)!=0, std::string("missing CSV column ")+key);
    std::vector<Control> rows;
    std::size_t number=1;
    while (std::getline(input,line)) {
        ++number;
        // A final record without its terminating newline may be truncated,
        // even if the remaining prefix happens to have the correct cell count.
        if (input.eof()) { ++counts.incomplete; continue; }
        if (line.find('\0')!=std::string::npos) { ++counts.nul; continue; }
        if (!line.empty() && line.back()=='\r') line.pop_back();
        const auto cells=Split(line);
        if (cells.size()!=header.size()) { ++counts.malformed; continue; }
        try {
            for (const auto& cell : cells) {
                std::size_t end=0;
                (void)std::stod(cell,&end); // nan diagnostics are legitimate.
                Require(end==cell.size(), "partial numeric CSV cell");
            }
            Control row;
            row.now=Unsigned(cells.at(column.at("control_start_ns")));
            row.trial=Unsigned(cells.at(column.at("trial_stage")));
            row.visual=Unsigned(cells.at(column.at("visual_stage")));
            row.sessions=Unsigned(cells.at(column.at("visual_session_count")));
            row.line=number;
            Require(rows.empty() || row.now>rows.back().now,"nonmonotonic control time");
            rows.push_back(row);
            ++counts.accepted;
        } catch (const std::exception&) { ++counts.malformed; }
    }
    Require(!rows.empty(), "no complete numeric CSV rows");
    return rows;
}

std::vector<RecordedPacket> ReadPackets(const std::filesystem::path& path, Counts& counts) {
    std::ifstream input(path, std::ios::binary);
    Require(bool(input), "cannot open "+path.string());
    std::vector<RecordedPacket> packets;
    std::string line, nonce;
    std::size_t number=0;
    while (std::getline(input,line)) {
        ++number;
        if (input.eof()) { ++counts.incomplete; continue; }
        if (line.find('\0')!=std::string::npos) { ++counts.nul; continue; }
        try {
            const auto node=YAML::Load(line);
            Require(node.IsMap(), "non-map packet");
            if (nonce.empty()) nonce=node["session_id"].as<std::string>();
            RecordedPacket row;
            row.packet=va15_visual::Parse(line,nonce);
            row.publish=node["publish_monotonic_ns"].as<Ns>();
            row.capture=node["capture_monotonic_ns"].as<Ns>();
            row.sequence=node["sequence"].as<Ns>();
            row.declared_valid=node["valid"].as<bool>();
            row.reason=node["reason"].as<std::string>("");
            if (node["near_diagnostics"]) {
                const auto diagnostic=node["near_diagnostics"];
                row.rejected_span=diagnostic["path_span_px"].as<double>(0.);
                row.rejected_look=diagnostic["look_supported"].as<bool>(false);
            }
            row.line=number;
            Require(row.publish>=row.capture && row.capture!=0,"invalid trace clock");
            Require(packets.empty() || row.publish>packets.back().publish,
                    "nonmonotonic trace publication");
            packets.push_back(std::move(row));
            ++counts.accepted;
        } catch (const std::exception&) { ++counts.malformed; }
    }
    Require(!packets.empty(), "no complete JSONL packets");
    return packets;
}

const RecordedPacket& Sequence(const std::vector<RecordedPacket>& packets, Ns sequence) {
    const auto it=std::find_if(packets.begin(),packets.end(),[&](const auto& row) {
        return row.sequence==sequence;
    });
    Require(it!=packets.end(), "missing recorded sequence "+std::to_string(sequence));
    return *it;
}

void PrintCounts(const char* name,const Counts& counts) {
    std::cout<<name<<": complete="<<counts.accepted<<" incomplete="<<counts.incomplete
             <<" NUL="<<counts.nul<<" malformed="<<counts.malformed<<'\n';
}

void ProveManualRestarts(Guide stopped,const std::vector<RecordedPacket>& packets,Ns stop_time) {
    va15_trial::Supervisor supervisor;
    supervisor.Configure(true,true,true);
    supervisor.Stop("recorded visual stale stop");
    std::size_t cursor=0;
    while (cursor<packets.size() && packets[cursor].publish<=stop_time) ++cursor;
    const auto sessions=stopped.sessions;
    for (int cycle=0; cycle<3; ++cycle) {
        Ns now=0;
        do {
            Require(cursor<packets.size(), "trace exhausted during manual-restart characterization");
            const auto& row=packets[cursor++];
            now=row.publish;
            stopped.Observe(row.packet,now);
            const auto held=stopped.Tick(now,.005,true,false,true);
            Require(held.x==0. && stopped.stage==Guide::Stage::Stopped,
                    "fresh packets automatically resumed a stopped guide");
            Require(supervisor.Step(.005,.15,0.,0.,true,true,false)==0.,
                    "fresh packets automatically rearmed the supervisor");
        } while (!stopped.Ready(now));
        for (int missing=0; missing<4; ++missing)
            Require(!supervisor.Rearm(missing!=0,missing!=1,missing!=2,missing!=3),
                    "rearm accepted missing quiet/health/centered/stopped proof");
        Require(!stopped.Start(now,false,true,true),"start accepted nonquiet support");
        Require(!stopped.Start(now,true,false,true),"start accepted displaced sticks");
        Require(!stopped.Start(now,true,true,false),"start accepted invalid health");
        // Supplied logical prerequisites are fixtures, NOT inferred contacts
        // and NOT claims that this recorded robot completed its physical stop.
        Require(supervisor.Rearm(true,true,true,true),"explicit qualified rearm failed");
        Require(stopped.Start(now,true,true,true),"explicit qualified visual restart failed");
        const auto moving=stopped.Tick(now,.005,true,false,true);
        Require(!moving.stop && moving.x==va15_visual::Speed,"manual restart gave no guide output");
        Require(supervisor.Step(.005,moving.x,0.,0.,true,true,false)==.15,
                "explicit rearm gave no supervisor output");
        // B is supplied as stop_key, before any new motion can be issued.
        // Guide-level proof only: this does not exercise a hardware FSM's
        // GetUp/GetDown transition ordering or claim authorization to lower.
        const auto b=stopped.Tick(now,.005,true,true,true);
        Require(b.stop && b.x==0. && b.yaw==0.,"B did not override live guide output");
        Require(supervisor.Step(.005,.15,0.,0.,true,true,true)==0. && supervisor.Stopping(),
                "B did not stop the supervisor");
    }
    Require(stopped.sessions==sessions+3,"manual session count mismatch");
    std::cout<<"PASS logical fixtures: three explicit restarts; missing prerequisites refused; "
                "fresh packets alone never resume; B overrides live guide output\n";
}
} // namespace

int main(int argc,char** argv) try {
    std::filesystem::path control,packets_path;
    if (argc==2) {
        const std::filesystem::path directory(argv[1]);
        control=directory/"control.csv";
        packets_path=directory/"exchange"/"guide_packets.jsonl";
    } else if (argc==3) {
        control=argv[1];packets_path=argv[2];
    } else {
        std::cerr<<"usage: "<<argv[0]<<" DIAGNOSIS_DIRECTORY\n       "<<argv[0]
                 <<" CONTROL_CSV GUIDE_PACKETS_JSONL\n";
        return 2;
    }
    Counts control_counts,packet_counts;
    const auto controls=ReadControls(control,control_counts);
    const auto packets=ReadPackets(packets_path,packet_counts);
    const auto start=std::find_if(controls.begin(),controls.end(),[](const auto& row) {
        return row.trial==3 && row.visual==1 && row.sessions==1;
    });
    Require(start!=controls.end(),"missing first accepted visual session");
    const auto recorded_stop=std::find_if(start,controls.end(),[](const auto& row) {
        return row.trial==4 && row.visual==2;
    });
    Require(recorded_stop!=controls.end(),"missing recorded visual stop");
    const auto& before=Sequence(packets,488);
    const auto& rejected=Sequence(packets,489);
    const auto& after=Sequence(packets,490);
    Require(before.declared_valid && !rejected.declared_valid && after.declared_valid,
            "seq488/489/490 validity pattern changed");
    Require(rejected.reason=="Need >=200 px supported curve and an observed bounded lookahead point" &&
            rejected.rejected_span==138. && !rejected.rejected_look,
            "seq489 recorded rejection evidence changed");
    Guide guide(true,true,true),before_stale(true,true,true);
    std::size_t cursor=0;
    bool started=false;
    Ns stop_time=0;
    // The recorder saves every fourth control tick. Nominal controller dt
    // stays .005; freshness uses the UNMODIFIED recorded monotonic now.
    // Publication is ideal availability; the 25 ms Inbox read phase was not
    // logged. This cannot reconstruct every real control/worker interleave.
    for (const auto& row : controls) {
        while (cursor<packets.size() && packets[cursor].publish<=row.now)
            guide.Observe(packets[cursor++].packet,row.now);
        if (row.now==start->now) {
            Require(guide.Start(row.now,true,true,true),
                    std::string("recorded-frame guide start failed: ")+guide.reason);
            started=true;
        }
        if (!started) continue;
        const auto prior=guide;
        const auto output=guide.Tick(row.now,.005,true,false,true);
        if (output.stop) { stop_time=row.now;before_stale=prior;break; }
    }
    Require(stop_time==recorded_stop->now,
            "publication-order sampled-tick guide replay differs from recorded stop time");
    Require(guide.latest.sequence==488 && guide.seen.sequence==489,
            "replay did not preserve seq488 through rejected seq489");
    Require(std::string(guide.reason)=="visual line invalid, changed, or stale","wrong stop cause");
    const Ns expires=before.capture+va15_visual::MaximumAgeNs;
    Require(stop_time>expires && after.publish>stop_time,"not the recorded stale/fresh edge");
    auto inclusive=before_stale;
    Require(!inclusive.Tick(expires,.005,true,false,true).stop,"500 ms inclusive bound changed");
    auto genuine_loss=before_stale;
    Require(genuine_loss.Tick(expires+1,.005,true,false,true).stop,
            "genuine loss did not stop at original freshness bound");
    Require(genuine_loss.Tick(after.publish+va15_visual::MaximumAgeNs,.005,true,false,true).x==0.,
            "genuine loss resumed without a manual request");
    auto b_priority=before_stale;
    const auto b=b_priority.Tick(expires+1,.005,false,true,true);
    Require(b.stop && b.x==0. && b.yaw==0. &&
            std::string(b_priority.reason)=="operator stop or stick moved",
            "B stop_key lost precedence over simultaneous stale/health faults");
    unsigned phase_stops=0,phase_survives=0;
    for (Ns phase=0;phase<5000000;phase+=100000) {
        auto shifted=before_stale;
        bool stopped=false;
        for (unsigned tick=0;tick<3;++tick) {
            const Ns now=expires+phase+tick*5000000ULL;
            if (now>=after.publish) shifted.Observe(after.packet,now);
            if (shifted.Tick(now,.005,true,false,true).stop) { stopped=true;break; }
        }
        if (stopped) ++phase_stops; else ++phase_survives;
    }
    Require(phase_stops>0 && phase_survives>0,"expected phase-sensitive edge absent");
    std::cout<<std::fixed<<std::setprecision(6)
             <<"GUIDE-ONLY CHARACTERIZATION / NOT ROBOT ACCEPTANCE\n"
             <<"Original packet timestamps; sampled control ticks; ideal publication availability.\n"
             <<"Unlogged Inbox polling/missing control ticks and physical support are not reconstructed.\n";
    PrintCounts("control.csv",control_counts);PrintCounts("guide_packets.jsonl",packet_counts);
    std::cout<<"recorded/replayed stop_ns="<<stop_time<<" control.csv:"<<recorded_stop->line
             <<" relative_start_s="<<double(stop_time-start->now)/1.e9<<'\n'
             <<"seq488 capture_age_at_stop_ms="<<double(stop_time-before.capture)/1.e6
             <<" threshold_ms="<<double(va15_visual::MaximumAgeNs)/1.e6<<'\n'
             <<"seq489 rejected_span_px="<<rejected.rejected_span
             <<" observed_lookahead="<<rejected.rejected_look<<" reason="<<rejected.reason<<'\n'
             <<"seq490 fresh_publication_after_stop_ms="<<double(after.publish-stop_time)/1.e6
             <<" expiry_to_next_publication_ms="<<double(after.publish-expires)/1.e6<<'\n'
             <<"5 ms ideal-tick phase sweep (100 us spacing): stop="<<phase_stops
             <<" remain_running="<<phase_survives<<" (capture/publication timestamps unchanged)\n";
    ProveManualRestarts(guide,packets,stop_time);
    std::cout<<"PASS unchanged 500 ms inclusive age bound; genuine loss stops; B priority over "
                "stale/health faults; no automatic resume; no hardware initialized\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr<<"FAIL characterization: "<<error.what()<<'\n';
    return 1;
}
