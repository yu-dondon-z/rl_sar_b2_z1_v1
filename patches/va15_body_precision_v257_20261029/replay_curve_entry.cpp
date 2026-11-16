// Saved V249 feedback + camera packets. Geometry-only counterfactual, NOT a
// closed-loop robot replay or a hardware/actuator qualification.
#include "production/visual_tcp.hpp"
#include "production/near_visual_io.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

static std::vector<std::string> Split(const std::string& line) {
    std::vector<std::string> out;std::stringstream s(line);std::string v;
    while(std::getline(s,v,','))out.push_back(v);return out;
}
int main(int argc,char** argv) {
    try {
        if(argc!=3)throw std::runtime_error("control.csv guide_packets.jsonl required");
        std::ifstream csv(argv[1]),packets(argv[2]);
        if(!csv||!packets)throw std::runtime_error("missing saved fixture");
        std::string line;std::getline(csv,line);const auto fields=Split(line);
        std::map<std::string,std::size_t> cols;
        for(std::size_t i=0;i<fields.size();++i)cols[fields[i]]=i;
        va15_tcp::History poses;std::uint64_t start=0,end=0;unsigned old_opposing=0;
        while(std::getline(csv,line)) {
            if(line.find('\0')!=std::string::npos)continue;
            const auto r=Split(line);if(r.size()!=fields.size())continue;
            const auto get=[&](const std::string& k){return r.at(cols.at(k));};
            const auto number=[&](const std::string& k){return std::stod(get(k));};
            const auto ns=std::stoull(get("control_end_ns"));
            if(get("visual_stage")=="1") {
                if(!start)start=ns;end=ns;
                if(number("visual_bearing_rad")>0.&&number("visual_look_bearing_rad")>0.&&number("visual_would_yaw")<0.)++old_opposing;
            }
            if(get("tcp_pose_valid")!="1")continue;
            va15_tcp::Pose p;p.ns=ns;p.valid=true;
            for(int j=0;j<3;++j)p.p[j]=number("local_pose_"+std::to_string(j));
            p.q=Eigen::Quaterniond(number("body_quaternion_wxyz_0"),number("body_quaternion_wxyz_1"),
                                  number("body_quaternion_wxyz_2"),number("body_quaternion_wxyz_3")).normalized();
            if(va15_tcp::ValidPose(p))poses.poses.push_back(p);
        }
        if(!start||end<=start||poses.poses.empty())throw std::runtime_error("incomplete entry fixture");
        std::vector<va15_visual::Packet> observations;
        while(std::getline(packets,line)) {
            if(line.find('\0')!=std::string::npos)continue;
            try {
                const auto n=YAML::Load(line);const auto nonce=n["session_id"].as<std::string>();
                observations.push_back(va15_visual::Parse(line,nonce));
            } catch(const std::exception&) {} // final incomplete record only
        }
        va15_visual::Guide guide(true,true,true);va15_tcp::Tracker tracker;
        bool begun=false;std::size_t next=0;unsigned running=0,opposing=0,body=0;
        double max_step=0.,previous=0.;
        for(auto now=poses.poses.front().ns;now<=end;now+=5000000ULL) {
            va15_tcp::Pose current;if(!poses.At(now,current))continue;
            while(next<observations.size()&&observations[next].publish_ns<=now) {
                guide.Observe(observations[next],now);va15_tcp::Pose capture;
                if(va15_visual::Valid(guide.latest,now)&&poses.At(guide.latest.capture_ns,capture))tracker.Observe(guide.latest,capture);
                ++next;
            }
            guide.metric_bearing_valid=tracker.Guidance(current,guide.metric_bearing);
            guide.body_reference_valid=tracker.BodyGuidance(current,guide.body_lateral,guide.body_heading,guide.body_history_used);
            if(!begun&&now>=start) {
                if(!guide.Start(now,true,true,true))throw std::runtime_error(guide.reason);
                begun=true;
            }
            if(!begun)continue;
            const auto out=guide.Tick(now,.005,true,false,true);
            if(!out.stop&&guide.stage==va15_visual::Guide::Stage::Running) {
                ++running;body+=guide.body_reference_valid;
                if(!guide.body_reference_valid&&guide.metric_bearing_valid&&guide.metric_bearing>.02&&out.yaw<0.)++opposing;
                max_step=std::max(max_step,std::fabs(out.yaw-previous));previous=out.yaw;
            }
        }
        const bool pass=begun&&old_opposing>=100&&running>=200&&opposing==0&&body==0&&max_step<=.000200001;
        std::cout<<std::boolalpha<<"{\"pass\":"<<pass<<",\"recorded_opposing_rows\":"<<old_opposing
                 <<",\"replay_running_samples\":"<<running<<",\"replay_opposing_samples\":"<<opposing
                 <<",\"unseen_body_authority_samples\":"<<body<<",\"max_yaw_step_rad_s\":"<<max_step
                 <<",\"recorded_geometry_only\":true,\"hardware_qualified\":false}\n";
        return pass?0:2;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
