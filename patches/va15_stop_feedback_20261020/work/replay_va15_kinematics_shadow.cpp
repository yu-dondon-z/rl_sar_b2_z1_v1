#include "va15_kinematics_model_io.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

static std::vector<std::string> Split(const std::string& text)
{
    std::istringstream stream(text); std::string part; std::vector<std::string> result;
    while (std::getline(stream,part,','))
    {
        if (!part.empty() && part.back() == '\r') part.pop_back();
        result.push_back(part);
    }
    return result;
}
static double Quantile(std::vector<double> values, double q)
{
    values.erase(std::remove_if(values.begin(),values.end(),[](double v){return !std::isfinite(v);}),values.end());
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(),values.end());
    return values[static_cast<std::size_t>((values.size()-1)*q)];
}
static void Number(std::ostream& out, double value)
{
    if (std::isfinite(value)) out << std::setprecision(12) << value; else out << "null";
}
int main(int argc, char** argv)
{
    if (argc != 4) throw std::runtime_error("Usage: replay model.json lowstate.csv NEW_output_prefix");
    const std::filesystem::path prefix(argv[3]);
    const auto csv = prefix.string()+".csv", json = prefix.string()+".json";
    if (std::filesystem::exists(csv) || std::filesystem::exists(json)) throw std::runtime_error("Refusing to overwrite output");
    va15_shadow::Tracker tracker(LoadVA15ShadowModel(argv[1]));
    std::ifstream input(argv[2]);
    std::string line;
    if (!std::getline(input,line)) throw std::runtime_error("Empty/unreadable input CSV");
    const auto names = Split(line);
    std::map<std::string,std::size_t> columns;
    for (std::size_t i=0;i<names.size();++i) columns[names[i]]=i;
    if (!prefix.parent_path().empty()) std::filesystem::create_directories(prefix.parent_path());
    std::ofstream output(csv);
    if (!output) throw std::runtime_error("Cannot open output CSV");
    output << "elapsed,tick,input_valid,fresh,sequence_valid,contact_evidence_valid,control_authorized,height_hypothesis,speed_hypothesis,height_spread,velocity_spread\n";
    std::vector<double> heights,speeds,height_spreads,velocity_spreads;
    std::array<std::vector<double>,4> leg_heights;
    int rows=0,valid=0,fresh=0;
    while (std::getline(input,line))
    {
        const auto values=Split(line);
        if (values.size()!=names.size()) throw std::runtime_error("Malformed CSV row");
        auto value=[&](const std::string& key){return std::stod(values.at(columns.at(key)));};
        va15_shadow::Sample sample;
        for (int i=0;i<12;++i) {sample.q[i]=value("q_"+std::to_string(i));sample.dq[i]=value("dq_"+std::to_string(i));}
        for (int i=0;i<4;++i) sample.quaternion[i]=value("quat_"+std::to_string(i));
        for (int i=0;i<3;++i) sample.gyro[i]=value("gyro_"+std::to_string(i));
        const auto tick=static_cast<std::uint32_t>(value("low_tick"));
        const double now=value("elapsed");
        tracker.Update(sample,tick,now);
        const auto result=tracker.Snapshot(now);
        ++rows; valid+=result.input_valid;fresh+=result.fresh && result.sequence_valid;
        if (result.input_valid)
        {
            heights.push_back(result.hypothesis_height);speeds.push_back(result.hypothesis_velocity_world.norm());
            height_spreads.push_back(result.height_spread);velocity_spreads.push_back(result.velocity_spread);
            for (int i=0;i<4;++i) leg_heights[i].push_back(result.feet[i].flat_ground_height);
        }
        output<<std::setprecision(12)<<now<<','<<tick<<','<<result.input_valid<<','<<result.fresh<<','
              <<result.sequence_valid<<",0,0,"<<result.hypothesis_height<<','<<result.hypothesis_velocity_world.norm()
              <<','<<result.height_spread<<','<<result.velocity_spread<<'\n';
    }
    std::ofstream summary(json);
    summary<<"{\n  \"samples\": "<<rows<<",\n  \"input_valid_rows\": "<<valid
           <<",\n  \"fresh_sequence_rows\": "<<fresh
           <<",\n  \"control_authorized\": false,\n  \"contact_evidence_supplied\": false,\n"
           <<"  \"hypotheses_only\": true,\n  \"height_hypothesis_median_m\": ";Number(summary,Quantile(heights,.5));
    summary<<",\n  \"speed_hypothesis_p95_m_s\": ";Number(summary,Quantile(speeds,.95));
    summary<<",\n  \"height_spread_p95_m\": ";Number(summary,Quantile(height_spreads,.95));
    summary<<",\n  \"velocity_spread_p95_m_s\": ";Number(summary,Quantile(velocity_spreads,.95));
    summary<<",\n  \"leg_height_medians_DDS_FR_FL_RR_RL\": [";
    for (int i=0;i<4;++i) {if(i)summary<<", ";Number(summary,Quantile(leg_heights[i],.5));}
    summary<<"]\n}\n";
    if (!rows || !valid) throw std::runtime_error("No valid kinematic input; no acceptance");
    std::cout<<"Replayed "<<rows<<" lowstate rows; shadow hypotheses only, no contact or motion permission. Saved "<<json<<'\n';
}
