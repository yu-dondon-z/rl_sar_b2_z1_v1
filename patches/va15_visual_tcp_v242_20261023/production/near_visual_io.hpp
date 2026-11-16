#pragma once
#include "near_visual_guide.hpp"
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace va15_visual {
inline std::uint64_t NowNs() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline Packet Parse(const std::string& text,const std::string& nonce) {
    Packet p;
    if(text.size()>16384)return p;
    try {
        const auto n=YAML::Load(text);
        if(!n.IsMap()||n["schema"].as<std::string>("")!="va15_long_visual_v1"||
           n["session_id"].as<std::string>("")!=nonce||
           n["calibration_sha256"].as<std::string>("")!=CalibrationSHA||
           n["timestamp_basis"].as<std::string>("")!="host_receive_monotonic"||
           n["image_width"].as<int>(0)!=1920||n["image_height"].as<int>(0)!=1080)return p;
        p.session_matched=true;p.sequence=n["sequence"].as<std::uint64_t>(0);
        p.capture_ns=n["capture_monotonic_ns"].as<std::uint64_t>(0);
        p.publish_ns=n["publish_monotonic_ns"].as<std::uint64_t>(0);
        p.valid=n["valid"].as<bool>(false);
        if(!p.valid)return p;
        p.u=n["near_u"].as<double>();p.v=n["near_v"].as<double>();
        p.width=n["native_width_px"].as<double>();p.confidence=n["confidence"].as<double>();
        p.contrast=n["contrast"].as<double>();p.rms=n["native_fit_rms_px"].as<double>();
        p.look_u=n["look_u"].as<double>();p.look_v=n["look_v"].as<double>();
        p.look_confidence=n["look_confidence"].as<double>();
        p.motion_compensated=n["motion_compensation_valid"].as<bool>(false);
        p.association_error_px=n["association_delta_px"].as<double>(0.);
        if(p.motion_compensated&&(!std::isfinite(p.association_error_px)||p.association_error_px<0.||
            p.association_error_px>60.||n["motion_inliers"].as<unsigned>(0)<20))return {};
        p.path_count=n["path_point_count"].as<unsigned>();p.path_span=n["path_vertical_span_px"].as<double>();
        const auto path=n["path_points_uv"];
        if(!path.IsSequence()||path.size()!=p.path_count||path.size()<25||path.size()>128)return {};
        std::vector<std::array<double,2>> points;
        for(const auto& node:path) {
            if(!node.IsSequence()||node.size()!=2)return {};
            const double u=node[0].as<double>(),v=node[1].as<double>();
            if(!std::isfinite(u)||!std::isfinite(v)||u<0||u>=1920||v<.55*1080||v>.98*1080)return {};
            if(!points.empty()&&(v>points.back()[1]+.01||std::hypot(u-points.back()[0],v-points.back()[1])>30.))return {};
            points.push_back({u,v});
        }
        if(std::fabs(points.front()[1]-points.back()[1]-p.path_span)>2.)return {};
        const auto interpolate=[&](double v) {
            for(std::size_t j=1;j<points.size();++j)if(points[j-1][1]>=v&&points[j][1]<=v) {
                const double delta=points[j-1][1]-points[j][1];
                if(delta<1.e-6)return points[j][0];
                return points[j-1][0]+(points[j][0]-points[j-1][0])*(points[j-1][1]-v)/delta;
            }
            return double(NAN);
        };
        const double near_u=interpolate(p.v),look_u=interpolate(p.look_v);
        if(!std::isfinite(near_u)||!std::isfinite(look_u)||std::fabs(near_u-p.u)>8.||std::fabs(look_u-p.look_u)>2.)return {};
        p.path=std::move(points);
    }catch(...){return {};}
    return p;
}
// All file access/parsing runs in this owned worker, never in the 200 Hz
// control or 500 Hz motor writer. No sensors, sockets, publishers or commands.
class Inbox {
    std::atomic<bool> stop_{false};std::thread thread_;
    std::atomic<unsigned> phase_{0};
    std::shared_ptr<const Packet> latest_=std::make_shared<const Packet>();
public:
    Inbox(const std::filesystem::path& directory,const std::string& nonce) {
        if(nonce.size()!=32||!std::all_of(nonce.begin(),nonce.end(),[](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='f');}))
            throw std::runtime_error("Invalid unique visual session id");
        if(!std::filesystem::is_directory(directory))throw std::runtime_error("Visual exchange directory missing");
        const auto path=directory/"near_guide.json";
        const auto context=directory/"vision_context.json";
        thread_=std::thread([this,path,context,nonce]{
            std::uint64_t last_context=0;
            while(!stop_.load()) {
                Packet p;
                try {
                    std::error_code e;const auto size=std::filesystem::file_size(path,e);
                    if(!e&&size<=16384) {std::ifstream f(path);std::string data((std::istreambuf_iterator<char>(f)),{});p=Parse(data,nonce);}
                }catch(...){}
                std::atomic_store(&latest_,std::make_shared<const Packet>(p));
                const auto now=NowNs();
                if(now-last_context>=100000000ULL) {
                    try {
                        const auto temporary=context.string()+".native.tmp";
                        {std::ofstream f(temporary);f<<"{\"session_id\":\""<<nonce<<"\",\"monotonic_ns\":"<<now
                            <<",\"phase\":"<<phase_.load()<<"}";f.flush();if(!f)throw std::runtime_error("context write failed");}
                        std::filesystem::rename(temporary,context);last_context=now;
                    }catch(...){} // stale context makes camera output invalid
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            }
        });
    }
    std::shared_ptr<const Packet> Snapshot()const{return std::atomic_load(&latest_);}
    // Control thread only publishes an atomic value; no filesystem work here.
    void SetPhase(unsigned phase){phase_.store(phase);}
    ~Inbox(){stop_.store(true);if(thread_.joinable())thread_.join();}
};
}
