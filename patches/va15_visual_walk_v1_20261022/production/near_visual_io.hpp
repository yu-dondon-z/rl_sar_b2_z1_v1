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

namespace va15_visual {
inline std::uint64_t NowNs() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline Packet Parse(const std::string& text,const std::string& nonce) {
    Packet p;
    if(text.size()>16384)return p;
    try {
        const auto n=YAML::Load(text);
        if(!n.IsMap()||n["schema"].as<std::string>("")!="va15_near_visual_v1"||
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
    }catch(...){return {};}
    return p;
}
// All file access/parsing runs in this owned worker, never in the 200 Hz
// control or 500 Hz motor writer. No sensors, sockets, publishers or commands.
class Inbox {
    std::atomic<bool> stop_{false};std::thread thread_;
    std::shared_ptr<const Packet> latest_=std::make_shared<const Packet>();
public:
    Inbox(const std::filesystem::path& directory,const std::string& nonce) {
        if(nonce.size()!=32||!std::all_of(nonce.begin(),nonce.end(),[](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='f');}))
            throw std::runtime_error("Invalid unique visual session id");
        if(!std::filesystem::is_directory(directory))throw std::runtime_error("Visual exchange directory missing");
        const auto path=directory/"near_guide.json";
        thread_=std::thread([this,path,nonce]{
            while(!stop_.load()) {
                Packet p;
                try {
                    std::error_code e;const auto size=std::filesystem::file_size(path,e);
                    if(!e&&size<=16384) {std::ifstream f(path);std::string data((std::istreambuf_iterator<char>(f)),{});p=Parse(data,nonce);}
                }catch(...){}
                std::atomic_store(&latest_,std::make_shared<const Packet>(p));
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            }
        });
    }
    std::shared_ptr<const Packet> Snapshot()const{return std::atomic_load(&latest_);}
    ~Inbox(){stop_.store(true);if(thread_.joinable())thread_.join();}
};
}
