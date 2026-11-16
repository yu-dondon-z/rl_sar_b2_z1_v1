// Geometry-only review of saved/recompressed camera images. No SDK clients,
// IPC Inbox, publishers, motion supervisor or controller is constructed.
#include "production/near_visual_io.hpp"
#include <cassert>
#include <filesystem>
#include <iostream>

int main(int argc,char** argv) {
    try {
        if(argc!=2)throw std::runtime_error("Offline geometry artifact directory required");
        unsigned valid=0,invalid=0;
        for(const auto& item:std::filesystem::directory_iterator(argv[1])) {
            if(item.path().filename().string().find("_geometry.json")==std::string::npos)continue;
            auto review=YAML::LoadFile(item.path().string());
            assert(review["schema"].as<std::string>()=="va15_offline_geometry_review_v1");
            assert(!review["motion_permission"].as<bool>()&&!review["timing_budget_qualified"].as<bool>());
            const auto original=review["original_packet"],recomputed=review["recomputed_packet"];
            for(const char* key:{"sequence","capture_monotonic_ns","publish_monotonic_ns"})
                assert(original[key].as<std::uint64_t>()==recomputed[key].as<std::uint64_t>());
            const auto nonce=original["session_id"].as<std::string>();
            assert(nonce==recomputed["session_id"].as<std::string>());
            auto packet=va15_visual::Parse(YAML::Dump(recomputed),nonce);
            const auto now=recomputed["publish_monotonic_ns"].as<std::uint64_t>();
            const bool expected=review["expected_geometry_valid"].as<bool>();
            assert(va15_visual::Valid(packet,now)==expected);
            if(expected) {
                ++valid;
                assert(!va15_visual::Valid(packet,now-1)); // future publication
                assert(!va15_visual::Valid(packet,packet.capture_ns+va15_visual::MaximumAgeNs+1));
                assert(!va15_visual::Valid(va15_visual::Parse(YAML::Dump(recomputed),"wrong-session"),now));
                auto bad=YAML::Clone(recomputed);bad["path_points_uv"][1][0]=10.;
                assert(!va15_visual::Valid(va15_visual::Parse(YAML::Dump(bad),nonce),now));
            }else ++invalid;
            // A review wrapper can NEVER masquerade as a live camera packet.
            assert(!va15_visual::Parse(YAML::Dump(review),nonce).valid);
        }
        assert(valid>0&&invalid>0);
        std::cout<<"PASS geometry-only artifacts: supported="<<valid<<", correctly rejected="<<invalid
            <<"; original clocks preserved, stale/future/session/gap guards unchanged; NO motion permission\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
