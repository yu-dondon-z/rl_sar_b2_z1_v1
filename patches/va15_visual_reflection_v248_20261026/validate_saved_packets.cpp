// Recorded/synthetic offline fixtures ONLY. Never initialize DDS or robot SDK.
#include "production/near_visual_io.hpp"
#include <cassert>
#include <iostream>
int main(int argc,char** argv) {
    if(argc!=2)return 1;
    std::ifstream input(argv[1]);if(!input)return 1;
    std::string line;unsigned count=0,accepted=0;
    while(std::getline(input,line)) {
        const auto fixture=YAML::Load(line);
        assert(fixture["offline_saved_image_fixture"].as<bool>(false));
        const auto packet=va15_visual::Parse(line,std::string(32,'a'));
        const bool expected=fixture["valid"].as<bool>();
        const auto publish=fixture["publish_monotonic_ns"].as<std::uint64_t>();
        const bool usable=va15_visual::Valid(packet,publish);
        if(usable!=expected) {
            std::cerr<<"Packet geometry mismatch: "<<fixture["frame"].as<std::string>()<<'\n';return 2;
        }
        if(usable) {
            assert(packet.path.size()==packet.path_count);
            assert(std::fabs(va15_visual::CurveU(packet,packet.look_v)-packet.look_u)<=2.);
            assert(std::fabs(va15_visual::CurveU(packet,packet.v)-packet.u)<=8.);
            ++accepted;
        }
        ++count;
    }
    if(!count)return 2;
    std::cout<<"PASS saved RGB packets="<<count<<" accepted="<<accepted
             <<"; original parser/anchor/geometry/freshness checks unchanged; no hardware initialized\n";
}
