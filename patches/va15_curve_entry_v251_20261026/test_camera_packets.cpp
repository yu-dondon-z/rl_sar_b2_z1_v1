// Saved-file frontend/parser fixtures ONLY. No SDK, DDS, sockets or commands.
#include "production/near_visual_io.hpp"
#include <cassert>
#include <fstream>
#include <iostream>

int main(int argc,char** argv) {
    if(argc!=2)return 2;
    std::ifstream file(argv[1]);std::string line;
    unsigned valid=0,rejected=0;
    while(std::getline(file,line)) {
        const auto packet=va15_visual::Parse(line,std::string(32,'a'));
        if(line.find("\"valid\": true")!=std::string::npos) {
            if(!va15_visual::Valid(packet,packet.publish_ns+1)) {
                std::cerr<<"Frontend/native mismatch at fixture "<<valid+rejected+1<<'\n';return 1;
            }
            ++valid;
        }else {assert(!packet.valid);++rejected;}
    }
    assert(valid>=10&&rejected>=2);
    std::cout<<"PASS saved-image native parser: valid="<<valid<<" rejected_short="<<rejected
        <<"; fixture timestamps only, not a motion acceptance\n";
}
