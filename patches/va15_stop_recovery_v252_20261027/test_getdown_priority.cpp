#include "production/priority_getdown.hpp"
#include <cassert>
#include <iostream>
#include <stdexcept>
int main() {
    va15_visual::Guide guide(true,true,true);guide.stage=va15_visual::Guide::Stage::Running;
    va15_visual::Output output{.15,.06,false};
    bool pending=false;unsigned visual_calls=0;
    const auto cycle=[&](bool pressed,bool simultaneous_start) {
        if(va15_priority::PreemptVisualForGetDown(pressed,pending,guide,output)) {
            pending=true;
            assert(output.x==0.&&output.yaw==0.&&output.stop);
            return; // exact native ordering: NO inbox, projection or rearm
        }
        ++visual_calls;
        if(simultaneous_start)throw std::runtime_error("start must not run while B pending");
    };
    cycle(true,true);
    // Releasing B does not cancel its intent. A missing camera or later T
    // must not enter the visual call path while the supported stop proceeds.
    guide.latest={};
    for(unsigned i=0;i<1000;++i)cycle(false,true);
    assert(pending&&visual_calls==0&&guide.stage==va15_visual::Guide::Stage::Stopped);
    pending=false;cycle(false,false);assert(visual_calls==1);
    va15_visual::Guide idle;output={};
    assert(va15_priority::PreemptVisualForGetDown(true,false,idle,output)&&output.stop);
    std::cout<<"PASS B/9 preempts simultaneous start and missing/stale camera work; pending request retained; no support/lowering authority\n";
}
