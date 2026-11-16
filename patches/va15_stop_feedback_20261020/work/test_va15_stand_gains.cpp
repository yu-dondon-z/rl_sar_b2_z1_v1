#include "va15_stand_gains.hpp"
#include <cassert>
#include <iostream>
#include <limits>
int main() {
    std::vector<float> first(18,200.f),last(18,450.f);
    for(int i=9;i<=12;++i) {first[i]=300;last[i]=675;}
    for(int i:{8,13,14,15,16,17}) first[i]=last[i]=60;
    auto previous=first;
    for(int step=0;step<=400;++step) {
        const auto out=VA15BlendGains(first,last,step/400.f);
        for(int i=0;i<18;++i) {
            assert(out[i]>=previous[i] && out[i]<=last[i]);
            assert(std::fabs(out[i]-previous[i])<2.f);
        }
        previous=out;
    }
    assert(previous==last);
    assert(VA15BlendGains(first,last,-1)==first);
    assert(VA15BlendGains(first,last,2)==last);
    bool rejected=false;
    try{VA15BlendGains({},last,0);}catch(const std::exception&){rejected=true;}
    assert(rejected);rejected=false;first[0]=std::numeric_limits<float>::quiet_NaN();
    try{VA15BlendGains(first,last,0);}catch(const std::exception&){rejected=true;}
    assert(rejected);
    std::cout<<"PASS: stand gain ramp bounds, monotonic steps, unchanged arm stiffness and invalid data\n";
}
