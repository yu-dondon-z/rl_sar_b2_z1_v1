#include "va15_action_state.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
    VA15ActionState state; std::vector<float> raw(18,10.f),obs(18,0.f);
    auto a=state.Update(raw,obs,.2f,5.f,true);
    assert(obs[0]==5.f && std::fabs(a[0]-1.f)<1e-6f);
    raw.assign(18,-5.f);a=state.Update(raw,obs,.2f,5.f,false);
    assert(obs[0]==-5.f && std::fabs(a[0]+.2f)<1e-6f);
    a=state.Update(raw,obs,.2f,5.f,true);
    assert(std::fabs(a[0]+1.f)<1e-6f);
    bool failed=false;raw[0]=std::numeric_limits<float>::quiet_NaN();
    try {state.Update(raw,obs,.2f,5.f,false);} catch(const std::exception&){failed=true;}
    assert(failed);
    std::cout<<"PASS: VA15 bounded raw history, independent EMA, reset and invalid output\n";
}
