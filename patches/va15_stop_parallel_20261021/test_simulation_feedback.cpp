#include "simulation_feedback.hpp"
#include <cassert>
#include <iostream>
int main()
{
    using namespace va15_independent;
    Result r;r.input_valid=r.sequence_valid=r.fresh=r.initialized=r.estimate_valid_for_review=true;
    r.support_count=r.accepted_feet=4;r.load_candidate.fill(true);
    r.base_height=.52;r.planar_speed=.001;r.position_sigma=.002;r.velocity_sigma=.003;
    auto e=va15_sim_stop::Feedback(r);
    assert(!r.control_authorized&&!r.hardware_qualified);
    assert(!e.sport_fresh&&!VA15StopFeedbackValid(e)); // production gate stays shut
    assert(VA15SimEstimatedStopValid(e));
    assert(VA15SimEstimatedStopSettled(e,.01,.01,.01,.48,.15));
    assert(std::fabs(e.base_height-.516)<1.e-6&&std::fabs(e.planar_speed-.007)<1.e-6);
    for(int leg=0;leg<4;++leg)
    {
        auto partial=r;partial.load_candidate[leg]=false;partial.support_count=3;
        auto value=va15_sim_stop::Feedback(partial);
        assert(std::count(value.contacts.begin(),value.contacts.end(),false)==1);
        const int mapping[]={1,0,3,2};assert(!value.contacts[mapping[leg]]);
        assert(!VA15SimEstimatedStopSettled(value,.01,.01,.01,.48,.15));
    }
    for(int mode=0;mode<8;++mode)
    {
        auto bad=r;
        if(mode==0)bad.fresh=false;if(mode==1)bad.initialized=false;
        if(mode==2)bad.estimate_valid_for_review=false;if(mode==3)bad.sequence_valid=false;
        if(mode==4)bad.accepted_feet=1;if(mode==5)bad.base_height=NaN;
        if(mode==6)bad.position_sigma=.06;if(mode==7)bad.velocity_sigma=NaN;
        assert(!VA15SimEstimatedStopValid(va15_sim_stop::Feedback(bad)));
    }
    e.sport_fresh=true;assert(!VA15SimEstimatedStopValid(e));
    e=va15_sim_stop::Feedback(r);e.unavailable_reason="synthetic MuJoCo truth";
    assert(!VA15SimEstimatedStopValid(e));
    r.velocity_sigma=.025;assert(!VA15SimEstimatedStopSettled(va15_sim_stop::Feedback(r),.01,.01,.01,.48,.15));
    std::cout<<"PASS: simulation-only bridge, mapping, uncertainty, fail-closed and no sport spoofing\n";
}
