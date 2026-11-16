#include "production/trial_supervisor.hpp"
#include "production/trial_feedback.hpp"
#include <cassert>
#include <iostream>
using va15_trial::Supervisor;
static void Arm(Supervisor& s)
{for(int i=0;i<100;++i)assert(s.Step(.005,0,0,0,true,true,false)==0.);assert(s.stage==Supervisor::Stage::Armed);}
int main()
{
    Supervisor s;s.Configure(false);
    for(int i=0;i<1000;++i)assert(s.Step(.005,.5,0,0,true,true,false)==0.);
    s.Configure(true);
    for(int i=0;i<200;++i)assert(s.Step(.005,.5,0,0,true,true,false)==0.);
    Arm(s);
    for(int i=0;i<400;++i)assert(s.Step(.005,.5,0,0,true,true,false)<=.10);
    assert(std::fabs(s.requested-2.)<1.e-9);
    assert(s.Step(.005,.5,0,0,true,true,false)==0.&&s.Stopping());
    s.Entry();assert(s.Stopping());
    for(int i=0;i<200;++i)assert(s.Step(.005,.5,0,0,true,true,false)==0.);
    for(int how=0;how<7;++how)
    {
        Supervisor t;t.Configure(true);Arm(t);
        assert(t.Step(.005,.1,0,0,true,true,false)>.0);
        const double nan=std::numeric_limits<double>::quiet_NaN();
        assert(t.Step(how==4?nan:.005,how==0?0.:how==5?nan:.1,
            how==1?.1:0.,how==2?.1:0.,true,how!=3,how==6)==0.);
        assert(t.Stopping());t.Entry();assert(t.Stopping());
    }
    Supervisor y;y.Configure(true);Arm(y);
    assert(y.Step(.005,0,.1,0,true,true,false)==0.&&!y.Forward());
    assert(y.Step(.005,.1,0,0,true,true,false)==0.); // held request cannot bypass neutral
    va15_independent::Result r;
    assert(!va15_trial::FeedbackValid(va15_trial::Feedback(r)));
    r.fresh=r.input_valid=r.sequence_valid=r.initialized=r.estimate_valid_for_review=true;
    r.accepted_feet=r.support_count=4;r.load_candidate={true,false,true,false};
    r.base_height=.52;r.planar_speed=.001;r.position_sigma=.001;r.velocity_sigma=.001;
    auto f=va15_trial::Feedback(r);
    assert(!f.sport_fresh&&!VA15StopFeedbackValid(f));
    assert(va15_trial::EstimatedValid(f)&&va15_trial::HealthyFeedback(f));
    assert(!f.contacts[0]&&f.contacts[1]&&!f.contacts[2]&&f.contacts[3]);
    for(float v:f.raw_dds)assert(std::isnan(v));
    assert(!va15_trial::Settled(f,0,0,0,.48,.15));
    r.load_candidate.fill(true);f=va15_trial::Feedback(r);
    assert(va15_trial::Settled(f,0,0,0,.48,.15));
    r.estimate_valid_for_review=false;r.support_count=0;
    f=va15_trial::Feedback(r);assert(va15_trial::HealthyFeedback(f)&&!va15_trial::EstimatedValid(f));
    r.input_valid=false;assert(!va15_trial::HealthyFeedback(va15_trial::Feedback(r)));
    r.input_valid=true;r.velocity_sigma=.15;
    assert(!va15_trial::HealthyFeedback(va15_trial::Feedback(r)));
    static_assert(!va15_independent::Result::control_authorized);
    static_assert(!va15_independent::Result::hardware_qualified);
    std::cout<<"PASS preview, held-stick, bounded request, release/X/B, fault latch, no rearm, source separation\n";
}
