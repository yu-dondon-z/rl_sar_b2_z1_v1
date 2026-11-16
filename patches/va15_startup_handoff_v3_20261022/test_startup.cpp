#include "production/va15_startup_handoff.hpp"
#include <cassert>
#include <iostream>
#include <limits>

struct Fixture
{
    RobotState<float> state;
    RobotCommand<float> stand,policy;
    std::vector<float> lower=std::vector<float>(18,-2.3f),upper=std::vector<float>(18,1.6f),limits=std::vector<float>(18,200.f);
    Fixture()
    {
        state.motor_state.resize(18);stand.motor_command.resize(18);policy.motor_command.resize(18);
        for(int i=0;i<18;++i)
        {
            const bool calf=i>=9&&i<=12;
            state.motor_state.q[i]=calf?-1.60f:.05f;state.motor_state.dq[i]=.04f;
            stand.motor_command.q[i]=calf?-1.50f:0.f;stand.motor_command.kp[i]=calf?675.f:450.f;
            stand.motor_command.kd[i]=15.f;stand.motor_command.dq[i]=.01f;stand.motor_command.tau[i]=.2f;
            policy.motor_command.q[i]=calf?-1.28f:.02f;policy.motor_command.kp[i]=160.f;
            policy.motor_command.kd[i]=5.f;policy.motor_command.dq[i]=-.01f;policy.motor_command.tau[i]=.1f;
            if(i<4){lower[i]=-.6f;upper[i]=.6f;}
            if(calf){upper[i]=-.45f;limits[i]=320.f;}
        }
    }
    VA15StartupHandoff::Result Step(VA15StartupHandoff& h,RobotCommand<float>& c,bool frame=true,bool health=true,bool support=true,bool stop=false,double dt=.005)
    {return h.Step(state,c,stand,frame,health,support,stop,dt,lower,upper,limits);}
};

static bool Same(const RobotCommand<float>& a,const RobotCommand<float>& b)
{return a.motor_command.q==b.motor_command.q&&a.motor_command.dq==b.motor_command.dq&&a.motor_command.kp==b.motor_command.kp&&a.motor_command.kd==b.motor_command.kd&&a.motor_command.tau==b.motor_command.tau;}

int main()
{
    using H=VA15StartupHandoff;
    Fixture f;H h;assert(h.Begin(f.stand));
    for(int i=0;i<3;++i){auto out=f.policy;assert(f.Step(h,out,false)==H::Result::Held);assert(Same(out,f.stand));}
    auto out=f.policy;assert(f.Step(h,out)==H::Result::Applied);
    assert(h.ever_applied&&h.initial_pd_error<1.e-3f);
    for(int i:H::Legs)
    {
        const auto& old=f.stand.motor_command;const auto& c=out.motor_command;
        const float q=f.state.motor_state.q[i],dq=f.state.motor_state.dq[i];
        assert(std::fabs(old.kp[i]*(old.q[i]-q)+old.kd[i]*(old.dq[i]-dq)+old.tau[i]-
            (c.kp[i]*(c.q[i]-q)+c.kd[i]*(c.dq[i]-dq)+c.tau[i]))<1.e-3f);
        assert(c.tau[i]==f.policy.motor_command.tau[i]); // no double gravity torque
    }
    for(int i:{8,13,14,15,16,17})assert(out.motor_command.q[i]==f.policy.motor_command.q[i]);
    for(int i=0;i<301;++i){out=f.policy;f.Step(h,out);}
    assert(h.stage==H::Stage::Complete);assert(Same(out,f.policy));
    // Verify the blend against its PD law at CHANGING q/dq, not just the
    // initial state. A frozen first-frame offset would fail this test.
    {
        Fixture moving;H join;assert(join.Begin(moving.stand));
        for(int step=0;step<=300;++step)
        {
            const float p=std::min(1.,join.elapsed/join.release_seconds);
            const float remaining=1.f-p*p*(3.f-2.f*p);
            for(int i:H::Legs)
            {
                moving.state.motor_state.q[i]+=.0001f*std::sin(.1f*step+i);
                moving.state.motor_state.dq[i]=.02f*std::cos(.05f*step+i);
            }
            auto command=moving.policy;
            assert(moving.Step(join,command)!=H::Result::Refused);
            for(int i:H::Legs)
            {
                const float q=moving.state.motor_state.q[i],dq=moving.state.motor_state.dq[i];
                const auto& a=moving.stand.motor_command;const auto& b=moving.policy.motor_command;
                const auto& c=command.motor_command;
                const float desired=remaining*(a.kp[i]*(a.q[i]-q)+a.kd[i]*(a.dq[i]-dq)+a.tau[i])+
                    (1.f-remaining)*(b.kp[i]*(b.q[i]-q)+b.kd[i]*(b.dq[i]-dq)+b.tau[i]);
                const float actual=c.kp[i]*(c.q[i]-q)+c.kd[i]*(c.dq[i]-dq)+c.tau[i];
                assert(std::fabs(actual-desired)<1.e-3f);
            }
        }
    }
    H timeout;assert(timeout.Begin(f.stand));
    for(int i=0;i<21;++i){out=f.policy;f.Step(timeout,out,false);assert(Same(out,f.stand));}
    assert(timeout.stage==H::Stage::Refused&&!timeout.ever_applied);
    H cancel;assert(cancel.Begin(f.stand));out=f.policy;
    assert(f.Step(cancel,out,false,true,true,true)==H::Result::Cancelled);assert(Same(out,f.stand));
    for(int fail=0;fail<8;++fail)
    {
        Fixture bad;H x;assert(x.Begin(bad.stand));out=bad.policy;
        if(fail==0)bad.state.motor_state.q[0]=std::numeric_limits<float>::quiet_NaN();
        if(fail==1)out.motor_command.kp[0]=0.f;
        if(fail==2)out.motor_command.q[1]=std::numeric_limits<float>::quiet_NaN();
        if(fail==3)bad.upper[0]=-.2f; // equivalent target would be outside existing bounds
        if(fail==4)bad.limits[0]=1.f;
        const auto r=bad.Step(x,out,true,fail!=5,fail!=6,false,fail==7?.1:.005);
        assert(r==H::Result::Refused);assert(Same(out,bad.stand)); // no partial leg update
    }
    // A refused blending frame retains the LAST applied command, not the
    // original standing posture or an unfiltered arm command.
    H mid;assert(mid.Begin(f.stand));out=f.policy;f.Step(mid,out);
    auto last=out;last.motor_command.q[14]=-.73f;
    auto invalid=f.policy;invalid.motor_command.q[0]=100.f;
    assert(mid.Step(f.state,invalid,last,true,true,true,false,.005,f.lower,f.upper,f.limits)==H::Result::Refused);
    assert(Same(invalid,last));
    std::cout<<"PASS complete-frame waiting, first-frame PD continuity, live-state convex PD crossfade, unchanged FF/arms, fade, cancellation, timeout and atomic refusal\n";
}
