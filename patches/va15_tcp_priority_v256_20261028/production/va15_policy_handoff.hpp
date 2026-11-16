#pragma once

#include "rl_sdk.hpp"
#include "va15_stand_gains.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

// A bounded stand -> actor join. No sensor inference, motor writes, or change
// to the actor/raw-history/EMA contract. Until a NEW actor frame is available
// the exact previous command is held. Once available, switch to the trained
// gains with a torque-equivalent reference, then release that reference offset.
class VA15PolicyHandoff
{
public:
    bool pending=false, joining=false, failed=false;
    double elapsed=0.;
    static constexpr double duration=1.5;
    const char* reason="idle";
    RobotCommand<float> held;
    std::vector<float> seed_q, seed_dq, seed_tau, first_policy_q;

    void Reset() { *this=VA15PolicyHandoff(); }
    void Begin(const RobotCommand<float>& previous)
    {
        Reset();held=previous;pending=true;reason="holding stand until first new actor frame";
    }
    bool Active() const { return pending||joining; }

    // Atomic output: an invalid candidate is never partially applied. On
    // failure, retain the LAST valid command and let the existing stop owner
    // catch it; this helper cannot grant support, restart, or lower B2.
    bool Apply(const RobotState<float>& state,RobotCommand<float>& output,
               bool new_frame,double dt,const std::vector<float>& lower,
               const std::vector<float>& upper,const std::vector<float>& limits)
    {
        if(failed) {output=held;return false;}
        if(!Active())return true;
        if(!Valid(state.motor_state.q)||!Valid(state.motor_state.dq)||
           !ValidCommand(held)||!ValidCommand(output)||!Valid(lower)||!Valid(upper)||
           !Valid(limits)||!std::isfinite(dt)||dt<=0.||dt>.02)
            return Fail(output,"invalid handoff state/config/time");
        if(pending&&!new_frame) {output=held;return true;}
        if(pending)
        {
            seed_q=state.motor_state.q;seed_dq=state.motor_state.dq;
            seed_tau.assign(18,0.f);first_policy_q=output.motor_command.q;
            for(int i:VA15LegIndices)
            {
                const auto& c=held.motor_command;
                seed_tau[i]=c.kp[i]*(c.q[i]-seed_q[i])+
                    c.kd[i]*(c.dq[i]-seed_dq[i])+c.tau[i];
                if(c.kp[i]<=0.f||c.kd[i]<0.f||limits[i]<=0.f||lower[i]>=upper[i]||
                   std::fabs(c.dq[i])>1.e-6f||std::fabs(c.tau[i])>1.e-6f||
                   !std::isfinite(seed_tau[i])||std::fabs(seed_tau[i])>.8f*limits[i])
                    return Fail(output,"unbounded/non-static standing effort; motion refused");
            }
            entry=held;pending=false;joining=true;
        }
        auto candidate=output;
        const float p=std::clamp(float(elapsed/duration),0.f,1.f);
        const float w=1.f-p*p*(3.f-2.f*p);
        for(int i:VA15LegIndices)
        {
            auto& c=candidate.motor_command;
            // Both endpoints have zero desired velocity/feedforward. Do not
            // silently discard a different actuator contract.
            if(c.kp[i]<=0.f||c.kd[i]<0.f||std::fabs(c.dq[i])>1.e-6f||std::fabs(c.tau[i])>1.e-6f)
                return Fail(output,"non-position actor contract");
            // Do NOT run the actor's full gait excursions behind stand gains
            // (450/675): offline closed-loop testing rejects that variant.
            // Use trained 160/5 immediately, compensating the reference rather
            // than multiplying the moving actor's excursions by higher Kp.
            // With changed gains, recover the reference giving the original
            // support effort at the seed state. A position-only old.q offset
            // loses 76% of knee support when 675 -> 160, despite equal q_des.
            const float equivalent=seed_q[i]+(seed_tau[i]+c.kd[i]*seed_dq[i])/c.kp[i];
            c.q[i]+=w*(equivalent-first_policy_q[i]);
            const float effort=c.kp[i]*(c.q[i]-state.motor_state.q[i])-
                c.kd[i]*state.motor_state.dq[i];
            if(!std::isfinite(c.q[i])||c.q[i]<lower[i]||c.q[i]>upper[i]||
               !std::isfinite(effort)||std::fabs(effort)>.8f*limits[i])
                return Fail(output,"handoff outside existing target/effort envelope");
        }
        // Only leg commands are joined; the independent arm handoff remains
        // responsible for Z1, including when the first actor frame is late.
        output=candidate;held=output;
        elapsed=std::min(duration,elapsed+dt);
        if(p>=1.f) {joining=false;reason="torque-continuous handoff completed";}
        else reason="torque-equivalent reference handoff at trained gains";
        return true;
    }
private:
    RobotCommand<float> entry;
    static bool Valid(const std::vector<float>& v)
    {return v.size()==18&&std::all_of(v.begin(),v.end(),[](float x){return std::isfinite(x);});}
    static bool ValidCommand(const RobotCommand<float>& x)
    {const auto& c=x.motor_command;return Valid(c.q)&&Valid(c.dq)&&Valid(c.kp)&&Valid(c.kd)&&Valid(c.tau);}
    bool Fail(RobotCommand<float>& output,const char* why)
    {failed=true;pending=joining=false;reason=why;output=held;return false;}
};
