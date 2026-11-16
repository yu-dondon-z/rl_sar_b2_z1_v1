#pragma once
#include "rl_sdk.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

// Isolated V3 candidate. A requested PD-equivalent join, NOT measured torque
// control or hardware qualification. No SDK, action history or inference calls.
class VA15StartupHandoff
{
public:
    enum class Stage { Idle, WaitingFirstFrame, Blending, Complete, Cancelled, Refused };
    enum class Result { Held, Applied, Finished, Cancelled, Refused };
    static constexpr double WaitSeconds=.10, FadeSeconds=1.50;
    inline static constexpr std::array<int,12> Legs{{0,1,2,3,4,5,6,7,9,10,11,12}};
    Stage stage=Stage::Idle;
    const char* reason="not requested";
    double waited=0., elapsed=0.;
    float initial_pd_error=0.f;
    bool ever_applied=false;

    void Reset() {*this=VA15StartupHandoff();}
    bool Active() const {return stage==Stage::WaitingFirstFrame||stage==Stage::Blending;}
    bool Waiting() const {return stage==Stage::WaitingFirstFrame;}
    bool Begin(const RobotCommand<float>& command)
    {
        Reset();
        if(!ValidCommand(command)) {stage=Stage::Refused;reason="invalid standing command";return false;}
        entry=command;stage=Stage::WaitingFirstFrame;reason="holding complete stand until first policy frame";
        return true;
    }

    Result Step(const RobotState<float>& state, RobotCommand<float>& candidate,
        const RobotCommand<float>& previous, bool new_frame, bool healthy,
        bool first_support, bool stop_requested, double dt,
        const std::vector<float>& lower,const std::vector<float>& upper,
        const std::vector<float>& torque_limits)
    {
        if(stage==Stage::Complete||stage==Stage::Idle)return Result::Finished;
        if(stage==Stage::Cancelled) {candidate=previous;return Result::Cancelled;}
        if(stage==Stage::Refused) {candidate=previous;return Result::Refused;}
        const auto refuse=[&](const char* why){stage=Stage::Refused;reason=why;candidate=previous;return Result::Refused;};
        if(!std::isfinite(dt)||dt<=0.||dt>.02||!healthy||!ValidCommand(previous)||
           !Vector(state.motor_state.q)||!Vector(state.motor_state.dq)||
           !Vector(lower)||!Vector(upper)||!Vector(torque_limits))
            return refuse("invalid startup time/state/source/config; no new target applied");
        for(int i:Legs)
            if(lower[i]>=upper[i]||torque_limits[i]<=0.f)return refuse("invalid startup envelope");

        if(Waiting())
        {
            if(stop_requested)
            {
                candidate=previous;stage=Stage::Cancelled;reason="stop before first frame; standing frame retained";
                return Result::Cancelled;
            }
            if(!new_frame)
            {
                waited+=dt;candidate=entry;
                if(waited>WaitSeconds+1.e-9)return refuse("first policy frame timeout; standing frame retained");
                return Result::Held;
            }
            if(!first_support)return refuse("first policy frame requires current quiet supported stand");
        }
        if(!ValidCommand(candidate))return refuse("nonfinite or invalid policy command");
        RobotCommand<float> next=candidate;
        std::array<float,18> proposed_offset=offset;
        const bool first=Waiting();
        float pd_error=0.f;
        const float progress=static_cast<float>(std::min(1.,elapsed/FadeSeconds));
        const float remaining=1.f-progress*progress*(3.f-2.f*progress);
        for(int i:Legs)
        {
            auto& c=next.motor_command;
            const float q=state.motor_state.q[i],dq=state.motor_state.dq[i];
            // Preserve loaded stand stiffness on the first frame, then join
            // BOTH gains to the original moving PD. Dropping Kp immediately
            // while only compensating position produced a large extension
            // offset and an unwanted upward excursion in paired simulation.
            c.kp[i]=remaining*entry.motor_command.kp[i]+(1.f-remaining)*c.kp[i];
            const float kd=remaining*entry.motor_command.kd[i]+(1.f-remaining)*c.kd[i];
            if(first)
            {
                const auto& old=previous.motor_command;
                const float old_pd=old.kp[i]*(old.q[i]-q)+old.kd[i]*(old.dq[i]-dq)+old.tau[i];
                if(!std::isfinite(old_pd)||std::fabs(old_pd)>.8f*torque_limits[i])
                    return refuse("standing requested PD outside bounded startup envelope");
                // Preserve TOTAL requested PD+FF at this SAME q/dq. No tau_est
                // feedback and no added gravity/feedforward torque are used.
                const float equivalent=q+(old_pd-kd*(c.dq[i]-dq)-c.tau[i])/c.kp[i];
                proposed_offset[i]=equivalent-c.q[i];
            }
            c.q[i]+=remaining*proposed_offset[i];
            c.kd[i]=kd;
            if(!std::isfinite(c.q[i])||c.q[i]<lower[i]||c.q[i]>upper[i])
                return refuse("startup-adjusted target outside unchanged leg envelope");
            if(first)
            {
                const auto& old=previous.motor_command;
                const float old_pd=old.kp[i]*(old.q[i]-q)+old.kd[i]*(old.dq[i]-dq)+old.tau[i];
                const float new_pd=c.kp[i]*(c.q[i]-q)+c.kd[i]*(c.dq[i]-dq)+c.tau[i];
                pd_error=std::max(pd_error,std::fabs(old_pd-new_pd));
            }
        }
        // Atomic whole-frame commit only after ALL 12 legs pass validation.
        candidate=std::move(next);offset=proposed_offset;
        if(first){initial_pd_error=pd_error;ever_applied=true;stage=Stage::Blending;reason="requested-PD-continuous first policy frame";}
        elapsed=std::min(FadeSeconds,elapsed+dt);
        if(progress>=1.f)
        {stage=Stage::Complete;reason="startup offset released; original policy gains/targets unchanged";return Result::Finished;}
        return Result::Applied;
    }

private:
    RobotCommand<float> entry;
    std::array<float,18> offset{};
    static bool Vector(const std::vector<float>& value)
    {return value.size()==18&&std::all_of(value.begin(),value.end(),[](float v){return std::isfinite(v);});}
    static bool ValidCommand(const RobotCommand<float>& value)
    {
        const auto& c=value.motor_command;
        if(!Vector(c.q)||!Vector(c.dq)||!Vector(c.kp)||!Vector(c.kd)||!Vector(c.tau))return false;
        for(int i:Legs)if(c.kp[i]<=0.f||c.kd[i]<0.f)return false;
        return true;
    }
};

