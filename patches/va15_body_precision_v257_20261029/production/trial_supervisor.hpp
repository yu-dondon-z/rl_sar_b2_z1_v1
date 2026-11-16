#pragma once
#include <algorithm>
#include <cmath>

namespace va15_trial
{
// A supervisor's fixed-speed permission is an UPPER bound for an autonomous
// finite-goal request, not permission to accelerate a terminal slow-down.
// The archived blind fixed-speed experiment retains its trigger semantics.
inline double VisualForwardLimit(double permitted,double requested,bool visual) {
    if(!std::isfinite(permitted)||!std::isfinite(requested)||permitted<=0.)return 0.;
    return visual?std::min(permitted,std::max(0.,requested)):permitted;
}
// One positive-command trial per process. A deadline bounds the positive
// request, NOT measured stopping time/distance. No dynamic recovery promise.
class Supervisor
{
public:
    static constexpr double SpeedLimit=.15, RequestSeconds=4.;
    static constexpr unsigned long long RequestNanoseconds=4000000000ull;
    enum class Stage { Preview, WaitingNeutral, Armed, Forward, Stopping };
    Stage stage=Stage::Preview;
    const char* reason="preview only: walking disabled";
    bool enabled=false, spent=false,repeatable=false,continuous=false;
    double neutral=0., requested=0.;
    void Configure(bool motion,bool repeat=false,bool sustained=false)
    {enabled=motion;repeatable=repeat;continuous=sustained;Entry();}
    // Explicit event only. Settled support and a completed supported stop
    // must be supplied by the caller; healthy never comes from lost alone.
    bool Rearm(bool quiet,bool healthy,bool centered,bool stopped) {
        if(!enabled||!repeatable||!Stopping()||!quiet||!healthy||!centered||!stopped)return false;
        spent=false;requested=0.;neutral=.5;stage=Stage::Armed;
        reason="explicit visual restart after supported stop";return true;
    }
    void Entry()
    {
        neutral=0.;
        if(spent){stage=Stage::Stopping;reason="trial consumed; restart required";}
        else {stage=enabled?Stage::WaitingNeutral:Stage::Preview;reason=enabled?"center stick on quiet support":"preview only: walking disabled";}
    }
    bool Stopping() const {return stage==Stage::Stopping;}
    bool Forward() const {return stage==Stage::Forward;}
    void Stop(const char* why)
    {if(enabled){spent=true;stage=Stage::Stopping;reason=why;}}
    double Step(double dt,double x,double y,double yaw,bool quiet,bool healthy,bool stop_key)
    {
        if(!enabled)return 0.;
        if(Stopping())return 0.;
        if(!std::isfinite(dt)||dt<=0.||dt>.02||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw))
        {Stop("invalid time/input");return 0.;}
        if(stop_key){Stop("operator X/B/Space stop");return 0.;}
        const bool centered=std::fabs(x)<=.03&&std::fabs(y)<=.03&&std::fabs(yaw)<=.03;
        if(stage==Stage::WaitingNeutral)
        {
            neutral=centered&&quiet&&healthy?neutral+dt:0.;
            if(neutral+1.e-9>=.5){stage=Stage::Armed;reason=continuous?
                "armed: explicit continuous visual session until stop":"armed: one fixed 0.15 m/s target / maximum 4 s request";}
            return 0.;
        }
        if(stage==Stage::Armed)
        {
            if(!quiet||!healthy){stage=Stage::WaitingNeutral;neutral=0.;return 0.;}
            if(centered)return 0.;
            if(x<=.03||std::fabs(y)>.03||std::fabs(yaw)>.03)
            {stage=Stage::WaitingNeutral;neutral=0.;reason="forward only; center stick before retry";return 0.;}
            stage=Stage::Forward;spent=true;reason="restricted forward request";
        }
        if(!healthy){Stop("independent input/sequence/estimate health lost");return 0.;}
        if(x<=.03||std::fabs(y)>.03||std::fabs(yaw)>.03)
        {Stop("stick released or non-forward input");return 0.;}
        // Check the next control interval, so no interval extending beyond
        // the requested budget is issued. Never renew from a held stick.
        if(!continuous&&requested+dt>RequestSeconds+1.e-9)
        {Stop("positive request deadline reached");return 0.;}
        requested+=dt;
        // Dedicated experiment ONLY: forward input is a held trigger for the
        // disclosed 0.15 m/s target, not a hidden minimum in normal joystick
        // mode. The original actor-command acceleration ramp still applies.
        return SpeedLimit;
    }
};
}
