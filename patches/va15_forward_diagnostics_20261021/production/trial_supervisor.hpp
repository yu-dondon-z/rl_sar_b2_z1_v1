#pragma once
#include <algorithm>
#include <cmath>

namespace va15_trial
{
// One positive-command trial per process. A deadline bounds the positive
// request, NOT measured stopping time/distance. No dynamic recovery promise.
class Supervisor
{
public:
    static constexpr double SpeedLimit=.10, RequestSeconds=2.;
    enum class Stage { Preview, WaitingNeutral, Armed, Forward, Stopping };
    Stage stage=Stage::Preview;
    const char* reason="preview only: walking disabled";
    bool enabled=false, spent=false;
    double neutral=0., requested=0.;
    void Configure(bool motion) {enabled=motion;Entry();}
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
            if(neutral+1.e-9>=.5){stage=Stage::Armed;reason="armed: one forward request <=0.10 m/s / 2 s";}
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
        if(requested+dt>RequestSeconds+1.e-9)
        {Stop("positive request deadline reached");return 0.;}
        requested+=dt;
        return std::clamp(x,0.,SpeedLimit);
    }
};
}
