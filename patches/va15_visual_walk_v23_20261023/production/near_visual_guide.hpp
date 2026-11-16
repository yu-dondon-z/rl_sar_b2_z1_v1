#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

// Supported long pixel curve, near/lookahead bearing experiment. NOT a metric path, odometry,
// TCP controller, depth measurement, or hardware/contact qualification.
namespace va15_visual {
inline constexpr char CalibrationSHA[]="525a898c45693a2b00839a6d0bc2cc3e4c9d629f6687ff80dde9a850cbfdb4e0";
inline constexpr std::uint64_t MaximumAgeNs=500000000ULL,RequestNs=4000000000ULL;
inline constexpr double Speed=.15,YawLimit=.06,YawSlew=.04,BearingGain=1.2;
// Wider IMAGE-ray acquisition window for the archived 18/21 degree tape.
// Command magnitudes, freshness, association and stop budgets are unchanged.
inline constexpr double NearBearingLimit=.40,LookBearingLimit=.45;
struct Packet {
    bool valid=false,session_matched=false;
    std::uint64_t sequence=0,capture_ns=0,publish_ns=0;
    double u=0.,v=0.,confidence=0.,width=0.,contrast=0.,rms=0.;
    double look_u=0.,look_v=0.,look_confidence=0.,path_span=0.;
    unsigned path_count=0;
    bool motion_compensated=false;
    double association_error_px=0.;
};
inline double Bearing(double u,double v) {
    if(!std::isfinite(u)||!std::isfinite(v)||u<0.||u>=1920.||v<.55*1080.||v>.98*1080.)return NAN;
    const double x0=(u-934.705)/756.700,y0=(v-510.684)/752.123;
    double x=x0,y=y0;
    for(int n=0;n<8;++n) {
        const double r=x*x+y*y,radial=1.+.049696*r-.03070179*r*r-.04031327*r*r*r;
        if(!std::isfinite(radial)||radial<.2)return NAN;
        const double dx=2.*(-.00359919)*x*y+(-.00590063)*(r+2.*x*x);
        const double dy=(-.00359919)*(r+2.*y*y)+2.*(-.00590063)*x*y;
        x=(x0-dx)/radial;y=(y0-dy)/radial;
    }
    const double forward=.00651959*x-.02434965*y+.99968224;
    const double left=-.99983166*x-.01730499*y+.00609906;
    return forward>.2?std::atan2(left,forward):NAN;
}
inline bool Valid(const Packet& p,std::uint64_t now) {
    return p.valid&&p.session_matched&&p.sequence&&p.capture_ns&&p.publish_ns>=p.capture_ns&&
        p.publish_ns<=now&&p.capture_ns<=now&&now-p.capture_ns<=MaximumAgeNs&&
        std::isfinite(p.confidence)&&p.confidence>=.95&&p.confidence<=1.&&
        std::isfinite(p.width)&&p.width>=12.&&p.width<=180.&&
        std::isfinite(p.contrast)&&p.contrast>=.25&&p.contrast<=1.&&
        std::isfinite(p.rms)&&p.rms>=0.&&p.rms<=6.&&std::fabs(p.v-972.)<.1&&
        std::isfinite(Bearing(p.u,p.v))&&std::fabs(p.look_v-.76*1080.)<.1&&
        std::isfinite(Bearing(p.look_u,p.look_v))&&
        std::isfinite(p.look_confidence)&&p.look_confidence>=.90&&p.look_confidence<=1.&&
        p.path_count>=25&&p.path_count<=128&&std::isfinite(p.path_span)&&p.path_span>=200.&&p.path_span<=1000.;
}
inline double SteeringBearing(const Packet& p) {
    return .6*Bearing(p.u,p.v)+.4*Bearing(p.look_u,p.look_v);
}
inline bool InSteeringEnvelope(const Packet& p) {
    const double near=Bearing(p.u,p.v),look=Bearing(p.look_u,p.look_v);
    return std::isfinite(near)&&std::isfinite(look)&&
        std::fabs(near)<=NearBearingLimit&&std::fabs(look)<=LookBearingLimit;
}
struct Output {double x=0.,yaw=0.;bool stop=false;};
class Guide {
public:
    explicit Guide(bool repeat=false,bool sustained=false):repeatable(repeat),continuous(sustained){}
    bool repeatable=false,continuous=false;
    enum class Stage {Idle,Running,Stopped};
    Stage stage=Stage::Idle;
    const char* reason="waiting for current near-tape observations";
    Packet latest;
    unsigned qualified=0;
    std::uint64_t observed_sequence=0,observed_stamp=0,start_ns=0,deadline_ns=0;
    bool spent=false;
    unsigned sessions=0;
    double yaw=0.;
    void Observe(const Packet& p,std::uint64_t now) {
        if(!Valid(p,now)) {qualified=0;latest=p;return;}
        if(p.sequence==observed_sequence) {
            // Re-reading one atomic file is not another independent frame.
            // Reusing its sequence with different contents is a rejection.
            if(p.capture_ns!=latest.capture_ns||p.publish_ns!=latest.publish_ns||
               p.u!=latest.u||p.v!=latest.v||p.confidence!=latest.confidence||
               p.width!=latest.width||p.contrast!=latest.contrast||p.rms!=latest.rms||
               p.look_u!=latest.look_u||p.look_v!=latest.look_v||p.look_confidence!=latest.look_confidence||
               p.motion_compensated!=latest.motion_compensated||p.association_error_px!=latest.association_error_px||
               p.path_count!=latest.path_count||p.path_span!=latest.path_span) {
                qualified=0;latest={};
            }
            return;
        }
        if(p.sequence<observed_sequence||p.capture_ns<=observed_stamp) {qualified=0;latest={};return;}
        if(observed_stamp&&p.capture_ns-observed_stamp>MaximumAgeNs)qualified=0;
        const bool verified_motion=p.motion_compensated&&std::isfinite(p.association_error_px)&&
            p.association_error_px>=0.&&p.association_error_px<=12.;
        if(qualified&&(std::fabs(Bearing(p.u,p.v)-Bearing(latest.u,latest.v))>(verified_motion?.08:.035)||
           std::fabs(Bearing(p.look_u,p.look_v)-Bearing(latest.look_u,latest.look_v))>(verified_motion?.10:.05)))qualified=0;
        observed_sequence=p.sequence;observed_stamp=p.capture_ns;latest=p;
        qualified=std::min(qualified+1,100u);
    }
    bool Ready(std::uint64_t now)const {
        return (!spent||repeatable)&&(stage==Stage::Idle||(repeatable&&stage==Stage::Stopped))&&qualified>=6&&Valid(latest,now)&&
            InSteeringEnvelope(latest);
    }
    bool Start(std::uint64_t now,bool quiet,bool centered,bool healthy) {
        if((spent&&!repeatable)||(stage!=Stage::Idle&&!(repeatable&&stage==Stage::Stopped)))
        {reason="start refused: walking request already active/consumed";return false;}
        if(!healthy){reason="start refused: current state or motor communication health not ready";return false;}
        if(!Valid(latest,now)){reason="start refused: invalid/stale curve packet; see camera rejection reason";return false;}
        if(qualified<6){reason="start refused: need six distinct fresh associated camera frames";return false;}
        if(!InSteeringEnvelope(latest)){reason="start refused: image near/lookahead angle outside acquisition window; see angle readout";return false;}
        if(!centered){reason="start refused: remote invalid or sticks not neutral";return false;}
        if(!quiet){reason="start refused: wait for policy standby, quiet support and trial ARMED";return false;}
        spent=true;++sessions;stage=Stage::Running;start_ns=now;deadline_ns=continuous?0:now+RequestNs;yaw=0.;
        reason="near-visual request started; TCP unchanged";return true;
    }
    void Stop(const char* why) {if(stage==Stage::Running){stage=Stage::Stopped;reason=why;}}
    Output Tick(std::uint64_t now,double dt,bool healthy,bool stop_key,bool sticks_centered) {
        if(stage!=Stage::Running)return {0.,0.,stage==Stage::Stopped};
        if(!std::isfinite(dt)||dt<=0.||dt>.02){Stop("invalid visual control interval");return {0,0,true};}
        if(deadline_ns&&now>=deadline_ns){Stop("four-second monotonic visual request deadline");return {0,0,true};}
        if(stop_key||!sticks_centered){Stop("operator stop or stick moved");return {0,0,true};}
        if(!healthy){Stop("current state/transport health lost");return {0,0,true};}
        if(!Valid(latest,now)||qualified<6){Stop("visual line invalid, changed, or stale");return {0,0,true};}
        if(!InSteeringEnvelope(latest)){Stop("curve outside bounded steering envelope");return {0,0,true};}
        const double target=std::clamp(BearingGain*SteeringBearing(latest),-YawLimit,YawLimit);
        const double filtered=yaw+dt/(.4+dt)*(target-yaw);
        yaw+=std::clamp(filtered-yaw,-YawSlew*dt,YawSlew*dt);
        return {Speed,yaw,false};
    }
};
}
