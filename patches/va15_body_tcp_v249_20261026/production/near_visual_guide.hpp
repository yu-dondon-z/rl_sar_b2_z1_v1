#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

// Supported long pixel curve + optional V2.4 flat-floor metric reference.
// This is NOT depth or hardware/contact qualification. Bare/default Guide
// retains the archived pixel-only acquisition contract for regression tests.
namespace va15_visual {
inline constexpr char CalibrationSHA[]="525a898c45693a2b00839a6d0bc2cc3e4c9d629f6687ff80dde9a850cbfdb4e0";
inline constexpr std::uint64_t MaximumAgeNs=500000000ULL,RequestNs=4000000000ULL;
inline constexpr double Speed=.15,YawLimit=.06,YawSlew=.04,BearingGain=1.2;
// Wider IMAGE-ray acquisition window for the archived 18/21 degree tape.
// Command magnitudes, freshness, association and stop budgets are unchanged.
inline constexpr double NearBearingLimit=.40,LookBearingLimit=.45;
inline constexpr double DisplayNearBearingLimit=.60,DisplayLookBearingLimit=.65;
inline constexpr double NominalLookV=.76*1080.,MaximumLookV=.82*1080.;
struct Packet {
    bool valid=false,session_matched=false;
    std::uint64_t sequence=0,capture_ns=0,publish_ns=0;
    double u=0.,v=0.,confidence=0.,width=0.,contrast=0.,rms=0.;
    double look_u=0.,look_v=0.,look_confidence=0.,path_span=0.;
    unsigned path_count=0;
    std::vector<std::array<double,2>> path;
    bool motion_compensated=false;
    double association_error_px=0.;
    bool image_received=false,endpoint_candidate=false;
    double endpoint_u=0.,endpoint_v=0.,endpoint_confidence=0.,endpoint_clear_pixels=0.;
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
        std::isfinite(Bearing(p.u,p.v))&&std::isfinite(p.look_v)&&
        p.look_v>=NominalLookV-.1&&p.look_v<=MaximumLookV&&
        std::isfinite(Bearing(p.look_u,p.look_v))&&
        std::isfinite(p.look_confidence)&&p.look_confidence>=.90&&p.look_confidence<=1.&&
        p.path_count>=25&&p.path_count<=128&&std::isfinite(p.path_span)&&p.path_span>=200.&&p.path_span<=1000.;
}
inline double SteeringBearing(const Packet& p) {
    return .6*Bearing(p.u,p.v)+.4*Bearing(p.look_u,p.look_v);
}
inline double CurveU(const Packet& p,double v) {
    for(std::size_t i=1;i<p.path.size();++i) {
        const auto& a=p.path[i-1];const auto& b=p.path[i];
        if(a[1]>=v&&v>=b[1]&&a[1]>b[1])
            return a[0]+(b[0]-a[0])*(a[1]-v)/(a[1]-b[1]);
    }
    return NAN; // no extrapolation or fabricated look point
}
inline double AssociatedLookChange(const Packet& p,const Packet& old) {
    if(std::fabs(p.look_v-old.look_v)<.1)
        return std::fabs(Bearing(p.look_u,p.look_v)-Bearing(old.look_u,old.look_v));
    // A variable look row on the SAME bend naturally has a different ray.
    // Compare observed curves at their COMMON row instead of rejecting that
    // change as a switch of physical tape. Missing common support fails.
    const double row=std::max(p.look_v,old.look_v);
    return std::fabs(Bearing(CurveU(p,row),row)-Bearing(CurveU(old,row),row));
}
inline bool InSteeringEnvelope(const Packet& p) {
    const double near=Bearing(p.u,p.v),look=Bearing(p.look_u,p.look_v);
    return std::isfinite(near)&&std::isfinite(look)&&
        std::fabs(near)<=NearBearingLimit&&std::fabs(look)<=LookBearingLimit;
}
struct Output {double x=0.,yaw=0.;bool stop=false;};
class Guide {
public:
    explicit Guide(bool repeat=false,bool sustained=false,bool tolerate=false):repeatable(repeat),continuous(sustained),tolerant(tolerate){}
    bool repeatable=false,continuous=false,tolerant=false;
    enum class Stage {Idle,Running,Stopped};
    Stage stage=Stage::Idle;
    const char* reason="waiting for current near-tape observations";
    Packet latest;
    unsigned qualified=0;
    std::uint64_t observed_sequence=0,observed_stamp=0,start_ns=0,deadline_ns=0;
    bool spent=false;
    unsigned sessions=0;
    double yaw=0.,course_slip=0.,metric_bearing=0.;
    bool metric_bearing_valid=false;
    double body_lateral=0.,body_heading=0.,yaw_reference=0.,visual_integral=0.;
    bool body_reference_valid=false,body_history_used=false;
    void ObserveBodyVelocity(double vx,double vy,double dt) {
        if(!std::isfinite(vx)||!std::isfinite(vy)||!std::isfinite(dt)||dt<=0.||dt>.02)return;
        // Diagnostic ONLY. The unqualified no-slip velocity estimate reversed
        // visual correction in the recorded real trial. It must not subtract
        // from the body/path controller or grant contact/pose authority.
        const double measured=vx>.06?std::clamp(std::atan2(vy,vx),-.30,.30):0.;
        course_slip+=dt/(.8+dt)*(measured-course_slip);
    }
    Packet seen;
    bool Envelope()const {
        return metric_bearing_valid?(std::isfinite(metric_bearing)&&std::fabs(metric_bearing)<=.45):InSteeringEnvelope(latest);
    }
    void Observe(const Packet& p,std::uint64_t now) {
        if(tolerant) {
            // One missed segmentation frame may reuse ONLY the last still-
            // fresh accepted frame. Its capture time is never refreshed.
            // Wrong sessions, clock reversals and modified duplicate packets
            // fail immediately. Expiry still stops at the original 500 ms.
            const bool metadata=p.session_matched&&p.sequence&&p.capture_ns&&
                p.capture_ns<=p.publish_ns&&p.publish_ns<=now&&now-p.capture_ns<=MaximumAgeNs;
            if(!metadata||p.sequence<seen.sequence||
               (p.sequence>seen.sequence&&p.capture_ns<=seen.capture_ns)) {qualified=0;latest={};return;}
            if(p.sequence==seen.sequence) {
                if(p.valid!=seen.valid||p.capture_ns!=seen.capture_ns||p.publish_ns!=seen.publish_ns||
                   p.u!=seen.u||p.v!=seen.v||p.look_u!=seen.look_u||p.look_v!=seen.look_v||
                   p.confidence!=seen.confidence||p.look_confidence!=seen.look_confidence||
                   p.width!=seen.width||p.contrast!=seen.contrast||p.rms!=seen.rms||
                   p.motion_compensated!=seen.motion_compensated||p.association_error_px!=seen.association_error_px||
                   p.path_count!=seen.path_count||p.path_span!=seen.path_span||p.path!=seen.path||
                   p.image_received!=seen.image_received||p.endpoint_candidate!=seen.endpoint_candidate||
                   p.endpoint_u!=seen.endpoint_u||p.endpoint_v!=seen.endpoint_v||
                   p.endpoint_confidence!=seen.endpoint_confidence||p.endpoint_clear_pixels!=seen.endpoint_clear_pixels) {qualified=0;latest={};}
                return;
            }
            seen=p;
            if(!p.valid&&stage==Stage::Running&&Valid(latest,now))return;
        }
        if(!Valid(p,now)) {qualified=0;latest=p;return;}
        if(p.sequence==observed_sequence) {
            // Re-reading one atomic file is not another independent frame.
            // Reusing its sequence with different contents is a rejection.
            if(p.capture_ns!=latest.capture_ns||p.publish_ns!=latest.publish_ns||
               p.u!=latest.u||p.v!=latest.v||p.confidence!=latest.confidence||
               p.width!=latest.width||p.contrast!=latest.contrast||p.rms!=latest.rms||
               p.look_u!=latest.look_u||p.look_v!=latest.look_v||p.look_confidence!=latest.look_confidence||
               p.motion_compensated!=latest.motion_compensated||p.association_error_px!=latest.association_error_px||
               p.path_count!=latest.path_count||p.path_span!=latest.path_span||p.path!=latest.path||
               p.image_received!=latest.image_received||p.endpoint_candidate!=latest.endpoint_candidate||
               p.endpoint_u!=latest.endpoint_u||p.endpoint_v!=latest.endpoint_v||
               p.endpoint_confidence!=latest.endpoint_confidence||p.endpoint_clear_pixels!=latest.endpoint_clear_pixels) {
                qualified=0;latest={};
            }
            return;
        }
        if(p.sequence<observed_sequence||p.capture_ns<=observed_stamp) {qualified=0;latest={};return;}
        if(observed_stamp&&p.capture_ns-observed_stamp>MaximumAgeNs)qualified=0;
        const bool verified_motion=p.motion_compensated&&std::isfinite(p.association_error_px)&&
            p.association_error_px>=0.&&p.association_error_px<=(tolerant?60.:12.);
        const double look_change=qualified?AssociatedLookChange(p,latest):0.;
        if(qualified&&(!std::isfinite(look_change)||
           std::fabs(Bearing(p.u,p.v)-Bearing(latest.u,latest.v))>(verified_motion?.08:.035)||
           look_change>(verified_motion?.10:.05)))qualified=0;
        observed_sequence=p.sequence;observed_stamp=p.capture_ns;latest=p;
        qualified=std::min(qualified+1,100u);
    }
    bool Ready(std::uint64_t now)const {
        return (!spent||repeatable)&&(stage==Stage::Idle||(repeatable&&stage==Stage::Stopped))&&qualified>=6&&Valid(latest,now)&&
            Envelope();
    }
    bool Start(std::uint64_t now,bool quiet,bool centered,bool healthy) {
        if((spent&&!repeatable)||(stage!=Stage::Idle&&!(repeatable&&stage==Stage::Stopped)))
        {reason="start refused: walking request already active/consumed";return false;}
        if(!healthy){reason="start refused: current state or motor communication health not ready";return false;}
        if(!Valid(latest,now)){reason="start refused: invalid/stale curve packet; see camera rejection reason";return false;}
        if(qualified<6){reason="start refused: need six distinct fresh associated camera frames";return false;}
        if(!Envelope()){reason="start refused: curve outside acquisition window; see metric/image angle readout";return false;}
        if(!centered){reason="start refused: remote invalid or sticks not neutral";return false;}
        if(!quiet){reason="start refused: wait for policy standby, quiet support and trial ARMED";return false;}
        spent=true;++sessions;stage=Stage::Running;start_ns=now;deadline_ns=continuous?0:now+RequestNs;yaw=visual_integral=0.;
        reason="visual body guidance requested; TCP handled by the selected deployment";return true;
    }
    void Stop(const char* why) {if(stage==Stage::Running){stage=Stage::Stopped;reason=why;}}
    Output Tick(std::uint64_t now,double dt,bool healthy,bool stop_key,bool sticks_centered) {
        if(stage!=Stage::Running)return {0.,0.,stage==Stage::Stopped};
        if(!std::isfinite(dt)||dt<=0.||dt>.02){Stop("invalid visual control interval");return {0,0,true};}
        if(deadline_ns&&now>=deadline_ns){Stop("four-second monotonic visual request deadline");return {0,0,true};}
        if(stop_key||!sticks_centered){Stop("operator stop or stick moved");return {0,0,true};}
        if(!healthy){Stop("current state/transport health lost");return {0,0,true};}
        if(!Valid(latest,now)||qualified<6){Stop("visual line invalid, changed, or stale");return {0,0,true};}
        if(!Envelope()){Stop("curve outside bounded steering envelope");return {0,0,true};}
        const double bearing=metric_bearing_valid?metric_bearing:SteeringBearing(latest);
        const bool body_valid=body_reference_valid&&std::isfinite(body_lateral)&&std::isfinite(body_heading);
        // Slowly remove persistent VISUAL cross-track bias without using the
        // no-slip velocity course as a steering command. Conditional integral
        // prevents windup while saturated; reset at every explicit start.
        const double proportional=body_valid?.70*std::atan2(body_lateral,.75)+.55*body_heading:BearingGain*bearing;
        if(body_valid) {
            const double candidate=std::clamp(visual_integral+.10*body_lateral*dt,-.20,.20);
            const double unsaturated=proportional+candidate;
            if(std::fabs(unsaturated)<=YawLimit||unsaturated*body_lateral<0.)visual_integral=candidate;
        }
        const double reference=proportional+(body_valid?visual_integral:0.);
        const double target=std::clamp(reference,-YawLimit,YawLimit);
        yaw_reference=target; // course_slip intentionally NOT applied
        const double filtered=yaw+dt/(.4+dt)*(target-yaw);
        yaw+=std::clamp(filtered-yaw,-YawSlew*dt,YawSlew*dt);
        return {Speed,yaw,false};
    }
};
}
