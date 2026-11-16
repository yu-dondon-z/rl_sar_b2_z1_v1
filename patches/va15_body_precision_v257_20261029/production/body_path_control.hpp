#pragma once
#include "visual_tcp.hpp"

namespace va15_body_path {
struct Output {double x=0.,y=0.,yaw=0.;bool valid=false,underbody=false;};
class Controller {
public:
    double yaw=0.,path_error=0.,path_heading=0.,curvature=0.;
    double visual_drift=0.,previous_error=0.,error_rate=0.;
    bool initialized=false,previous_under=false;
    void Reset(){*this=Controller();}
    Output Step(const va15_tcp::Pose& pose,const va15_tcp::Tracker& tracker,double dt) {
        if(!va15_tcp::ValidPose(pose)||!std::isfinite(dt)||dt<=0.||dt>.02)return {};
        double bearing=0.;va15_tcp::Tracker::ForwardReference preview;
        if(!tracker.Guidance(pose,bearing,&preview))return {};
        va15_tcp::V3 point;double heading=0.;
        const bool under=tracker.Station(pose,0.,point,heading);
        if(!under) {
            point=preview.point_body;va15_tcp::V3 unused;
            if(!tracker.Station(pose,point.x(),unused,heading)) {
                const auto a=pose.q.conjugate()*(tracker.live.near-pose.p);
                const auto b=pose.q.conjugate()*(tracker.live.look-pose.p);
                heading=std::atan2(b.y()-a.y(),b.x()-a.x());
            }
        }
        // Preview tangent error is explicitly a model for steering while the
        // underbody path is unobserved. It is NOT rear-TCP/foot-clearance data.
        path_error=under?point.y():point.y()-point.x()*std::tan(heading);
        path_heading=heading;curvature=0.;
        va15_tcp::V3 a,b;double ha=0.,hb=0.;
        if(under&&tracker.Station(pose,-.20,a,ha)&&tracker.Station(pose,.20,b,hb))
            curvature=std::clamp(std::atan2(std::sin(hb-ha),std::cos(hb-ha))/std::max(.2,(b-a).head<2>().norm()),-.5,.5);
        if(initialized&&under==previous_under) {
            const double raw_rate=(path_error-previous_error)/dt;
            error_rate+=dt/(.6+dt)*(raw_rate-error_rate);
            // Estimate persistent lateral drift from the change of the
            // VISUAL path error, not no-slip odometry course. This is a
            // command bias estimate and never supplies contact authority.
            const double drift=std::clamp(error_rate-.15*std::sin(heading),-.04,.04);
            visual_drift+=dt/(1.+dt)*(drift-visual_drift);
        } else error_rate=0.;
        initialized=true;previous_error=path_error;previous_under=under;
        const double desired_heading=heading+std::atan2(.5*path_error+visual_drift,.15);
        const double requested=std::clamp(.8*desired_heading+.15*curvature,-.06,.06);
        const double filtered=yaw+dt/(.20+dt)*(requested-yaw);
        yaw+=std::clamp(filtered-yaw,-.04*dt,.04*dt);
        return {.15,0.,yaw,true,under};
    }
};
}
