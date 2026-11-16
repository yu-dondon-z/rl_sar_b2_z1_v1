#pragma once
#include "visual_front_end_stop.hpp"
#include "visual_tcp.hpp"

// A confirmed, finite endpoint is a different navigation source from live
// vision. Never renew an old camera timestamp, extrapolate observed crack
// points, or report the front-camera cap as rear-TCP completion. This module
// is an offline candidate, not a qualified hardware navigation controller.
namespace va15_visual_end {
class RearCompletion {
public:
    enum class Stage {Idle,Watching,Confirming,Approaching,Braking,Standing,Interrupted};
    Stage stage=Stage::Idle;
    unsigned frames=0,generation=0;
    const char* reason="waiting for an explicit visual session";
    va15_tcp::V3 endpoint=va15_tcp::V3::Zero(),tangent=va15_tcp::V3::UnitX();
    std::vector<va15_tcp::V3> observed_path;
    double remaining=0.,lateral=0.,height_error=0.,distance=0.,steering_integral=0.;
    static constexpr double Clearance=.09,ArrivalXY=.05,ArrivalZ=.05;
    bool Confirmed()const{return stage==Stage::Approaching||Requested();}
    bool Requested()const{return stage==Stage::Braking||stage==Stage::Standing;}
    bool Stood()const{return stage==Stage::Standing;}
    void Begin(unsigned pose_generation=0) {
        *this=RearCompletion();generation=pose_generation;cap_.Begin();
        stage=Stage::Watching;reason="watching for a repeatedly observed finite endpoint";
    }
    void Interrupt(const char* why) {
        if(stage==Stage::Idle||stage==Stage::Standing||stage==Stage::Interrupted)return;
        stage=Stage::Interrupted;reason=why;quiet_=arrival_=0.;
    }
    void Observe(const va15_visual::Packet& p,const va15_tcp::Pose& capture,
                 std::uint64_t now,bool associated,unsigned pose_generation) {
        if(stage!=Stage::Watching&&stage!=Stage::Confirming)return;
        if(pose_generation!=generation){Interrupt("pose generation changed; endpoint invalidated");return;}
        if(p.sequence==last_sequence_)return;
        va15_tcp::V3 point,direction;
        std::vector<va15_tcp::V3> path;
        bool metric=associated&&va15_visual::Valid(p,now)&&va15_tcp::ValidPose(capture)&&
            capture.ns==p.capture_ns&&p.path.size()>=25&&p.endpoint_candidate&&
            va15_tcp::Project(capture,{p.endpoint_u,p.endpoint_v},point);
        if(metric) {
            for(const auto& uv:p.path) {
                va15_tcp::V3 at;
                if(!va15_tcp::Project(capture,uv,at)||(!path.empty()&&(at-path.back()).norm()>.20)) {metric=false;break;}
                path.push_back(at);
            }
            if(metric) {
                direction=point-path[path.size()-2];direction.z()=0.;
                // The final row may nearly duplicate the preceding sample.
                if(direction.norm()<.005)direction=point-path[path.size()-3];
                direction.z()=0.;
                metric=direction.norm()>=.005&&(point-path.back()).norm()<.02;
                if(metric)direction.normalize();
            }
        }
        if(metric&&cap_.frames&&((point-anchor_).head<2>().norm()>.10||direction.dot(anchor_tangent_)<std::cos(.20))) {
            cap_.Begin(); // six spatially inconsistent caps are not a fixed end
        }
        if(metric&&!cap_.frames){anchor_=point;anchor_tangent_=direction;}
        cap_.Observe(p,now,metric);
        last_sequence_=p.sequence;frames=cap_.frames;
        if(cap_.stage==Stop::Stage::Interrupted){Interrupt(cap_.reason);return;}
        stage=frames?Stage::Confirming:Stage::Watching;
        if(cap_.Requested()&&metric) {
            endpoint=point;tangent=direction;observed_path=std::move(path);
            confirmed_ns_=now;stage=Stage::Approaching;
            reason="finite endpoint stored; front-camera cap is NOT TCP arrival";
        }
    }
    // Before confirmation the ordinary live guide owns the command. After
    // confirmation this finite-goal controller owns it, with fresh local pose,
    // measured arm FK, original health/manual gates and a finite progress budget.
    va15_visual::Output Tick(std::uint64_t now,double dt,const va15_tcp::Pose& pose,
                            const va15_tcp::V3& measured_tcp,bool tcp_valid,
                            unsigned pose_generation,bool healthy,bool stop,bool centered,
                            double previous_yaw,double prior_integral=0.) {
        if(stage!=Stage::Approaching)return {0.,0.,Requested()||stage==Stage::Interrupted};
        if(!healthy||stop||!centered||!std::isfinite(dt)||dt<=0.||dt>.02||
           !va15_tcp::ValidPose(pose)||now<pose.ns||now-pose.ns>100000000ULL||
           pose_generation!=generation||!tcp_valid||!measured_tcp.allFinite()||
           !std::isfinite(previous_yaw)||now<confirmed_ns_||now-confirmed_ns_>60000000000ULL) {
            const char* why=stop||!centered?"terminal approach: operator interruption":
                !healthy?"terminal approach: state/transport health lost":
                !std::isfinite(dt)||dt<=0.||dt>.02?"terminal approach: control interval invalid":
                !tcp_valid||!measured_tcp.allFinite()?"terminal approach: measured arm FK unavailable":
                now<confirmed_ns_||now-confirmed_ns_>60000000000ULL?"terminal approach: finite progress budget expired":
                "terminal approach: local pose/generation/steering source invalid";
            Interrupt(why);return {0.,0.,true};
        }
        Evaluate(measured_tcp);
        if(!initial_distance_set_) {
            initial_remaining_=remaining;initial_lateral_=lateral;initial_distance_set_=true;
            steering_integral=std::isfinite(prior_integral)?std::clamp(prior_integral,-.20,.20):0.;
        }
        // A rear TCP offset already present at cap confirmation cannot be
        // confused with new terminal divergence. Require a bounded initial
        // offset and bound subsequent growth separately while steering settles.
        if(remaining<-.12||remaining>initial_remaining_+.35||std::fabs(initial_lateral_)>.35||
           std::fabs(lateral)>std::max(.30,std::fabs(initial_lateral_)+.10)) {
            Interrupt("finite endpoint missed or diverging; not normal completion");return {0.,0.,true};
        }
        const bool at=distance<=ArrivalXY&&std::fabs(height_error)<=ArrivalZ;
        arrival_=at?arrival_+dt:0.;
        if(arrival_+1.e-9>=.10) {
            stage=Stage::Braking;reason="measured rear TCP reached finite endpoint; requesting supported stop";
            return {0.,0.,true};
        }
        // Use an actually observed segment while it brackets the body. After
        // the body passes the end, its finite carry pose is ahead of the crack
        // by the rear tool offset; that is a robot plan, NOT more crack points.
        const auto body_tangent=pose.q.conjugate()*tangent;
        double heading=std::atan2(body_tangent.y(),body_tangent.x());
        const va15_tcp::V3 body_end=pose.q.conjugate()*(endpoint-pose.p);
        // Cross-track of the finite carry-pose plan, separated from heading.
        // Using endpoint_body.y here after passing the end feeds the REAR
        // lever arm back into yaw with the opposite sign.
        const va15_tcp::V3 carry_delta=endpoint+.775*tangent-pose.p;
        double y=tangent.x()*carry_delta.y()-tangent.y()*carry_delta.x();bool bracket=false;
        for(std::size_t i=1;i<observed_path.size();++i) {
            const va15_tcp::V3 a=pose.q.conjugate()*(observed_path[i-1]-pose.p);
            const va15_tcp::V3 b=pose.q.conjugate()*(observed_path[i]-pose.p);
            if(b.x()-a.x()<1.e-6)continue;
            const double f=-a.x()/(b.x()-a.x());
            if(f<0.||f>1.)continue;
            y=(1.-f)*a.y()+f*b.y();heading=std::atan2(b.y()-a.y(),b.x()-a.x());bracket=true;break;
        }
        if(!bracket&&body_end.x()>0.&&observed_path.size()>1) {
            // Do not turn toward a very near first cached sample: its short
            // lever arm amplifies lateral error into a spurious sharp turn.
            // Interpolate ONLY where an actual finite segment spans 0.60 m.
            for(std::size_t i=1;i<observed_path.size();++i) {
                const va15_tcp::V3 a=pose.q.conjugate()*(observed_path[i-1]-pose.p);
                const va15_tcp::V3 b=pose.q.conjugate()*(observed_path[i]-pose.p);
                if(b.x()-a.x()<1.e-6)continue;
                const double f=(.60-a.x())/(b.x()-a.x());
                if(f<0.||f>1.)continue;
                const va15_tcp::V3 selected=(1.-f)*a+f*b;
                y=selected.y();heading=std::atan2(selected.y(),selected.x());break;
            }
        }
        if(!std::isfinite(heading)||std::fabs(heading)>.45||!std::isfinite(y)||std::fabs(y)>.30) {
            Interrupt("terminal path outside steering corridor");return {0.,0.,true};
        }
        // Do NOT feed rear-TCP lateral error into a forward-look yaw law:
        // for r_x<0, instantaneous lateral TCP yaw velocity is r_x*omega,
        // the opposite sign of a front lookahead. The failed v5 bench probe
        // confirmed that such a shortcut can worsen heading. Keep body/path
        // steering here; the original actor receives the finite TCP target.
        const double proportional=.70*std::atan2(y,.75)+.55*heading;
        const double next_integral=std::clamp(steering_integral+.10*y*dt,-.20,.20);
        if(std::fabs(proportional+next_integral)<=va15_visual::YawLimit||(proportional+next_integral)*y<0.)
            steering_integral=next_integral;
        const double desired=std::clamp(proportional+steering_integral,-va15_visual::YawLimit,va15_visual::YawLimit);
        const double filtered=previous_yaw+dt/(.4+dt)*(desired-previous_yaw);
        const double yaw=previous_yaw+std::clamp(filtered-previous_yaw,-va15_visual::YawSlew*dt,va15_visual::YawSlew*dt);
        // Slow only close to the MEASURED rear TCP, not at front-cap detection.
        const double speed=std::clamp(.40*std::max(0.,remaining),.10,va15_visual::Speed);
        return {speed,yaw,false};
    }
    void Park(double dt,bool healthy,bool original_ready_and_quiet,
              const va15_tcp::V3& measured_tcp,bool tcp_valid) {
        if(stage!=Stage::Braking)return;
        if(!healthy||!std::isfinite(dt)||dt<=0.||dt>.02||!tcp_valid||!measured_tcp.allFinite()) {
            Interrupt("invalid state during rear-TCP stop");return;
        }
        Evaluate(measured_tcp);
        const bool arrived=distance<=.07&&std::fabs(height_error)<=.06;
        braking_time_+=dt;quiet_=original_ready_and_quiet&&arrived?quiet_+dt:0.;
        if(quiet_+1.e-9>=.5) {stage=Stage::Standing;reason="rear TCP endpoint and original supported stand confirmed; holding";}
        else if(braking_time_>10.)Interrupt("stop not settled at rear endpoint; completion not accepted");
    }
private:
    Stop cap_;
    std::uint64_t confirmed_ns_=0,last_sequence_=0;
    va15_tcp::V3 anchor_=va15_tcp::V3::Zero(),anchor_tangent_=va15_tcp::V3::UnitX();
    double quiet_=0.,arrival_=0.,braking_time_=0.,initial_remaining_=0.,initial_lateral_=0.;
    bool initial_distance_set_=false;
    void Evaluate(const va15_tcp::V3& tcp) {
        const va15_tcp::V3 delta=endpoint-tcp;
        remaining=delta.dot(tangent);lateral=tangent.x()*delta.y()-tangent.y()*delta.x();
        distance=delta.head<2>().norm();height_error=tcp.z()-(endpoint.z()+Clearance);
    }
};
}
