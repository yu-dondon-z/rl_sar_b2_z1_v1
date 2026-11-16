#pragma once
#include "visual_tcp.hpp"
#include <cstdint>
#include <limits>
#include <utility>

// Explicit, finite end-of-tape mode. This is an unqualified monocular
// flat-floor experiment, not depth, localization or a stopping certificate.
// Guide's ordinary freshness contract is NOT changed by this class.
namespace va15_endpoint {
using V3=va15_tcp::V3;
using Pose=va15_tcp::Pose;
struct EndpointEvidence {
    bool candidate=false;
    double u=0.,v=0.,confidence=0.,clear_pixels=0.;
};
struct Input {
    Pose current;
    V3 tcp_world=V3::Constant(NAN); // measured-joint FK, never a nominal offset
    V3 target_seed=V3(-.775,0.,-.35);
    double clearance=.09,actual_speed_upper=NAN,dt=.005,speed_seed=.15,yaw_seed=0.;
    std::uint64_t now_ns=0,camera_ns=0;
    unsigned generation=0;
    std::uint64_t session=0;
    bool healthy=false,camera_healthy=false,operator_stop=false,sticks_centered=true;
    // A healthy no-slip pose is NOT an independently verified metric tool
    // position. Real deployment leaves this false until a separate terminal
    // visual/localization accuracy check is implemented and qualified.
    bool terminal_metric_verified=false;
};
enum class Stage {Observing,Confirmed,Tail,AwaitStand,Completed,Stopped,Fault};
struct Output {
    double x=0.,yaw=0.,remaining_arc=NAN,endpoint_error=NAN;
    V3 target=V3(-.775,0.,-.35);
    bool target_valid=false,stop=false,completed=false,awaiting_stand=false;
    Stage stage=Stage::Observing;
    const char* reason="waiting for a directly observed terminal cap";
};
class Follower {
    struct Projection {double s=0.,distance=INFINITY;std::size_t segment=0;double t=0.;V3 point=V3::Zero();};
    struct Node {V3 p=V3::Zero();std::uint64_t ns=0;};
    std::vector<Node> bank_,tail_;
    va15_visual::Packet previous_;
    EndpointEvidence previous_evidence_;
    unsigned generation_=0,count_=0;
    std::uint64_t session_=0,first_cap_ns_=0,last_cap_ns_=0,last_seq_=0,last_capture_=0;
    std::uint64_t tail_start_ns_=0,last_step_ns_=0,travel_sample_ns_=0;
    std::uint64_t stand_review_ns_=0,stand_good_ns_=0;
    V3 mean_cap_=V3::Zero(),cap_anchor_=V3::Zero(),endpoint_=V3::Zero(),target_=V3(-.775,0.,-.35);
    V3 last_base_=V3::Zero(),last_tcp_=V3::Zero(),first_tcp_=V3::Zero(),travel_sample_base_=V3::Zero();
    double clearance_=.09,progress_=0.,length_=0.,initial_remaining_=0.,travel_=0.,command_travel_=0.;
    double speed_=0.,yaw_=0.,remaining_=NAN,error_=NAN;
    Stage stage_=Stage::Observing;
    const char* reason_="waiting for a directly observed terminal cap";
    bool stopped_near_=false;
    static double Planar(const V3& p){return p.head<2>().norm();}
    static double Length(const std::vector<Node>& p) {
        double s=0.;for(std::size_t j=1;j<p.size();++j)s+=Planar(p[j].p-p[j-1].p);return s;
    }
    static Projection Nearest(const std::vector<Node>& p,const V3& v) {
        Projection best;double arc=0.;
        for(std::size_t j=1;j<p.size();++j) {
            const V3 d=p[j].p-p[j-1].p;const double d2=d.head<2>().squaredNorm();
            if(d2<1.e-12)continue;
            const double t=std::clamp((v-p[j-1].p).head<2>().dot(d.head<2>())/d2,0.,1.);
            const V3 at=p[j-1].p+t*d;const double distance=Planar(v-at);
            if(distance<best.distance)best={arc+t*std::sqrt(d2),distance,j-1,t,at};
            arc+=std::sqrt(d2);
        }
        return best;
    }
    static V3 At(const std::vector<Node>& p,double s,V3* tangent=nullptr) {
        if(p.size()<2)return V3::Constant(NAN);
        s=std::max(0.,s);
        for(std::size_t j=1;j<p.size();++j) {
            const V3 d=p[j].p-p[j-1].p;const double n=Planar(d);
            if(n<1.e-9)continue;
            if(s<=n||j+1==p.size()) {
                if(tangent)*tangent=d/n;
                return p[j-1].p+std::clamp(s/n,0.,1.)*d;
            }
            s-=n;
        }
        return p.back().p;
    }
    void ResetCap() {count_=0;first_cap_ns_=last_cap_ns_=0;mean_cap_.setZero();}
    bool Epoch(unsigned generation,std::uint64_t session) {
        if(session&&generation==generation_&&session==session_)return true;
        Fail("endpoint generation/session changed; explicit restart required");return false;
    }
    bool Fail(const char* why){stage_=Stage::Fault;reason_=why;speed_=yaw_=0.;return false;}
    bool Usable(const Input& in)const {
        return in.healthy&&in.camera_healthy&&va15_tcp::ValidPose(in.current)&&in.now_ns&&
            in.current.ns<=in.now_ns&&in.now_ns-in.current.ns<=50000000ULL&&
            in.camera_ns&&in.camera_ns<=in.now_ns&&in.now_ns-in.camera_ns<=va15_visual::MaximumAgeNs&&
            in.tcp_world.allFinite()&&std::isfinite(in.actual_speed_upper)&&in.actual_speed_upper>=0.&&
            in.actual_speed_upper<=.6&&std::isfinite(in.dt)&&in.dt>0.&&in.dt<=.02;
    }
    static bool Workspace(const V3& p) {
        return p.allFinite()&&p.x()>=-1.&&p.x()<=-.55&&std::fabs(p.y())<=.15&&p.z()>=-.55&&p.z()<=-.30;
    }
    bool Merge(const std::vector<Node>& curve) {
        if(bank_.empty()){bank_=curve;return true;}
        const auto tip=Nearest(curve,bank_.back().p);
        const double n=Length(bank_);
        const auto before=Nearest(curve,At(bank_,std::max(0.,n-.12)));
        // A join is allowed only inside a genuinely overlapping observed
        // interval. Never join disjoint frame endpoints across a dark gap.
        if(n>=.12&&tip.distance<=.035&&before.distance<=.035&&
            tip.s-before.s>=.08&&tip.s-before.s<=.18&&tip.s>=.10) {
            // If this view ends a few mm BEFORE the existing tip, Nearest
            // clamps to its last endpoint. That is valid overlap, NOT forward
            // extension. Appending the clamped last point would insert a
            // reverse segment and eventually poison the bank's arc ordering.
            if(Length(curve)-tip.s>.003) {
                for(std::size_t j=tip.segment+1;j<curve.size();++j)
                    if(Planar(curve[j].p-bank_.back().p)>.003)bank_.push_back(curve[j]);
            }
            // Preserve a bounded observed bank, enough to reach the rear TCP.
            while(bank_.size()>2&&(Length(bank_)>10.||bank_.size()>2048))bank_.erase(bank_.begin());
            return true;
        }
        const auto head=Nearest(curve,bank_.front().p);
        const auto after=Nearest(curve,At(bank_,std::min(.12,n)));
        if(n>=.12&&head.distance<=.035&&after.distance<=.035&&
            after.s-head.s>=.08&&after.s-head.s<=.18) {
            // Camera pitch can reveal an earlier portion of the SAME curve
            // while its far view is shorter than the old bank. Preserve that
            // real backward extension only with a directed 12cm overlap.
            if(head.s>.003) {
                std::vector<Node> joined;
                for(std::size_t j=0;j<=head.segment;++j)joined.push_back(curve[j]);
                if(!joined.empty()&&Planar(joined.back().p-bank_.front().p)<.003)joined.pop_back();
                joined.insert(joined.end(),bank_.begin(),bank_.end());bank_=std::move(joined);
            }
            return true;
        }
        // A shorter view entirely inside the same already observed bank is
        // harmless. It is not treated as newly observed forward extension.
        const auto near=Nearest(bank_,curve.front().p),middle=Nearest(bank_,At(curve,Length(curve)/2.));
        const auto far=Nearest(bank_,curve.back().p);
        return near.distance<=.035&&middle.distance<=.035&&far.distance<=.035&&
            near.s+.08<middle.s&&middle.s+.08<far.s;
    }
    Output Result(bool stop=false)const {
        Output o;o.x=speed_;o.yaw=yaw_;o.remaining_arc=remaining_;o.endpoint_error=error_;o.target=target_;
        o.target_valid=stage_==Stage::Tail||stage_==Stage::AwaitStand||stage_==Stage::Completed;
        o.stop=stop||stage_==Stage::AwaitStand||stage_==Stage::Completed||stage_==Stage::Stopped||stage_==Stage::Fault;
        o.awaiting_stand=stage_==Stage::AwaitStand;o.completed=stage_==Stage::Completed;
        o.stage=stage_;o.reason=reason_;return o;
    }
public:
    static constexpr std::uint64_t MaximumTailNs=45000000000ULL,MaximumBankAgeNs=60000000000ULL;
    static constexpr double MaximumTailArc=4.,EndpointTolerance=.065;
    static constexpr double ArrivalAlongTolerance=.015,ApproachSpeedFloor=.10;
    static constexpr std::uint64_t StandConfirmNs=1000000000ULL,StandReviewNs=3000000000ULL;
    void Reset(unsigned generation,std::uint64_t session) {
        *this=Follower();generation_=generation;session_=session;
        if(!session)Fail("endpoint session token must be nonzero");
    }
    Stage GetStage()const{return stage_;}
    const char* Reason()const{return reason_;}
    bool Confirmed()const{return stage_==Stage::Confirmed||stage_==Stage::Tail||stage_==Stage::AwaitStand||stage_==Stage::Completed;}
    bool Active()const{return stage_==Stage::Tail||stage_==Stage::AwaitStand;}
    bool AwaitingStand()const{return stage_==Stage::AwaitStand;}
    bool Completed()const{return stage_==Stage::Completed;}
    unsigned ConfirmingFrames()const{return count_;}
    const V3& Endpoint()const{return endpoint_;}
    const V3& Target()const{return target_;}
    double RemainingArc()const{return remaining_;}
    std::size_t BankPoints()const{return bank_.size();}
    void Abort(const char* why="operator stop; explicit restart required") {
        stage_=Stage::Stopped;reason_=why;speed_=yaw_=0.;
    }
    bool Observe(const va15_visual::Packet& p,const Pose& capture,std::uint64_t now,
                 unsigned generation,std::uint64_t session,const EndpointEvidence& e) {
        if(!Epoch(generation,session)||stage_==Stage::Stopped||stage_==Stage::Fault||stage_==Stage::Completed)return false;
        if(!p.session_matched||!p.sequence||!p.capture_ns||p.capture_ns>p.publish_ns||p.publish_ns>now||
            now<p.capture_ns||now-p.capture_ns>va15_visual::MaximumAgeNs) {
            if(!Confirmed())ResetCap();
            reason_="endpoint observation is stale or session-invalid";return false;
        }
        if(p.sequence<last_seq_||(p.sequence>last_seq_&&p.capture_ns<=last_capture_))
            return Fail("endpoint camera sequence/clock reversed; explicit restart required");
        if(p.sequence==last_seq_) {
            const bool same=p.capture_ns==previous_.capture_ns&&p.publish_ns==previous_.publish_ns&&
                p.valid==previous_.valid&&p.image_received==previous_.image_received&&
                p.path==previous_.path&&e.candidate==previous_evidence_.candidate&&
                e.u==previous_evidence_.u&&e.v==previous_evidence_.v&&e.confidence==previous_evidence_.confidence&&
                e.clear_pixels==previous_evidence_.clear_pixels;
            return same?true:Fail("endpoint duplicate sequence changed contents");
        }
        last_seq_=p.sequence;last_capture_=p.capture_ns;previous_=p;previous_evidence_=e;
        if(!p.image_received||!va15_visual::Valid(p,now)||capture.ns!=p.capture_ns||!va15_tcp::ValidPose(capture)||
            p.path.size()!=p.path_count) {
            if(!Confirmed())ResetCap();
            return false;
        }
        std::vector<Node> curve;
        for(const auto& uv:p.path) {
            V3 world;if(!va15_tcp::Project(capture,uv,world))break;
            if(!curve.empty()&&Planar(world-curve.back().p)>.20) {
                if(Active())return Fail("observed endpoint path has a physical coverage gap");
                ResetCap();return false;
            }
            curve.push_back({world,p.capture_ns});
        }
        if(curve.size()<8||Length(curve)<.15||Length(curve)>8.) {if(!Confirmed())ResetCap();return false;}
        V3 cap;
        const bool cap_valid=e.candidate&&std::isfinite(e.confidence)&&e.confidence>=.97&&e.confidence<=1.&&
            std::isfinite(e.clear_pixels)&&e.clear_pixels>=24.&&e.clear_pixels<=300.&&
            e.u>=24.&&e.u<=1896.&&e.v>=618.&&e.v<=1034.4&&
            curve.size()==p.path.size()&&std::hypot(e.u-p.path.back()[0],e.v-p.path.back()[1])<=3.&&
            va15_tcp::Project(capture,{e.u,e.v},cap);
        if(Confirmed()&&cap_valid&&Planar(cap-endpoint_)>.08)
            return Fail("conflicting physical endpoint; do not switch to another line");
        if(Confirmed()) {
            const auto locked=Nearest(curve,endpoint_);
            if(locked.distance<=.05&&Length(curve)-locked.s>.10)
                return Fail("observed connected path continues beyond confirmed terminal");
        }
        if(!Active()&&!Merge(curve)) {
            if(Confirmed())return Fail("endpoint curve is disjoint from previously observed path");
            ResetCap();reason_="endpoint confirmation refused: no observed curve overlap";return false;
        }
        if(!cap_valid) {if(!Confirmed())ResetCap();return true;}
        if(Confirmed()) {last_cap_ns_=p.capture_ns;return true;}
        if(last_cap_ns_&&p.capture_ns-last_cap_ns_>va15_visual::MaximumAgeNs)ResetCap();
        if(count_&&(Planar(cap-mean_cap_)>.05||Planar(cap-cap_anchor_)>.05))ResetCap();
        if(!count_){first_cap_ns_=p.capture_ns;cap_anchor_=cap;}
        mean_cap_=(double(count_)*mean_cap_+cap)/double(count_+1);++count_;last_cap_ns_=p.capture_ns;
        if(count_>=6&&last_cap_ns_-first_cap_ns_>=500000000ULL) {
            endpoint_=mean_cap_;stage_=Stage::Confirmed;
            reason_="endpoint confirmed by distinct fresh cap observations; finite tail not yet started";
        } else reason_="confirming terminal cap across independent camera frames";
        return true;
    }
    bool BeginTail(const Input& in) {
        if(!Epoch(in.generation,in.session)||stage_!=Stage::Confirmed)return false;
        if(in.operator_stop||!in.sticks_centered){Abort();return false;}
        if(!Usable(in)||in.now_ns<last_cap_ns_||in.now_ns-last_cap_ns_>va15_visual::MaximumAgeNs)
            return Fail("fresh camera/ego state required to enter finite endpoint tail");
        if(bank_.size()<2||!Workspace(in.target_seed)||!std::isfinite(in.clearance)||in.clearance<0.||in.clearance>.25)
            return Fail("finite tail bank/target seed is invalid");
        const auto end=Nearest(bank_,endpoint_),tcp=Nearest(bank_,in.tcp_world);
        if(end.distance>.05)return Fail("confirmed endpoint is not on the observed terminal path");
        V3 first_tangent;At(bank_,0.,&first_tangent);
        const double before_start=(in.tcp_world-bank_.front().p).head<2>().dot(first_tangent.head<2>());
        if((tcp.s<.02&&before_start<-.02)||end.s-tcp.s>MaximumTailArc) {
            reason_="endpoint confirmed; waiting for actual rear TCP to enter finite observed coverage";return false;
        }
        V3 end_tangent;At(bank_,end.s,&end_tangent);
        if(tcp.s>end.s+.02||(in.tcp_world-end.point).head<2>().dot(end_tangent.head<2>())>.12)
            return Fail("actual rear TCP has already passed the finite observed terminal");
        if(tcp.distance>.08) {
            // The ordinary fresh-image Guide may still be aligning the tool.
            // Do not turn an unready cached-tail transition into a new fault,
            // and do not grant blind permission or widen its 8cm corridor.
            reason_="endpoint confirmed; waiting for actual rear TCP alignment inside observed corridor";return false;
        }
        // Truncate strictly at a point ON an observed segment. No extension
        // is made from a nominal rear offset or an unseen endpoint estimate.
        tail_.clear();
        const double start_arc=std::max(0.,tcp.s-.15);
        const V3 start_point=At(bank_,start_arc);
        const auto begin=Nearest(bank_,start_point);
        tail_.push_back({start_point,bank_[begin.segment+1].ns});
        for(std::size_t j=begin.segment+1;j<=end.segment;++j)
            if(Planar(bank_[j].p-tail_.back().p)>.001)tail_.push_back(bank_[j]);
        tail_.push_back({end.point,bank_[end.segment+1].ns});
        if(tail_.size()<2||in.now_ns<tail_.front().ns||in.now_ns-tail_.front().ns>MaximumBankAgeNs)
            return Fail("finite endpoint cache is too old");
        endpoint_=end.point;length_=Length(tail_);progress_=std::max(0.,tcp.s-start_arc);remaining_=length_-progress_;
        initial_remaining_=remaining_;target_=in.target_seed;clearance_=in.clearance;
        speed_=std::clamp(in.speed_seed,0.,va15_visual::Speed);
        yaw_=std::clamp(in.yaw_seed,-va15_visual::YawLimit,va15_visual::YawLimit);
        tail_start_ns_=last_step_ns_=travel_sample_ns_=in.now_ns;
        last_base_=travel_sample_base_=in.current.p;last_tcp_=first_tcp_=in.tcp_world;
        travel_=command_travel_=0.;stage_=Stage::Tail;reason_="bounded observed terminal tail; actual rear TCP progress";
        return true;
    }
    Output Step(const Input& in) {
        if(!Epoch(in.generation,in.session))return Result(true);
        if(in.operator_stop||!in.sticks_centered){Abort();return Result(true);}
        if(stage_!=Stage::Tail&&stage_!=Stage::AwaitStand)return Result();
        if(!Usable(in)){Fail("terminal tail lost fresh healthy camera/ego pose");return Result(true);}
        if(in.now_ns<last_step_ns_||in.now_ns-tail_start_ns_>MaximumTailNs) {
            Fail("finite endpoint tail time/cache budget exceeded");return Result(true);
        }
        if(in.now_ns!=last_step_ns_) {
            const double b=Planar(in.current.p-last_base_),t=Planar(in.tcp_world-last_tcp_);
            if(b>.08||t>.08){Fail("endpoint ego/TCP discontinuity");return Result(true);}
            // Budget base travel at 10 Hz, rather than summing IMU/no-slip
            // numerical jitter or the TCP's cyclic gait vibration at 200 Hz.
            // Commanded travel and net FK displacement have independent caps.
            command_travel_+=speed_*double(in.now_ns-last_step_ns_)*1.e-9;
            if(in.now_ns-travel_sample_ns_>=100000000ULL) {
                travel_+=Planar(in.current.p-travel_sample_base_);
                travel_sample_base_=in.current.p;travel_sample_ns_=in.now_ns;
            }
            last_base_=in.current.p;last_tcp_=in.tcp_world;last_step_ns_=in.now_ns;
        }
        if(travel_>initial_remaining_+.50||command_travel_>initial_remaining_+.75||
            Planar(in.tcp_world-first_tcp_)>initial_remaining_+.25) {
            Fail("finite endpoint tail travel budget exceeded");return Result(true);
        }
        const auto tcp=Nearest(tail_,in.tcp_world);
        V3 tangent;At(tail_,length_,&tangent);
        error_=Planar(in.tcp_world-endpoint_);
        const double overshoot=(in.tcp_world-endpoint_).head<2>().dot(tangent.head<2>());
        if(tcp.distance>.10||tcp.s<progress_-.12||overshoot>.12) {
            Fail("actual TCP left finite terminal corridor or overshot end");return Result(true);
        }
        progress_=std::max(progress_,tcp.s);
        // Reaching a later part of the finite bank consumes the earlier
        // corridor; its age must not keep authorizing OR invalidating motion.
        // Keep15cm of already OBSERVED back-step margin. No point is made
        // beyond a captured segment, and remaining-node stamps are untouched.
        if(progress_>.18) {
            const double cut=progress_-.15;const V3 first=At(tail_,cut);
            const auto begin=Nearest(tail_,first);
            std::vector<Node> rest{{first,tail_[begin.segment+1].ns}};
            for(std::size_t j=begin.segment+1;j<tail_.size();++j)
                if(Planar(tail_[j].p-rest.back().p)>.001)rest.push_back(tail_[j]);
            tail_=std::move(rest);progress_-=cut;length_=Length(tail_);
        }
        if(tail_.size()<2||in.now_ns<tail_.front().ns||in.now_ns-tail_.front().ns>MaximumBankAgeNs) {
            Fail("finite endpoint tail currently needed observed segment is too old");return Result(true);
        }
        remaining_=std::max(0.,length_-progress_);
        stopped_near_=error_<=EndpointTolerance&&std::fabs(in.tcp_world.z()-(endpoint_.z()+.09))<=.05&&
            in.actual_speed_upper<=.04;
        if(stage_==Stage::AwaitStand) {
            speed_=yaw_=0.;
            if(error_>.12){Fail("actual TCP drifted away while settling at terminal");return Result(true);}
            return Result(true);
        }
        V3 world=At(tail_,std::min(length_,progress_+.06),&tangent);
        clearance_+=std::clamp(.09-clearance_,-.01*in.dt,.01*in.dt);world.z()+=clearance_;
        const V3 desired=in.current.q.conjugate()*(world-in.current.p);
        if(!Workspace(desired)){Fail("terminal TCP target outside trained workspace");return Result(true);}
        const V3 rate(.04,.04,.02);
        for(int j=0;j<3;++j)
            target_[j]+=std::clamp(in.dt/(.6+in.dt)*(desired[j]-target_[j]),-rate[j]*in.dt,rate[j]*in.dt);
        // Monotonic progress alone can latch an earlier forward gait peak.
        // Use the CURRENT measured along-tail TCP position as well; reserve
        // the wider6.5cm limit for AFTER the supported stand has settled.
        if(remaining_<=ArrivalAlongTolerance&&std::fabs(overshoot)<=ArrivalAlongTolerance&&
           error_<=EndpointTolerance) {
            stage_=Stage::AwaitStand;speed_=yaw_=0.;reason_="actual TCP at terminal; original controlled stand required";
            return Result(true);
        }
        // Slow down BEFORE the actual rear TCP arrives. Body position alone
        // neither consumes the tail nor qualifies task completion.
        // VA15 does not reliably move at tiny forward requests: allowing
        // .02-.07m/s can stall before the end across tested frictions. This ENDPOINT-ONLY approach
        // floor is still <= the existing .15cap and .03m/s^2 slew; it is not
        // a global joystick/gait change. Actual FK arrival above, not a timer
        // or nominal body offset, requests the unchanged supported stop.
        const double wanted=std::min(va15_visual::Speed,
            std::max(ApproachSpeedFloor,va15_visual::Speed*std::max(0.,remaining_-.035)/.55));
        speed_+=std::clamp(wanted-speed_,-.03*in.dt,.03*in.dt);
        const V3 body_tangent=in.current.q.conjugate()*tangent;
        const V3 error=in.tcp_world-tcp.point;
        const double lateral=-tangent.y()*error.x()+tangent.x()*error.y();
        const double bearing=std::atan2(body_tangent.y(),body_tangent.x())-std::atan2(lateral,.6);
        if(!std::isfinite(bearing)||std::fabs(bearing)>.45) {
            Fail("finite tail outside bounded steering envelope");return Result(true);
        }
        const double wanted_yaw=std::clamp(va15_visual::BearingGain*bearing,-va15_visual::YawLimit,va15_visual::YawLimit);
        const double filtered=yaw_+in.dt/(.4+in.dt)*(wanted_yaw-yaw_);
        yaw_+=std::clamp(filtered-yaw_,-va15_visual::YawSlew*in.dt,va15_visual::YawSlew*in.dt);
        reason_="bounded observed terminal tail; actual rear TCP progress";return Result();
    }
    bool NotifyStableStand(const Input& in,bool original_stop_ready) {
        Step(in);
        if(stage_!=Stage::AwaitStand)return false;
        if(!original_stop_ready) {stand_review_ns_=stand_good_ns_=0;return false;}
        if(!stand_review_ns_)stand_review_ns_=in.now_ns;
        if(stopped_near_) {
            if(!stand_good_ns_)stand_good_ns_=in.now_ns;
            if(in.now_ns-stand_good_ns_>=StandConfirmNs) {
                if(!in.terminal_metric_verified) {
                    stage_=Stage::Stopped;speed_=yaw_=0.;
                    reason_="LOCAL ESTIMATED END / PHYSICAL TERMINAL UNVERIFIED: safe stand, NOT task completion";
                    return false;
                }
                stage_=Stage::Completed;speed_=yaw_=0.;
                reason_="actual TCP at terminal for1s after original supported stable stand";return true;
            }
        } else stand_good_ns_=0;
        if(in.now_ns-stand_review_ns_>=StandReviewNs) {
            stage_=Stage::Stopped;speed_=yaw_=0.;
            reason_="SAFE STAND / ENDPOINT NOT COMPLETE: actual TCP position/height/speed outside tolerance";
            return false;
        }
        reason_="supported stand reached; verifying actual terminal TCP continuously before completion";
        return false;
    }
};
}
