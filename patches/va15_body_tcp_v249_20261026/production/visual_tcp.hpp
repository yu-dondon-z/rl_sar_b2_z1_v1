#pragma once
#include "near_visual_guide.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <deque>
#include <limits>

// Monocular FLAT-floor experiment. Local inertial/no-slip odom, NOT depth,
// global localization, a contact sensor, or a hardware-qualified estimator.
namespace va15_tcp {
using V3=Eigen::Vector3d;
struct Pose {
    std::uint64_t ns=0;
    V3 p=V3::Zero();
    Eigen::Quaterniond q=Eigen::Quaterniond::Identity();
    bool valid=false;
};
inline bool ValidPose(const Pose& p) {
    return p.valid&&p.ns&&p.p.allFinite()&&p.q.coeffs().allFinite()&&
        std::fabs(p.q.squaredNorm()-1.)<.01&&p.p.z()>.3&&p.p.z()<.8;
}
class History {
public:
    std::deque<Pose> poses;
    unsigned generation=0;
    void Clear(){poses.clear();++generation;}
    bool Add(const Pose& p) {
        if(!ValidPose(p)){Clear();return false;}
        if(!poses.empty()) {
            const auto& last=poses.back();
            if(p.ns<=last.ns||p.ns-last.ns>100000000ULL||(p.p-last.p).norm()>.08||
               last.q.angularDistance(p.q)>.12)Clear();
        }
        poses.push_back(p);
        while(poses.size()>2&&p.ns-poses.front().ns>2000000000ULL)poses.pop_front();
        return true;
    }
    bool At(std::uint64_t stamp,Pose& out)const {
        if(poses.empty()||stamp<poses.front().ns||stamp>poses.back().ns)return false;
        for(std::size_t i=1;i<poses.size();++i)if(poses[i].ns>=stamp) {
            const auto& a=poses[i-1];const auto& b=poses[i];
            if(b.ns-a.ns>80000000ULL)return false;
            const double t=double(stamp-a.ns)/double(b.ns-a.ns);
            out={stamp,(1.-t)*a.p+t*b.p,a.q.slerp(t,b.q).normalized(),true};return true;
        }
        return false;
    }
};
inline bool Ray(double u,double v,V3& ray) {
    if(!std::isfinite(u)||!std::isfinite(v)||u<0.||u>=1920.||v<594.||v>1058.4)return false;
    const double x0=(u-934.705)/756.700,y0=(v-510.684)/752.123;
    double x=x0,y=y0;
    for(int n=0;n<8;++n) {
        const double r=x*x+y*y,f=1.+.049696*r-.03070179*r*r-.04031327*r*r*r;
        if(!std::isfinite(f)||f<.2)return false;
        const double dx=2.*(-.00359919)*x*y+(-.00590063)*(r+2.*x*x);
        const double dy=(-.00359919)*(r+2.*y*y)+2.*(-.00590063)*x*y;
        x=(x0-dx)/f;y=(y0-dy)/f;
    }
    ray=V3(.00651959*x-.02434965*y+.99968224,
           -.99983166*x-.01730499*y+.00609906,
           .01715098*x-.99955372*y-.02445837);
    return ray.allFinite();
}
inline bool Project(const Pose& capture,const std::array<double,2>& uv,V3& point) {
    if(!ValidPose(capture))return false;
    V3 ray;if(!Ray(uv[0],uv[1],ray))return false;
    const V3 origin=capture.p+capture.q*V3(.44643878,-.00569431,-.00031663);
    const V3 world_ray=capture.q*ray;
    if(world_ray.z()>=-.08)return false;
    const double scale=-origin.z()/world_ray.z();
    if(!std::isfinite(scale)||scale<=0.||scale>5.)return false;
    point=origin+scale*world_ray;
    const V3 base=capture.q.conjugate()*(point-capture.p);
    return point.allFinite()&&base.x()>=.45&&base.x()<=4.&&std::fabs(base.y())<1.5&&std::fabs(point.z())<1.e-6;
}
class Tracker {
public:
    struct Curve {std::uint64_t ns=0;std::vector<V3> points;V3 near=V3::Zero(),look=V3::Zero();Pose capture;};
    std::deque<Curve> curves;
    Curve live;bool live_valid=false;
    static constexpr std::uint64_t RetentionNs=90000000000ULL;
    static constexpr std::size_t MaximumCurves=256;
    V3 target=V3(-.775,0.,-.35);
    double clearance=.16,blend_time=0.;
    bool active=false,ever_tracked=false;
    unsigned generation=0;
    std::uint64_t sequence=0;
    unsigned rear_crossings=0,workspace_rejections=0;
    double nearest_rear_gap=0.,cache_age=0.;
    V3 desired=V3(-.775,0.,-.35);
    const char* reason="waiting for observed rear path";
    void Reset(){*this=Tracker();}
    void ClearObservations(){curves.clear();live={};live_valid=false;sequence=0;active=false;}
    void Begin(const Pose& p,const V3& seed=V3(-.775,0.,-.35)) {
        target=desired=seed;clearance=(p.p+p.q*seed).z();
        active=false;ever_tracked=false;blend_time=0.;reason="waiting for observed rear path";
    }
    bool Observe(const va15_visual::Packet& packet,const Pose& capture) {
        if(packet.sequence==sequence)return true;
        if(!packet.valid||packet.sequence<sequence||!ValidPose(capture)||capture.ns!=packet.capture_ns||
           packet.path.size()<25||packet.path.size()>128)return false;
        Curve curve;curve.ns=capture.ns;curve.capture=capture;
        // Do not connect separate frames with invented line segments.
        for(const auto& uv:packet.path) {
            V3 p;if(!Project(capture,uv,p))break;
            if(!curve.points.empty()&&(p-curve.points.back()).norm()>.20)break;
            curve.points.push_back(p);
        }
        if(curve.points.size()<8)return false;
        if(!Project(capture,{packet.u,packet.v},curve.near)||
           !Project(capture,{packet.look_u,packet.look_v},curve.look))return false;
        live=curve;live_valid=true;sequence=packet.sequence;
        // A frame-count-only cache expired BEFORE the slow real robot could
        // carry front-observed tape to its rear TCP. Retain measured curves
        // by travelled distance/yaw, not camera FPS or gait pitch oscillation.
        bool store=curves.empty();
        if(!store) {
            const auto& last=curves.back().capture;
            const V3 delta=capture.p-last.p;
            const V3 a=last.q*V3::UnitX(),b=capture.q*V3::UnitX();
            const double heading=std::atan2(a.x()*b.y()-a.y()*b.x(),a.x()*b.x()+a.y()*b.y());
            store=std::hypot(delta.x(),delta.y())>=.035||std::fabs(heading)>=.02||capture.ns-last.ns>=1000000000ULL;
        }
        if(store)curves.push_back(std::move(curve));
        while(curves.size()>MaximumCurves||(!curves.empty()&&capture.ns>=curves.front().ns&&capture.ns-curves.front().ns>RetentionNs))curves.pop_front();
        return true;
    }
    bool Guidance(const Pose& current,double& bearing)const {
        if(!ValidPose(current)||!live_valid||current.ns<live.ns||
           current.ns-live.ns>va15_visual::MaximumAgeNs)return false;
        const auto& curve=live;
        const V3 near=current.q.conjugate()*(curve.near-current.p);
        const V3 look=current.q.conjugate()*(curve.look-current.p);
        if(near.x()<.35||look.x()<.35)return false;
        // BODY guidance must not be weakened by adding rear-tool distance.
        // Rear TCP positioning remains a separate observed-history lookup.
        bearing=.6*std::atan2(near.y(),near.x())+.4*std::atan2(look.y(),look.x());
        return std::isfinite(bearing)&&std::fabs(bearing)<=.45;
    }
    bool BodyGuidance(const Pose& current,double& lateral,double& heading,bool& historical)const {
        double unused;if(!Guidance(current,unused))return false;
        const V3 near=current.q.conjugate()*(live.near-current.p);
        const V3 look=current.q.conjugate()*(live.look-current.p);
        const V3 direction=look-near;
        if(direction.x()<.10)return false;
        heading=std::atan2(direction.y(),direction.x());
        // Before any real observation reaches beneath the body, use a local
        // tangent-based cross-track ESTIMATE. It is NOT added to the crack
        // bank and is NEVER used as an unseen rear TCP target.
        lateral=near.y()-near.x()*direction.y()/direction.x();historical=false;
        const double preview_lateral=lateral,preview_heading=heading;
        double best=std::numeric_limits<double>::infinity();
        for(const auto& curve:curves) {
            if(current.ns<curve.ns||current.ns-curve.ns>RetentionNs)continue;
            for(std::size_t i=1;i<curve.points.size();++i) {
                const V3 a=current.q.conjugate()*(curve.points[i-1]-current.p);
                const V3 b=current.q.conjugate()*(curve.points[i]-current.p);
                if(b.x()-a.x()<1.e-6)continue;
                const double f=-a.x()/(b.x()-a.x());
                if(f<0.||f>1.)continue;
                const double y=(1.-f)*a.y()+f*b.y();
                const double score=std::fabs(y)+.0001*double(current.ns-curve.ns)*1.e-9;
                const double tangent=std::atan2(b.y()-a.y(),b.x()-a.x());
                // Old locally integrated poses can drift. Do not let a
                // convenient cached crossing erase a CURRENT visual offset.
                if(std::fabs(y-preview_lateral)<=.04&&std::fabs(tangent-preview_heading)<=.08&&score<best) {
                    best=score;lateral=.8*preview_lateral+.2*y;
                    heading=.8*preview_heading+.2*tangent;historical=true;
                }
            }
        }
        return std::isfinite(lateral)&&std::isfinite(heading)&&std::fabs(heading)<=.65&&std::fabs(lateral)<=1.5;
    }
    bool Step(const Pose& current,double dt,bool moving) {
        active=false;rear_crossings=workspace_rejections=0;nearest_rear_gap=100.;cache_age=0.;
        if(!ValidPose(current)||!std::isfinite(dt)||dt<=0.||dt>.02) {reason="invalid local pose/time; target frozen";return false;}
        while(!curves.empty()&&(current.ns<curves.front().ns||current.ns-curves.front().ns>RetentionNs))curves.pop_front();
        if(!curves.empty())cache_age=double(current.ns-curves.front().ns)*1.e-9;
        if(!moving){reason="stopped: holding last TCP command";return true;}
        bool found=false;double score=std::numeric_limits<double>::infinity();
        for(const auto& curve:curves)for(std::size_t i=1;i<curve.points.size();++i) {
            const V3 a=current.q.conjugate()*(curve.points[i-1]-current.p);
            const V3 b=current.q.conjugate()*(curve.points[i]-current.p);
            nearest_rear_gap=std::min(nearest_rear_gap,std::min(std::fabs(a.x()+.775),std::fabs(b.x()+.775)));
            if(std::fabs(b.x()-a.x())<1.e-6)continue;
            // The training contract permits x in [-1,-.55], not only one
            // rigid rear station. Allocate the observed segment within that
            // SAME workspace rather than inventing/clamping an unseen path.
            // Intersect the segment parameter with all original XYZ bounds.
            double lo=0.,hi=1.;
            const V3 delta=b-a;
            const V3 lift=current.q.conjugate()*V3(0.,0.,clearance);
            const V3 lower(-1.,-.15,-.55),upper(-.55,.15,-.30);
            for(int j=0;j<3;++j) {
                if(std::fabs(delta[j])<1.e-9) {
                    if(a[j]+lift[j]<lower[j]||a[j]+lift[j]>upper[j])lo=2.;
                } else {
                    const double f0=(lower[j]-a[j]-lift[j])/delta[j];
                    const double f1=(upper[j]-a[j]-lift[j])/delta[j];
                    lo=std::max(lo,std::min(f0,f1));hi=std::min(hi,std::max(f0,f1));
                }
            }
            const double nominal=(-.775-a.x())/delta.x();
            if(nominal>=0.&&nominal<=1.)++rear_crossings;
            if(lo>hi) {if(nominal>=0.&&nominal<=1.)++workspace_rejections;continue;}
            const double fraction=std::clamp(nominal,lo,hi);
            V3 world=(1.-fraction)*curve.points[i-1]+fraction*curve.points[i];world.z()+=clearance;
            const V3 candidate=current.q.conjugate()*(world-current.p);
            // Reject rather than silently clamp an unreachable crack target.
            if(candidate.x()< -1.||candidate.x()>-.55||std::fabs(candidate.y())>.15||
               candidate.z()<-.55||candidate.z()>-.30){++workspace_rejections;continue;}
            const double cost=2.*std::fabs(candidate.x()+.775)+std::fabs(candidate.y())+.0001*double(current.ns-curve.ns)*1.e-9;
            if(cost<score){score=cost;desired=candidate;found=true;}
        }
        if(!found){reason=rear_crossings?"observed rear path exists but outside trained workspace; target held":"waiting for actual observed history to reach rear TCP; target held";return true;}
        active=ever_tracked=true;blend_time+=dt;
        // Checkpoint env.yaml: tool_height_m=.09; limit_ranges.pos_z=[-.55,-.30].
        // The archived deployment's -.38 cutoff excluded the actual training
        // trajectory at a ~.52 m base height. Restore the TRAINING contract,
        // not an arbitrary larger workspace. Lower the target plane only
        // after real rear history exists, at <=1 cm/s; never jump at R1+Y.
        clearance+=std::clamp(.09-clearance,-.01*dt,.01*dt);
        const double gain=std::min(1.,blend_time/4.);
        const V3 rate(.04,.04,.02);
        for(int j=0;j<3;++j) {
            const double step=dt/(.6+dt)*(desired[j]-target[j])*gain;
            target[j]+=std::clamp(step,-rate[j]*dt,rate[j]*dt);
        }
        reason="tracking observed cached flat-floor path; NOT depth";return true;
    }
};
}
