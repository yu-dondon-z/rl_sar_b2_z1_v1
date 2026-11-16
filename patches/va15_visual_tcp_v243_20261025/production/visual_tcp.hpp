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
    struct Curve {std::uint64_t ns=0;std::vector<V3> points;V3 near=V3::Zero(),look=V3::Zero();};
    std::deque<Curve> curves;
    V3 target=V3(-.775,0.,-.35);
    double clearance=.16,blend_time=0.;
    bool active=false,ever_tracked=false;
    unsigned generation=0;
    std::uint64_t sequence=0;
    const char* reason="waiting for observed rear path";
    void Reset(){*this=Tracker();}
    void Begin(const Pose& p,const V3& seed=V3(-.775,0.,-.35)) {
        target=seed;clearance=(p.p+p.q*seed).z();
        active=false;ever_tracked=false;blend_time=0.;reason="waiting for observed rear path";
    }
    bool Observe(const va15_visual::Packet& packet,const Pose& capture) {
        if(packet.sequence==sequence)return true;
        if(!packet.valid||packet.sequence<sequence||!ValidPose(capture)||capture.ns!=packet.capture_ns||
           packet.path.size()<25||packet.path.size()>128)return false;
        Curve curve;curve.ns=capture.ns;
        // Do not connect separate frames with invented line segments.
        for(const auto& uv:packet.path) {
            V3 p;if(!Project(capture,uv,p))break;
            if(!curve.points.empty()&&(p-curve.points.back()).norm()>.20)break;
            curve.points.push_back(p);
        }
        if(curve.points.size()<8)return false;
        if(!Project(capture,{packet.u,packet.v},curve.near)||
           !Project(capture,{packet.look_u,packet.look_v},curve.look))return false;
        curves.push_back(std::move(curve));sequence=packet.sequence;
        while(curves.size()>192)curves.pop_front();
        return true;
    }
    bool Guidance(const Pose& current,double& bearing)const {
        if(!ValidPose(current)||curves.empty()||current.ns<curves.back().ns||
           current.ns-curves.back().ns>va15_visual::MaximumAgeNs)return false;
        const auto& curve=curves.back();
        const V3 near=current.q.conjugate()*(curve.near-current.p);
        const V3 look=current.q.conjugate()*(curve.look-current.p);
        if(near.x()<.35||look.x()<.35)return false;
        // Guide the nominal rear TCP reference rather than B2's nose.
        // Nose-only pursuit plus lateral course error can leave the rear tool
        // outside +/-15 cm even when the body is near the tape. This geometry
        // balances body centering with the actual rear-tool workspace; it
        // does not extrapolate any unobserved TCP path.
        bearing=.6*std::atan2(near.y(),near.x()+.775)+.4*std::atan2(look.y(),look.x()+.775);
        return std::isfinite(bearing)&&std::fabs(bearing)<=.45;
    }
    bool Step(const Pose& current,double dt,bool moving) {
        active=false;
        if(!ValidPose(current)||!std::isfinite(dt)||dt<=0.||dt>.02) {reason="invalid local pose/time; target frozen";return false;}
        while(!curves.empty()&&(current.ns<curves.front().ns||current.ns-curves.front().ns>45000000000ULL))curves.pop_front();
        if(!moving){reason="stopped: holding last TCP command";return true;}
        V3 desired;bool found=false;double score=std::numeric_limits<double>::infinity();
        for(const auto& curve:curves)for(std::size_t i=1;i<curve.points.size();++i) {
            const V3 a=current.q.conjugate()*(curve.points[i-1]-current.p);
            const V3 b=current.q.conjugate()*(curve.points[i]-current.p);
            if(std::fabs(b.x()-a.x())<1.e-6)continue;
            const double fraction=(-.775-a.x())/(b.x()-a.x());
            if(fraction<0.||fraction>1.)continue;
            V3 world=(1.-fraction)*curve.points[i-1]+fraction*curve.points[i];world.z()+=clearance;
            const V3 candidate=current.q.conjugate()*(world-current.p);
            // Reject rather than silently clamp an unreachable crack target.
            if(candidate.x()< -1.||candidate.x()>-.55||std::fabs(candidate.y())>.15||
               candidate.z()<-.55||candidate.z()>-.30)continue;
            const double cost=std::fabs(candidate.y())+.0001*double(current.ns-curve.ns)*1.e-9;
            if(cost<score){score=cost;desired=candidate;found=true;}
        }
        if(!found){reason=ever_tracked?"rear path outside trained workspace; target frozen":"waiting for front-observed path to reach rear TCP";return true;}
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
