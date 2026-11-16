#pragma once
#include "visual_tcp.hpp"
#include "va15_independent_state.hpp"

namespace va15_foot_seam {
using V3=Eigen::Vector3d;
inline constexpr std::array<int,12> PolicyDDS{{1,5,10,0,4,9,3,7,12,2,6,11}};

// Geometry only; this must never assert or override contact/load evidence.
inline bool Centers(const va15_independent::Model& model,const std::vector<float>& q,
                    std::array<V3,4>& centers) {
    if(q.size()!=18)return false;
    for(int leg=0;leg<4;++leg) {
        Eigen::Matrix3d r=Eigen::Matrix3d::Identity();V3 p=V3::Zero();
        for(int j=0;j<3;++j) {
            const auto& joint=model.legs[leg].joints[j];const double value=q[PolicyDDS[3*leg+j]];
            if(!std::isfinite(value)||value<joint.lower-.01||value>joint.upper+.01)return false;
            p+=r*joint.origin;r*=va15_independent::Rpy(joint.rpy);
            r*=Eigen::AngleAxisd(value,joint.axis).toRotationMatrix();
        }
        centers[leg]=p+r*model.legs[leg].tip;
    }
    return true;
}
struct Clearance {
    std::array<double,4> edge_m{{NAN,NAN,NAN,NAN}};
    unsigned covered=0;
    double minimum=NAN;
};
inline Clearance Check(const va15_independent::Model& model,const std::vector<float>& q,
                       const va15_tcp::Pose& pose,const va15_tcp::Tracker& tracker) {
    Clearance out;std::array<V3,4> center;
    if(!va15_tcp::ValidPose(pose)||!Centers(model,q,center))return out;
    for(int leg=0;leg<4;++leg) {
        const V3 foot=pose.p+pose.q*center[leg];
        for(auto it=tracker.curves.rbegin();it!=tracker.curves.rend();++it) {
            if(pose.ns<it->ns||pose.ns-it->ns>va15_tcp::Tracker::RetentionNs||it->half_width<=0.)continue;
            bool covered=false;double best=INFINITY;
            for(std::size_t i=1;i<it->points.size();++i) {
                const V3 a=pose.q.conjugate()*(it->points[i-1]-pose.p);
                const V3 b=pose.q.conjugate()*(it->points[i]-pose.p);
                // Do not label initial unobserved rear feet as clear by
                // measuring distance to an unrelated front endpoint.
                if(center[leg].x()<std::min(a.x(),b.x())||center[leg].x()>std::max(a.x(),b.x()))continue;
                const Eigen::Vector2d delta=(it->points[i]-it->points[i-1]).head<2>();
                if(delta.squaredNorm()<1.e-10)continue;
                const double f=std::clamp((foot-it->points[i-1]).head<2>().dot(delta)/delta.squaredNorm(),0.,1.);
                best=std::min(best,(foot.head<2>()-it->points[i-1].head<2>()-f*delta).norm()-
                    model.legs[leg].radius-it->half_width);covered=true;
            }
            if(covered) {out.edge_m[leg]=best;++out.covered;break;}
        }
        if(std::isfinite(out.edge_m[leg]))out.minimum=std::isfinite(out.minimum)?std::min(out.minimum,out.edge_m[leg]):out.edge_m[leg];
    }
    return out;
}
}
