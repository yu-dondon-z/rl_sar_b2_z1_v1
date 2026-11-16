#pragma once
#include "visual_tcp.hpp"

// Local, finite-segment visual odometry calibration. A repeatedly observed
// STATIC cap should not move in the local map. Its measured drift relative
// to independently estimated base displacement identifies along-path scale:
//   cap_map = cap_true + (1-scale)*base_odom + constant.
// No world truth, extra crack points, contact authority or global localization.
namespace va15_tcp {
class TerminalVisualScale {
public:
    struct Sample {double base=0.,cap=0.;};
    std::deque<Sample> samples;
    Pose anchor;
    V3 direction=V3::UnitX();
    bool valid=false,fault=false;
    double scale=1.,rms=0.,baseline=0.,slope_error=0.;
    unsigned consistent=0;
    std::uint64_t stamp=0;
    void Add(const Pose& capture,const V3& observed_cap,const V3& tangent) {
        if(!ValidPose(capture)||!observed_cap.allFinite()||!tangent.allFinite()||
           std::fabs(tangent.norm()-1.)>.001||capture.ns<=stamp)return;
        if(samples.empty())direction=tangent;
        if(direction.dot(tangent)<std::cos(.10))return;
        samples.push_back({capture.p.dot(direction),observed_cap.dot(direction)});
        while(samples.size()>64)samples.pop_front();
        stamp=capture.ns;anchor=capture;
        if(samples.size()<8)return;
        double x=0.,y=0.,low=INFINITY,high=-INFINITY;
        for(const auto& p:samples){x+=p.base;y+=p.cap;low=std::min(low,p.base);high=std::max(high,p.base);}
        x/=samples.size();y/=samples.size();baseline=high-low;
        if(baseline<.40)return;
        double xx=0.,xy=0.;
        for(const auto& p:samples){xx+=(p.base-x)*(p.base-x);xy+=(p.base-x)*(p.cap-y);}
        if(xx<.02)return;
        const double slope=xy/xx;double residual=0.;
        for(const auto& p:samples)residual+=std::pow(p.cap-y-slope*(p.base-x),2);
        rms=std::sqrt(residual/samples.size());
        slope_error=std::sqrt(residual/(samples.size()-2))/std::sqrt(xx);
        const double candidate=1.-slope;
        if(!std::isfinite(candidate)||candidate<.90||candidate>1.10||rms>.02||slope_error>.025) {
            if(valid)fault=true;
            consistent=0;return;
        }
        if(consistent&&std::fabs(candidate-last_)>.02)consistent=0;
        last_=candidate;
        // Overlapping fits are not independent observations. A sufficiently
        // precise fit over >=8 distinct caps and >=0.40 m is stronger evidence
        // than waiting for three near-identical fits after the cap leaves view.
        ++consistent;
        if(slope_error<=.01||consistent>=3){valid=true;scale=candidate;}
    }
    bool Correct(const Pose& raw,Pose& corrected)const {
        corrected=raw;
        if(fault)return false;
        if(!valid)return ValidPose(raw);
        if(!ValidPose(raw)||raw.ns<anchor.ns||raw.ns-anchor.ns>45000000000ULL)return false;
        const double along=(raw.p-anchor.p).dot(direction);
        const double shift=(scale-1.)*along;
        if(!std::isfinite(shift)||std::fabs(shift)>.25||along<-.10||along>3.5)return false;
        corrected.p+=direction*shift;
        return true;
    }
private:
    double last_=1.;
};
}
