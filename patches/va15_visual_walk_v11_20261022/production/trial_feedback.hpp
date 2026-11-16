#pragma once
#include "rl_sdk.hpp"
#include "va15_independent_state.hpp"

// Experimental bridge ONLY in the isolated forward-trial executable.
// This is NOT a SportModeState packet, contact calibration or hardware
// qualification. Original production validators reject these frames.
namespace va15_trial
{
inline constexpr char Source[]="EXPERIMENTAL_MODEL_SUPPORT_NOT_CERTIFIED";
inline constexpr char HealthOnly[]="EXPERIMENTAL_HEALTHY_PREDICTION_NO_SUPPORT_PERMISSION";
inline bool Healthy(const va15_independent::Result& r)
{
    return r.fresh && r.input_valid && r.sequence_valid && r.initialized &&
        std::isfinite(r.base_height) && std::isfinite(r.planar_speed) &&
        r.planar_speed>=0. && std::isfinite(r.position_sigma) &&
        r.position_sigma>=0. && r.position_sigma<.05 &&
        std::isfinite(r.velocity_sigma) && r.velocity_sigma>=0. && r.velocity_sigma<.15;
}
inline VA15StopFeedback Feedback(const va15_independent::Result& r)
{
    VA15StopFeedback f;
    f.raw_dds.fill(std::numeric_limits<float>::quiet_NaN());
    f.lowstate_fresh=r.fresh && r.input_valid && r.sequence_valid;
    f.unavailable_reason="experimental estimate unavailable; no motion/capture permission";
    if(!Healthy(r)) return f;
    f.unavailable_reason=HealthOnly;
    if(!r.estimate_valid_for_review || r.accepted_feet<2 || r.support_count<2) return f;
    f.base_height=r.base_height-2.*r.position_sigma;
    f.planar_speed=r.planar_speed+2.*r.velocity_sigma;
    if(!std::isfinite(f.base_height)||f.base_height<=0.f||!std::isfinite(f.planar_speed)||f.planar_speed<0.f)return f;
    f.contacts_valid=f.height_valid=f.velocity_valid=true;
    constexpr int map[]={1,0,3,2}; // native FR FL RR RL -> policy FL FR RL RR
    for(int i=0;i<4;++i)f.contacts[i]=r.load_candidate[map[i]];
    f.unavailable_reason=Source; // sport_fresh deliberately remains FALSE
    return f;
}
inline bool EstimatedValid(const VA15StopFeedback& f)
{
    return f.unavailable_reason==Source && !f.sport_fresh && f.lowstate_fresh &&
        f.contacts_valid && f.height_valid && f.velocity_valid &&
        std::isfinite(f.base_height) && f.base_height>0.f &&
        std::isfinite(f.planar_speed) && f.planar_speed>=0.f;
}
inline bool FeedbackValid(const VA15StopFeedback& f)
{return VA15StopFeedbackValid(f) || EstimatedValid(f);}
inline bool HealthyFeedback(const VA15StopFeedback& f)
{return f.lowstate_fresh && (f.unavailable_reason==Source || f.unavailable_reason==HealthOnly);}
inline bool Settled(const VA15StopFeedback& f,float tilt,float gyro,float dq,float h,float max_tilt)
{
    if(VA15RealStopSettled(f,tilt,gyro,dq,h,max_tilt))return true;
    return EstimatedValid(f) &&
        std::all_of(f.contacts.begin(),f.contacts.end(),[](bool v){return v;}) &&
        std::isfinite(tilt)&&tilt>=0.f&&std::isfinite(gyro)&&gyro>=0.f&&
        std::isfinite(dq)&&dq>=0.f&&std::isfinite(h)&&h>0.f&&
        std::isfinite(max_tilt)&&max_tilt>0.f&&f.base_height>=h&&tilt<=max_tilt&&
        f.planar_speed<.04f&&gyro<.18f&&dq<.55f;
}
}
