#pragma once
#ifndef VA15_ESTIMATED_STOP_SIMULATION_ONLY
#error "This unqualified estimator/stop bridge is SIMULATION ONLY; never include in a real controller"
#endif

#include "rl_sdk.hpp"
#include "va15_independent_state.hpp"

// The archived FSM's container is reused without changing RobotState ABI or
// the archived actor/core. No field below is an actor observation. Crucially,
// sport_fresh stays FALSE: this is not a manufactured SportModeState packet.
namespace va15_sim_stop
{
inline constexpr char Source[]="SIMULATION_ONLY_independent_estimated_support";
inline VA15StopFeedback Feedback(const va15_independent::Result& result)
{
    VA15StopFeedback value;
    value.raw_dds.fill(std::numeric_limits<float>::quiet_NaN());
    value.lowstate_fresh=result.fresh&&result.input_valid&&result.sequence_valid;
    value.unavailable_reason="independent simulation estimate unavailable; no lowering";
    if(!value.lowstate_fresh || !result.initialized || !result.estimate_valid_for_review ||
       result.accepted_feet<2 || result.support_count<2 ||
       !std::isfinite(result.base_height)||!std::isfinite(result.planar_speed)||
       !std::isfinite(result.position_sigma)||!std::isfinite(result.velocity_sigma)||
       result.position_sigma<0. || result.position_sigma>=.05 ||
       result.velocity_sigma<0. || result.velocity_sigma>=.15)return value;
    // Preserve the observer's conservative two-sigma margins, not just means.
    value.base_height=result.base_height-2.*result.position_sigma;
    value.planar_speed=result.planar_speed+2.*result.velocity_sigma;
    if(!std::isfinite(value.base_height)||value.base_height<=0.f ||
       !std::isfinite(value.planar_speed)||value.planar_speed<0.f)return value;
    value.contacts_valid=value.height_valid=value.velocity_valid=true;
    const int native_to_policy[]={1,0,3,2};
    for(int i=0;i<4;++i)value.contacts[i]=result.load_candidate[native_to_policy[i]];
    value.unavailable_reason=Source;
    return value;
}
}

inline bool VA15SimEstimatedStopValid(const VA15StopFeedback& value)
{
    return !value.sport_fresh && value.unavailable_reason==va15_sim_stop::Source &&
        value.lowstate_fresh&&value.contacts_valid&&value.height_valid&&value.velocity_valid&&
        std::isfinite(value.base_height)&&value.base_height>0.f&&
        std::isfinite(value.planar_speed)&&value.planar_speed>=0.f;
}
inline bool VA15SimEstimatedStopSettled(const VA15StopFeedback& value,float tilt,
    float gyro,float joint_speed,float min_height,float max_tilt)
{
    return VA15SimEstimatedStopValid(value) &&
        std::all_of(value.contacts.begin(),value.contacts.end(),[](bool v){return v;}) &&
        std::isfinite(tilt)&&tilt>=0.f&&std::isfinite(gyro)&&gyro>=0.f&&
        std::isfinite(joint_speed)&&joint_speed>=0.f&&
        std::isfinite(min_height)&&min_height>0.f&&std::isfinite(max_tilt)&&max_tilt>0.f&&
        value.base_height>=min_height&&tilt<=max_tilt&&
        value.planar_speed<.04f&&gyro<.18f&&joint_speed<.55f;
}
