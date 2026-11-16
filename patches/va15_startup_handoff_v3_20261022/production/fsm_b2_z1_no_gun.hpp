/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef B2_Z1_NO_GUN_FSM_HPP
#define B2_Z1_NO_GUN_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include "va15_stand_gains.hpp"
#include "va15_controlled_stop.hpp"
#include "trial_supervisor.hpp"
#include "va15_startup_handoff.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <utility>

namespace b2_z1_no_gun_fsm
{

inline std::vector<float> DeploymentStandPosition(RL &rl)
{
    auto stand = rl.params.Get<std::vector<float>>(
        "deployment_stand_dof_pos", {});
    if (stand.size() != static_cast<std::size_t>(
            rl.params.Get<int>("num_of_dofs")))
    {
        stand = rl.params.Get<std::vector<float>>("default_dof_pos") +
            rl.params.Get<std::vector<float>>("sim_dof_pos_offset");
    }
    return stand;
}

inline std::vector<float> DeploymentStandGain(RL &rl,const std::string& key,const std::string& fallback)
{
    auto gains=rl.params.Get<std::vector<float>>(key,{});
    if(gains.empty()) gains=rl.params.Get<std::vector<float>>(fallback);
    if(gains.size()!=18) throw std::runtime_error("Invalid stand gain dimensions");
    for(float v:gains) if(!std::isfinite(v) || v<0)
        throw std::runtime_error("Invalid stand gain value");
    return gains;
}

inline std::vector<float> DeploymentLyingArmPosition(RL &rl)
{
    auto arm = rl.params.Get<std::vector<float>>(
        "deployment_lying_arm_dof_pos", {});
    if (arm.size() != 6)
    {
        // Z1 compact/home pose. The welding gun must be physically removed
        // before entering this prone transport configuration.
        arm = {0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f};
    }
    return arm;
}

inline void SetArmPosition(std::vector<float> &position,
                           const std::vector<float> &arm)
{
    static const std::vector<int> indices = {8, 13, 14, 15, 16, 17};
    if (position.size() < 18 || arm.size() != indices.size())
    {
        return;
    }
    for (std::size_t joint = 0; joint < indices.size(); ++joint)
    {
        position[indices[joint]] = arm[joint];
    }
}

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = rl.params.Get<std::vector<float>>("fixed_kd")[i];
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    static bool IsArmJoint(int index)
    {
        return index == 8 || index >= 13;
    }

    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    float percent_arm_deploy = 0.0f;
    int post_getup_hold_cycles = 0;
    int settle_window_cycles = 0;
    std::vector<float> settle_window_start_q;
    bool locomotion_requested = false;
    bool lowering_requested = false;
    bool post_getup_hold_reported = false;
    bool work_pose_reported = false;
    static constexpr int kMinimumPostGetUpHoldCycles = 200; // 1 s at 200 Hz
    static constexpr int kSettleWindowCycles = 100;         // 0.5 s
    static constexpr float kStableLegPositionDelta = 0.03f;
    // Isaac Lab policy order: hips, thighs, joint1, calves, joint2..joint6.
    // Keep Z1 at its default pose while B2 transitions through the crouched pose.
    std::vector<float> pre_running_pos = {
         0.10, -0.10,  0.10, -0.10,
         1.36,  1.36,  1.36,  1.36,  0.00,
        -2.65, -2.65, -2.65, -2.65,  0.10, -0.10,
         0.00,  0.00,  0.00
    };
    bool stand_from_passive = true;

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        percent_arm_deploy = 0.0f;
        post_getup_hold_cycles = 0;
        settle_window_cycles = 0;
        settle_window_start_q.clear();
        locomotion_requested = false;
        lowering_requested = false;
        post_getup_hold_reported = false;
        work_pose_reported = false;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
        SetArmPosition(pre_running_pos, DeploymentLyingArmPosition(rl));
    }

    void Run() override
    {
        const auto stand_pos = DeploymentStandPosition(rl);
        if(stand_from_passive)
        {

            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 1.0f, "Pre Getting up", true))
            {
                return;
            }
            auto standing_folded_pos = stand_pos;
            SetArmPosition(standing_folded_pos, DeploymentLyingArmPosition(rl));
            if (Interpolate(percent_getup, pre_running_pos,
                            standing_folded_pos, 2.0f,
                            "Getting up with Z1 stowed", true))
            {
                return;
            }
            if (Interpolate(percent_arm_deploy, standing_folded_pos,
                            stand_pos, 2.0f,
                            "Deploying Z1 to welding work pose", true))
            {
                return;
            }
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, stand_pos, 1.0f, "Getting up", true))
            {
                return;
            }
        }

        if (!work_pose_reported)
        {
            work_pose_reported = true;
            std::cout << std::endl << LOGGER::NOTE
                      << "Standing target interpolation completed; physical standing is not yet confirmed. "
                      << "Wait for settling; keep welding gun and hose removed during this test." << std::endl;
        }

        // Interpolate stops writing once complete. Explicitly hold the final
        // stance with extra damping while waiting for a stable policy handoff.
        const auto fixed_kp=rl.params.Get<std::vector<float>>("fixed_kp");
        const auto stand_kp=DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
        const auto stand_kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
        const float ramp=std::max(.5f,rl.params.Get<float>("deployment_stand_gain_ramp_s",2.f));
        const float progress=post_getup_hold_cycles*rl.params.Get<float>("dt")/ramp;
        const auto kp=VA15BlendGains(fixed_kp,stand_kp,progress);
        const auto kd=VA15BlendGains(std::vector<float>(18,8.f),stand_kd,progress);
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.q[i] = stand_pos[i];
            fsm_command->motor_command.dq[i] = 0.0f;
            fsm_command->motor_command.kp[i] = kp[i];
            fsm_command->motor_command.kd[i] = kd[i];
            fsm_command->motor_command.tau[i] = 0.0f;
        }

        ++post_getup_hold_cycles;
        float max_leg_velocity = 0.0f;
        float max_leg_target_error = 0.0f;
        int worst_leg_joint=0;
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            if (!IsArmJoint(i))
            {
                max_leg_velocity = std::max(
                    max_leg_velocity, std::fabs(fsm_state->motor_state.dq[i]));
                max_leg_target_error = std::max(
                    max_leg_target_error,
                    std::fabs(fsm_state->motor_state.q[i] - stand_pos[i]));
                if(std::fabs(fsm_state->motor_state.q[i]-stand_pos[i]) >= max_leg_target_error)
                    worst_leg_joint=i;
            }
        }
        if (post_getup_hold_cycles >= kMinimumPostGetUpHoldCycles &&
            !post_getup_hold_reported)
        {
            if (settle_window_cycles == 0)
            {
                settle_window_start_q = fsm_state->motor_state.q;
            }
            ++settle_window_cycles;
            if (settle_window_cycles >= kSettleWindowCycles)
            {
                float max_leg_position_delta = 0.0f;
                for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
                {
                    if (!IsArmJoint(i))
                    {
                        max_leg_position_delta = std::max(
                            max_leg_position_delta,
                            std::fabs(fsm_state->motor_state.q[i] -
                                      settle_window_start_q[i]));
                    }
                }

                const float maximum_stand_error = rl.params.Get<float>(
                    "deployment_stand_max_joint_error", 0.12f);
                if (progress >= 1.f && max_leg_position_delta <= kStableLegPositionDelta &&
                    max_leg_target_error <= maximum_stand_error)
                {
                    post_getup_hold_reported = true;
                    std::cout << std::endl << LOGGER::INFO
                              << "Post-getup settling completed; max_leg_dq="
                              << max_leg_velocity
                              << ", max_leg_q_delta=" << max_leg_position_delta
                              << ", max_leg_target_error=" << max_leg_target_error
                              << ", policy handoff enabled" << std::endl;
                }
                else
                {
                    std::cout << std::endl << LOGGER::INFO
                              << "Post-getup settling: max_leg_dq="
                              << max_leg_velocity
                              << ", max_leg_q_delta=" << max_leg_position_delta
                              << ", max_leg_target_error=" << max_leg_target_error
                              << ", joint=" << rl.params.Get<std::vector<std::string>>("joint_names")[worst_leg_joint]
                              << ", actual=" << fsm_state->motor_state.q[worst_leg_joint]
                              << ", target=" << stand_pos[worst_leg_joint]
                              << ", kp=" << kp[worst_leg_joint]
                              << ", tau_est=" << fsm_state->motor_state.tau_est[worst_leg_joint]
                              << std::endl;
                    settle_window_cycles = 0;
                }
            }
        }
    }

    void Exit() override {}

    void MakeArmPassive()
    {
        for (int index : {8, 13, 14, 15, 16, 17})
        {
            fsm_command->motor_command.q[index] = fsm_state->motor_state.q[index];
            fsm_command->motor_command.dq[index] = 0.0f;
            fsm_command->motor_command.kp[index] = 0.0f;
            fsm_command->motor_command.kd[index] = 1.0f;
            fsm_command->motor_command.tau[index] = 0.0f;
        }
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            lowering_requested=true;
            locomotion_requested=false;
        }
        if(lowering_requested)
        {
            rl.control.x=rl.control.y=rl.control.yaw=0.f;
            return percent_getup>=1.f ? "RLFSMStateGetDown" : state_name_;
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
            rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            locomotion_requested = true;
            if (!post_getup_hold_reported)
            {
                std::cout << std::endl << LOGGER::INFO
                          << "Locomotion requested; waiting for post-getup settling"
                          << std::endl;
            }
        }
        if (percent_getup >= 1.0f)
        {
            if (locomotion_requested && post_getup_hold_reported)
            {
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    explicit RLFSMStateGetDown(RL* value):RLFSMState(*value,"RLFSMStateGetDown"){}
    RobotCommand<float> entry,held;
    std::vector<float> target,fixed_kp,fixed_kd;
    double elapsed=0.,arm_progress=0.,body_progress=0.,quiet=0.;
    bool first=true,blocked=false,complete=false,body_paused=false;
    const char* reason="not entered";
    static constexpr double Duration=3.; // smoothstep peak slope equals old 2 s linear slope
    static bool Arm(int i){return i==8||i>=13;}
    static float Blend(double p){return static_cast<float>(p*p*(3.-2.*p));}
    static bool Vector(const std::vector<float>& v)
    {return v.size()==18&&std::all_of(v.begin(),v.end(),[](float x){return std::isfinite(x);});}
    static bool Command(const RobotCommand<float>& c)
    {return Vector(c.motor_command.q)&&Vector(c.motor_command.dq)&&Vector(c.motor_command.kp)&&
        Vector(c.motor_command.kd)&&Vector(c.motor_command.tau);}
    void Block(const char* message)
    {
        blocked=true;reason=message;
        std::cout<<std::endl<<LOGGER::WARNING<<"[GetDown] "<<reason
                 <<"; holding last command, operator intervention required"<<std::endl;
    }
    void Enter() override
    {
        elapsed=arm_progress=body_progress=quiet=0.;first=true;blocked=complete=body_paused=false;
        entry=held=*fsm_command;target=rl.start_state.motor_state.q;
        SetArmPosition(target,DeploymentLyingArmPosition(rl));
        fixed_kp=rl.params.Get<std::vector<float>>("fixed_kp");
        fixed_kd=rl.params.Get<std::vector<float>>("fixed_kd");
        if(!Command(entry)||!Vector(target)||!Vector(fixed_kp)||!Vector(fixed_kd))
        {Block("invalid entry/config; no automatic lowering");return;}
        for(int i=0;i<18;++i)
            if(fixed_kp[i]<=0||fixed_kd[i]<0||entry.motor_command.kp[i]<=0||entry.motor_command.kd[i]<0)
            {Block("invalid entry gains; no automatic lowering");return;}
        reason="parallel arm stow + B2 lowering; tool must already be removed";
        std::cout<<std::endl<<LOGGER::NOTE<<reason<<std::endl;
    }
    void Run() override
    {
        if(blocked||complete){*fsm_command=held;return;}
        const double dt=rl.params.Get<float>("dt");
        if(!std::isfinite(dt)||dt<=0||dt>.02||!Vector(fsm_state->motor_state.q)||!Vector(fsm_state->motor_state.dq))
        {Block("invalid measured pose/time");*fsm_command=held;return;}
        // Exact command continuity, including gains, dq and feedforward.
        if(first){first=false;*fsm_command=held;return;}
        elapsed+=dt;
        if(elapsed>10.)
        {Block("parallel lowering did not finish; no timed passive release");*fsm_command=held;return;}
        float arm_lag=0.f;
        for(int i=0;i<18;++i)if(Arm(i))
            arm_lag=std::max(arm_lag,std::fabs(fsm_state->motor_state.q[i]-held.motor_command.q[i]));
        arm_progress=std::min(1.,arm_progress+dt/Duration);
        const bool previous_pause=body_paused;
        body_paused=arm_lag>.20f;
        if(body_paused!=previous_pause)
            std::cout<<std::endl<<LOGGER::WARNING<<"[GetDown] "
                     <<(body_paused?"B2 descent paused: arm tracking lag":"Arm tracking recovered; B2 descent resumed")
                     <<" (rad="<<arm_lag<<")"<<std::endl;
        // Arm keeps folding if it lags; B2 stops descending rather than
        // waiting for 100% stow before it may begin its first lowering step.
        if(!body_paused)body_progress=std::min(arm_progress,body_progress+dt/Duration);
        for(int i=0;i<18;++i)
        {
            const float s=Blend(Arm(i)?arm_progress:body_progress);
            auto& c=held.motor_command;const auto& initial=entry.motor_command;
            c.q[i]=(1.f-s)*initial.q[i]+s*target[i];
            c.kp[i]=(1.f-s)*initial.kp[i]+s*fixed_kp[i];
            c.kd[i]=(1.f-s)*initial.kd[i]+s*fixed_kd[i];
            c.dq[i]=(1.f-s)*initial.dq[i];c.tau[i]=(1.f-s)*initial.tau[i];
        }
        float arm_error=0.f,leg_error=0.f,max_dq=0.f;
        for(int i=0;i<18;++i)
        {
            const float error=std::fabs(fsm_state->motor_state.q[i]-target[i]);
            if(Arm(i))arm_error=std::max(arm_error,error);else leg_error=std::max(leg_error,error);
            max_dq=std::max(max_dq,std::fabs(fsm_state->motor_state.dq[i]));
        }
        const bool finished=arm_progress>=1.&&body_progress>=1.&&arm_error<.12f&&leg_error<.18f&&max_dq<.5f;
        quiet=finished?quiet+dt:0.;complete=quiet+1.e-9>=.20;
        if(complete)std::cout<<std::endl<<LOGGER::INFO<<"[GetDown] Parallel arm stow and B2 lowering completed"<<std::endl;
        reason=complete?"parallel lowering finished with quiet measured joints":
            (body_paused?"B2 lowering paused: arm lag exceeds 0.20 rad":"parallel arm stow + B2 lowering");
        *fsm_command=held;
    }
    void Exit() override{}
    std::string CheckChange() override
    {
        if(rl.control.current_keyboard==Input::Keyboard::P||rl.control.current_gamepad==Input::Gamepad::LB_X)
            return "RLFSMStatePassive";
        if(complete)return "RLFSMStatePassive";
        if(!blocked&&(rl.control.current_keyboard==Input::Keyboard::Num0||rl.control.current_gamepad==Input::Gamepad::A))
            return "RLFSMStateGetUp";
        return state_name_;
    }
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    float percent_transition = 0.0f;
    bool policy_started = false;
    bool policy_running = false;
    bool gait_starter_active = false;
    bool gait_starter_policy_warmup = false;
    float gait_starter_time = 0.0f;
    int gait_starter_prepare_cycles = 0;
    std::vector<float> gait_starter_prepare_start_q;
    std::vector<float> gait_starter_symmetric_start_q;
    float gait_prepare_max_tilt = 0.0f;
    float gait_prepare_max_leg_dq = 0.0f;
    float gait_replay_max_tilt = 0.0f;
    float gait_replay_max_leg_dq = 0.0f;
    int motion_start_hold_cycles = 0;
    int policy_stop_wait_cycles = 0;
    int policy_stop_stable_cycles = 0;
    int real_start_support_cycles = 0;
    bool real_start_inhibited = false;
    bool captured_support_hold_active = false;
    VA15ControlledStop controlled_stop;
    va15_trial::Supervisor trial;
    bool trial_configured=false;
    va15_trial::Supervisor::Stage reported_trial_stage=va15_trial::Supervisor::Stage::Preview;
    VA15ControlledStop::Stage reported_controlled_stop_stage = VA15ControlledStop::Stage::Idle;
    int stop_settle_cycles = 0;
    std::vector<float> stop_settle_start_q;
    std::vector<float> standby_hold_q;
    std::vector<float> policy_entry_q;
    std::vector<float> filtered_leg_q;
    std::vector<float> filtered_arm_q;
    std::vector<float> latest_policy_q;
    std::vector<float> latest_policy_dq;
    float leg_offset_scale = 1.0f;
    int front_handoff_cycles = 0;
    int rear_handoff_cycles = 0;
    std::vector<float> rear_handoff_start_q;
    std::vector<float> gait_handoff_offset_q;
    bool gait_handoff_offset_initialized = false;
    bool policy_frame_received_this_cycle = false;
    VA15StartupHandoff startup_handoff;
    bool startup_join_enabled=true;
    bool startup_join_for_motion=false;
    bool getdown_requested = false;
    int handoff_hold_cycles = 0;
    int arm_blend_cycles = 0;
    static constexpr int kPolicyWarmupCycles = 400; // 2.0 s at 200 Hz
    // Remove only the initial arm target discontinuity.  This does not slow
    // the learned arm motion after takeover and therefore avoids the TCP lag
    // caused by using an excessively slow permanent output filter.
    static constexpr int kArmBlendCycles = 400; // 2.0 s at 200 Hz
    static constexpr int kGaitStarterPrepareCycles = 300; // 1.5 s at 200 Hz
    // Preserve the moving AMP phase, but release the positional difference
    // between its final physical target and the first learned target over
    // 0.75 seconds. An instantaneous release starts with ~0.22 rad body tilt
    // and drops the support diagonal before the policy can recover.  The
    // former 0.5 s release still left one visible impulse on trajectory entry.
    static constexpr int kFrontHandoffCycles = 300; // 1.50 s at 200 Hz
    static constexpr int kRearHandoffCycles = 300;  // 1.50 s at 200 Hz
    // At zero command, capture only an upright four-foot support phase. A long
    // quiet window is actively harmful for this checkpoint because its
    // zero-speed policy gradually lowers B2 onto its knees. Contact, height,
    // and attitude checks make a short support window the safer criterion.
    static constexpr int kPolicyStopStableCycles = 30; // 0.15 s at 200 Hz
    static constexpr int kPolicyStopMaxWaitCycles = 600; // report every 3.0 s
    static constexpr int kStopSettleHoldCycles = 400; // 2.0 s at 200 Hz
    std::vector<float> previous_leg_target_q;
    std::vector<float> previous_leg_actual_q;
    std::vector<float> per_joint_target_travel;
    std::vector<float> per_joint_actual_travel;
    float leg_target_travel = 0.0f;
    float leg_actual_travel = 0.0f;
    unsigned long long diagnostic_control_cycles = 0;
    unsigned long long last_motion_diagnostic_episode = ~0ULL;

    bool RealStopSettled() const
    {
        if (!fsm_state || fsm_state->imu.quaternion.size() != 4 ||
            fsm_state->imu.gyroscope.size() != 3 || fsm_state->motor_state.dq.size() != 18)
            return false;
        const auto gravity = QuatRotateInverse(fsm_state->imu.quaternion, {0.f, 0.f, -1.f});
        float gyro = 0.f, speed = 0.f, norm = 0.f;
        for (float value : fsm_state->imu.quaternion)
        {
            if (!std::isfinite(value)) return false;
            norm += value * value;
        }
        if (norm < .9f || norm > 1.1f) return false;
        for (float value : fsm_state->imu.gyroscope)
        {
            if (!std::isfinite(value)) return false;
            gyro += value * value;
        }
        for (int i : VA15LegIndices)
        {
            if (!std::isfinite(fsm_state->motor_state.dq[i])) return false;
            speed = std::max(speed, std::fabs(fsm_state->motor_state.dq[i]));
        }
        return va15_trial::Settled(fsm_state->real_stop_feedback,
            std::hypot(gravity[0], gravity[1]), std::sqrt(gyro), speed,
            rl.params.Get<float>("stop_min_base_height", .48f),
            rl.params.Get<float>("stop_max_body_tilt", .15f));
    }

    std::string StopContactsString() const
    {
        std::string value;
        if (rl.RequiresRealStopFeedback())
        {
            if (!fsm_state->real_stop_feedback.contacts_valid) return "????";
            for (bool contact : fsm_state->real_stop_feedback.contacts) value += contact ? '1' : '0';
        }
        else for (int i = 0; i < 4; ++i)
            value += rl.obs.foot_contacts.size() > static_cast<std::size_t>(i) &&
                rl.obs.foot_contacts[i] ? '1' : '0';
        return value;
    }

    void SyncActionObservationFromJointTargets(
        const std::vector<float> &joint_targets)
    {
        if(rl.params.Get<bool>("use_raw_policy_action_observation",false)) return;
        const auto default_dof_pos =
            rl.params.Get<std::vector<float>>("default_dof_pos");
        const auto action_scale =
            rl.params.Get<std::vector<float>>("action_scale");
        std::vector<float> applied_actions(default_dof_pos.size(), 0.0f);
        for (std::size_t i = 0; i < applied_actions.size(); ++i)
        {
            if (std::fabs(action_scale[i]) > 1.0e-6f)
            {
                applied_actions[i] =
                    (joint_targets[i] - default_dof_pos[i]) /
                    action_scale[i];
            }
        }

        // Forward() reads the last-action observation while holding this same
        // mutex. Keep it equal to the command that actually reached the PD
        // controller after stance hold, handoff blending, and arm filtering.
        std::lock_guard<std::mutex> lock(rl.model_mutex);
        rl.obs.actions = std::move(applied_actions);
    }

    void SyncAppliedActionObservation(bool force = false)
    {
        if (!force &&
            !rl.params.Get<bool>("sync_applied_action_observation", false))
        {
            return;
        }
        SyncActionObservationFromJointTargets(
            fsm_command->motor_command.q);
    }

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;
        policy_started = true;
        if(!trial_configured)
        {trial.Configure(rl.params.Get<bool>("isolated_forward_trial_motion",false));trial_configured=true;}
        else trial.Entry(); // never restore an already consumed request budget
        policy_running = false;
        gait_starter_active = false;
        gait_starter_policy_warmup = false;
        gait_starter_time = 0.0f;
        gait_starter_prepare_cycles = 0;
        gait_starter_prepare_start_q.clear();
        gait_starter_symmetric_start_q.clear();
        gait_prepare_max_tilt = 0.0f;
        gait_prepare_max_leg_dq = 0.0f;
        gait_replay_max_tilt = 0.0f;
        gait_replay_max_leg_dq = 0.0f;
        motion_start_hold_cycles = 0;
        policy_stop_wait_cycles = 0;
        policy_stop_stable_cycles = 0;
        real_start_support_cycles = 0;
        real_start_inhibited = false;
        captured_support_hold_active = false;
        controlled_stop.Reset();
        reported_controlled_stop_stage = VA15ControlledStop::Stage::Idle;
        stop_settle_cycles = 0;
        stop_settle_start_q.clear();
        rl.rl_init_done = false;
        rl.motion_heading_anchor_valid = false;
        rl.motion_heading_quat = {1.0f, 0.0f, 0.0f, 0.0f};
        rl.control.x = 0.0f;
        rl.control.y = 0.0f;
        rl.control.yaw = 0.0f;
        rl.control.current_keyboard = Input::Keyboard::None;
        standby_hold_q = fsm_state->motor_state.q;
        policy_entry_q = fsm_state->motor_state.q;
        filtered_leg_q = standby_hold_q;
        filtered_arm_q = standby_hold_q;
        latest_policy_q = standby_hold_q;
        latest_policy_dq.assign(standby_hold_q.size(), 0.0f);
        leg_offset_scale = 1.0f;
        front_handoff_cycles = 0;
        rear_handoff_cycles = 0;
        rear_handoff_start_q = standby_hold_q;
        gait_handoff_offset_q.assign(standby_hold_q.size(), 0.0f);
        gait_handoff_offset_initialized = false;
        policy_frame_received_this_cycle = false;
        startup_handoff.Reset();
        startup_join_enabled=rl.params.Get<bool>("startup_torque_handoff_candidate",true);
        startup_join_for_motion=false;
        getdown_requested = false;
        handoff_hold_cycles = kPolicyWarmupCycles;
        arm_blend_cycles = kArmBlendCycles;
        previous_leg_target_q = standby_hold_q;
        previous_leg_actual_q = standby_hold_q;
        per_joint_target_travel.assign(standby_hold_q.size(), 0.0f);
        per_joint_actual_travel.assign(standby_hold_q.size(), 0.0f);
        leg_target_travel = 0.0f;
        leg_actual_travel = 0.0f;
        diagnostic_control_cycles = 0;
        last_motion_diagnostic_episode = ~0ULL;

        std::vector<float> discarded_output;
        while (rl.output_dof_pos_queue.try_pop(discarded_output)) {}
        while (rl.output_dof_vel_queue.try_pop(discarded_output)) {}
        while (rl.output_dof_tau_queue.try_pop(discarded_output)) {}

        // read params from yaml
        rl.config_name = rl.params.Get<std::string>("deployment_policy_profile", "robot_lab");
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            if (rl.params.Get<bool>("enable_forward_gait_starter", false))
            {
                const std::string motion_file_path =
                    std::string(POLICY_DIR) + "/" + robot_config_path + "/" +
                    rl.params.Get<std::string>("forward_gait_starter_file");
                rl.motion_loader = std::make_unique<MotionLoader>(
                    motion_file_path,
                    rl.params.Get<float>("forward_gait_starter_fps"));
                std::cout << LOGGER::INFO
                          << "Forward gait starter loaded: "
                          << motion_file_path
                          << ", duration=" << rl.motion_loader->GetDuration()
                          << " s" << std::endl;
            }
            // Do not run the policy while the command is zero. This policy was
            // trained with rel_standing_envs=0 and reference-state
            // initialization, so free-running it behind a held stance creates
            // a last-action history that was never physically applied.
            rl.rl_init_done = false;
            // GetUp already commands this stance. Keep commanding that same
            // target during policy warmup instead of freezing the sagged
            // measured pose captured while the joints are still converging.
            const auto stand_target = DeploymentStandPosition(rl);
            for (int index = 0;
                 index < rl.params.Get<int>("num_of_dofs"); ++index)
            {
                standby_hold_q[index] = stand_target[index];
                filtered_leg_q[index] = stand_target[index];
                filtered_arm_q[index] = stand_target[index];
                latest_policy_q[index] = stand_target[index];
            }
            // Pre-position Z1 while all four legs are still held in the
            // high-damping standing state.  Without this step the physical
            // TCP is near ground level at T, whereas the trained rear target
            // is about 0.10 m above the ground.  Asking the policy to lift the
            // complete arm at the same instant that it starts its first trot
            // diagonal creates a rearward pitch impulse.  The configured pose
            // is the measured steady Z1 pose of this checkpoint at 0.30 m/s.
            const auto standby_arm = rl.params.Get<std::vector<float>>(
                "policy_standby_arm_pos", {});
            const int arm_indices[] = {8, 13, 14, 15, 16, 17};
            if (standby_arm.size() == 6)
            {
                for (std::size_t arm = 0; arm < 6; ++arm)
                {
                    const int index = arm_indices[arm];
                    standby_hold_q[index] = standby_arm[arm];
                    filtered_arm_q[index] = standby_arm[arm];
                    latest_policy_q[index] = standby_arm[arm];
                }
            }
            std::cout << std::endl << LOGGER::INFO
                      << "Policy entry joint state (policy order):" << std::endl;
            for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
            {
                std::cout << LOGGER::INFO
                          << "  [" << i << "] "
                          << rl.params.Get<std::vector<std::string>>("joint_names")[i]
                          << " q=" << fsm_state->motor_state.q[i]
                          << " dq=" << fsm_state->motor_state.dq[i]
                          << std::endl;
            }
            std::cout << std::endl << LOGGER::INFO
                      << "Policy loaded on RLLocomotion entry" << std::endl;
            std::cout << LOGGER::INFO
                      << "Policy warmup started; holding GetUp stand target for 2.0 s"
                      << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // Consume B before a possible successful-stop transition in this Run.
        // FSM::Run calls CheckChange only AFTER Run; otherwise a same-frame B
        // can be lost when Run requests GetUp and GetUp resets its own latch.
        const auto previous_applied_command=*fsm_command;
        if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
            rl.control.current_gamepad == Input::Gamepad::B)
            getdown_requested = true;
        if (getdown_requested) rl.control.x = rl.control.y = rl.control.yaw = 0.f;
        if (rl.RequiresRealStopFeedback() && (policy_running || gait_starter_active) &&
            (rl.control.current_keyboard == Input::Keyboard::Num0 ||
             rl.control.current_keyboard == Input::Keyboard::Num1 ||
             rl.control.current_gamepad == Input::Gamepad::A ||
             rl.control.current_gamepad == Input::Gamepad::RB_DPadUp))
            rl.control.x = rl.control.y = rl.control.yaw = 0.f;
        if (rl.RequiresRealStopFeedback())
            real_start_support_cycles = RealStopSettled()
                ? std::min(real_start_support_cycles + 1, 100) : 0;
        if (rl.params.Get<bool>("independent_B_stop_candidate",false) &&
            !policy_running && !gait_starter_active && real_start_support_cycles>=100)
            controlled_stop.ObserveStanding(*fsm_state,*fsm_command);

        if (handoff_hold_cycles > 0)
        {
            const auto kp = DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
            const auto hold_kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
            const float handoff_progress =
                1.0f - static_cast<float>(handoff_hold_cycles) /
                           static_cast<float>(kPolicyWarmupCycles);
            const float handoff_blend = handoff_progress * handoff_progress *
                (3.0f - 2.0f * handoff_progress);
            for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
            {
                const bool is_arm = (i == 8 || i >= 13);
                // Keep B2 exactly at its verified standing pose while Z1 is
                // moved into its policy workspace with zero endpoint step.
                fsm_command->motor_command.q[i] = is_arm
                    ? (1.0f - handoff_blend) * policy_entry_q[i] +
                          handoff_blend * standby_hold_q[i]
                    : standby_hold_q[i];
                fsm_command->motor_command.dq[i] = 0.0f;
                fsm_command->motor_command.kp[i] = kp[i];
                fsm_command->motor_command.kd[i] = hold_kd[i];
                fsm_command->motor_command.tau[i] = 0.0f;
            }

            // Let the action-history observation settle, but never replay the
            // transient outputs produced during policy startup.
            std::vector<float> discarded_output;
            while (rl.output_dof_pos_queue.try_pop(discarded_output)) {}
            while (rl.output_dof_vel_queue.try_pop(discarded_output)) {}
            while (rl.output_dof_tau_queue.try_pop(discarded_output)) {}

            SyncAppliedActionObservation(true);

            --handoff_hold_cycles;
            if (handoff_hold_cycles == 0)
            {
                previous_leg_target_q = standby_hold_q;
                previous_leg_actual_q = fsm_state->motor_state.q;
                filtered_leg_q = standby_hold_q;
                leg_target_travel = 0.0f;
                leg_actual_travel = 0.0f;
                std::fill(per_joint_target_travel.begin(),
                          per_joint_target_travel.end(), 0.0f);
                std::fill(per_joint_actual_travel.begin(),
                          per_joint_actual_travel.end(), 0.0f);
                std::cout << std::endl << LOGGER::INFO
                          << "Policy warmup completed; starting smooth handoff"
                          << std::endl;
            }
            return;
        }

        if(rl.params.Get<bool>("isolated_forward_trial",false))
        {
            const bool key_stop=rl.control.current_keyboard==Input::Keyboard::Space ||
                rl.control.current_keyboard==Input::Keyboard::Num9 ||
                rl.control.current_gamepad==Input::Gamepad::B ||
                rl.control.current_gamepad==Input::Gamepad::X;
            const double command=trial.Step(rl.params.Get<float>("dt"),rl.control.x,rl.control.y,
                rl.control.yaw,RealStopSettled(),va15_trial::HealthyFeedback(fsm_state->real_stop_feedback),key_stop);
            rl.control.x=command;rl.control.y=rl.control.yaw=0.f;
            if(trial.stage!=reported_trial_stage)
            {
                std::cout<<std::endl<<LOGGER::WARNING<<"[Restricted trial] "<<trial.reason
                    <<" requested_s="<<trial.requested<<" not a hardware acceptance"<<std::endl;
                reported_trial_stage=trial.stage;
            }
        }

        // Isolated candidate: B catches a validated supported phase BEFORE
        // demanding quiet joints. The independent fixed-stand controller,
        // not the zero-speed actor, then earns permission to enter GetDown.
        // OFF by default; PC4 still uses the separately backed-up baseline.
        if (rl.params.Get<bool>("independent_B_stop_candidate", false) &&
            (getdown_requested || trial.Stopping()) && (policy_running || controlled_stop.Active()))
        {
            if (rl.control.current_keyboard == Input::Keyboard::P ||
                rl.control.current_gamepad == Input::Gamepad::LB_X) return;
            controlled_stop.Request();
            auto evidence = *fsm_state;
            if (!rl.RequiresRealStopFeedback())
            {
                // Physics-test ground truth ONLY; never use this branch in
                // the real binary, whose virtual override is fixed to true.
                auto& value=evidence.real_stop_feedback;
                value.lowstate_fresh=value.sport_fresh=true;
                value.contacts_valid=rl.obs.foot_contacts.size()==4;
                for(int i=0;i<4 && value.contacts_valid;++i) value.contacts[i]=rl.obs.foot_contacts[i]!=0;
                value.base_height=rl.obs.base_height;
                value.planar_speed=rl.obs.lin_vel.size()>=2
                    ? std::hypot(rl.obs.lin_vel[0],rl.obs.lin_vel[1])
                    : std::numeric_limits<float>::quiet_NaN();
                value.height_valid=std::isfinite(value.base_height) && value.base_height>0.f;
                value.velocity_valid=std::isfinite(value.planar_speed);
            }
            float command_speed=0.f;
            for(float v:rl.obs.commands) command_speed=std::max(command_speed,std::fabs(v));
            controlled_stop.Step(evidence,*fsm_command,DeploymentStandPosition(rl),
                DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp"),
                DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd"),
                rl.params.Get<float>("dt"),command_speed,
                rl.params.Get<float>("stop_min_base_height",.48f),
                rl.params.Get<float>("stop_max_body_tilt",.15f));
            if (controlled_stop.stage!=reported_controlled_stop_stage)
            {
                std::cout<<std::endl<<LOGGER::INFO<<"[Trial estimated stop] "
                    <<controlled_stop.reason<<" captured="<<controlled_stop.captured
                    <<" ready_to_lower="<<controlled_stop.Ready()<<std::endl;
                reported_controlled_stop_stage=controlled_stop.stage;
            }
            if (controlled_stop.OwnsCommand())
            {
                policy_running=false;
                rl.rl_init_done=false;
                rl.obs.commands.assign(3,0.f);
                rl.motion_heading_anchor_valid=false;
                std::vector<float> discarded;
                while(rl.output_dof_pos_queue.try_pop(discarded)){}
                while(rl.output_dof_vel_queue.try_pop(discarded)){}
                while(rl.output_dof_tau_queue.try_pop(discarded)){}
                return; // no actor frame may overwrite the caught posture
            }
        }

        if(startup_join_for_motion && !startup_handoff.ever_applied && !policy_running &&
            (startup_handoff.stage==VA15StartupHandoff::Stage::Refused ||
             startup_handoff.stage==VA15StartupHandoff::Stage::Cancelled))
        {
            // A refused/missing FIRST frame never entered moving PD. Keep
            // the COMPLETE last standing command on subsequent cycles too:
            // letting the legacy standby blend resume at scale~=0 would
            // otherwise drop its gains one cycle after a safe refusal.
            // CheckChange retains the original static B/emergency handling.
            *fsm_command=previous_applied_command;
            rl.control.x=rl.control.y=rl.control.yaw=0.f;
            return;
        }

        // Blend from the exact GetUp stance to the raw learned gait once
        // motion is requested. Release quickly enough that small real-robot
        // commands are not trapped in a half-stance/half-gait compromise.
        bool operator_motion_requested =
            std::fabs(rl.control.x) > 0.03f ||
            std::fabs(rl.control.y) > 0.03f ||
            std::fabs(rl.control.yaw) > 0.03f;
        if (rl.RequiresRealStopFeedback() && !policy_running && !gait_starter_active)
        {
            if (!operator_motion_requested) real_start_inhibited = false;
            else if (real_start_support_cycles < 100 || real_start_inhibited ||
                     (captured_support_hold_active && stop_settle_cycles > 0))
            {
                if (!real_start_inhibited)
                    std::cout << std::endl << LOGGER::WARNING
                              << "[Real stop gate] Motion refused: "
                              << (va15_trial::FeedbackValid(fsm_state->real_stop_feedback)
                                  ? "support/stand handoff is not ready"
                                  : fsm_state->real_stop_feedback.unavailable_reason)
                              << "; need 0.5 s fresh experimental estimated four-foot stationary support (not calibrated contacts). "
                              << "Zero input and explicitly request again after validation; A/B static test remains available."
                              << std::endl;
                real_start_inhibited = true;
                rl.control.x = rl.control.y = rl.control.yaw = 0.f;
                operator_motion_requested = false;
            }
        }
        // On release, keep the learned controller active while RunModel ramps
        // its command observation down. Freezing an arbitrary swing phase at
        // 0.4--0.5 m/s makes the robot tip before the stand blend can help.
        bool policy_command_active = false;
        if (policy_running)
        {
            for (const float command : rl.obs.commands)
            {
                policy_command_active =
                    policy_command_active || std::fabs(command) > 0.03f;
            }
        }
        const bool motion_requested =
            operator_motion_requested || policy_command_active;

        if (gait_starter_active)
        {
            if (!motion_requested)
            {
                gait_starter_active = false;
                gait_starter_policy_warmup = false;
                gait_starter_time = 0.0f;
                gait_starter_prepare_cycles = 0;
                rl.rl_init_done = false;
                policy_running = false;
                captured_support_hold_active = false;
                rl.obs.commands.assign(3, 0.0f);
                standby_hold_q = fsm_state->motor_state.q;
                stop_settle_start_q = standby_hold_q;
                stop_settle_cycles = kStopSettleHoldCycles;
                filtered_leg_q = standby_hold_q;
                filtered_arm_q = standby_hold_q;
                latest_policy_q = standby_hold_q;
                latest_policy_dq.assign(standby_hold_q.size(), 0.0f);
                leg_offset_scale = 1.0f;
                std::vector<float> discarded_output;
                while (rl.output_dof_pos_queue.try_pop(discarded_output)) {}
                while (rl.output_dof_vel_queue.try_pop(discarded_output)) {}
                while (rl.output_dof_tau_queue.try_pop(discarded_output)) {}
                std::cout << std::endl << LOGGER::INFO
                          << "Gait starter cancelled; locking the current support posture"
                          << std::endl;
            }
            else
            {
                ApplyGaitStarter();
                if (gait_starter_active)
                {
                    return;
                }
            }
        }

        if (motion_requested && !policy_running)
        {
            policy_stop_wait_cycles = 0;
            policy_stop_stable_cycles = 0;
            captured_support_hold_active = false;
            stop_settle_cycles = 0;
            rl.motion_heading_quat =
                QuaternionYawOnly(fsm_state->imu.quaternion);
            rl.motion_heading_anchor_valid = true;
            // Do not seed the first policy observation with the full operator
            // command. RunModel owns the configured acceleration limiter and
            // must ramp from the previous (normally zero) command. Directly
            // writing 0.30 m/s here bypassed that limiter on the exact frame
            // that T enabled vision mode and caused a visible whole-body jerk.
            if (rl.obs.commands.size() != 3)
            {
                rl.obs.commands.assign(3, 0.0f);
            }
            SyncAppliedActionObservation(true);
            std::vector<float> discarded_output;
            while (rl.output_dof_pos_queue.try_pop(discarded_output)) {}
            while (rl.output_dof_vel_queue.try_pop(discarded_output)) {}
            while (rl.output_dof_tau_queue.try_pop(discarded_output)) {}
            latest_policy_q = standby_hold_q;
            latest_policy_dq.assign(standby_hold_q.size(), 0.0f);
            filtered_leg_q = standby_hold_q;
            filtered_arm_q = standby_hold_q;
            leg_offset_scale = 1.0f;
            front_handoff_cycles = 0;
            rear_handoff_cycles = 0;
            gait_handoff_offset_initialized = false;
            arm_blend_cycles = kArmBlendCycles;
            const float gait_starter_turn_tolerance =
                rl.params.Get<float>(
                    "gait_starter_turn_tolerance", 0.03f);
            const bool forward_only =
                rl.control.x > 0.03f &&
                std::fabs(rl.control.y) <= gait_starter_turn_tolerance &&
                std::fabs(rl.control.yaw) <= gait_starter_turn_tolerance;
            if (forward_only &&
                rl.params.Get<bool>("enable_forward_gait_starter", false) &&
                rl.motion_loader)
            {
                gait_starter_active = true;
                gait_starter_policy_warmup = false;
                gait_starter_time = 0.0f;
                gait_starter_prepare_cycles = kGaitStarterPrepareCycles;
                gait_starter_prepare_start_q = fsm_state->motor_state.q;
                gait_starter_symmetric_start_q.clear();
                gait_prepare_max_tilt = 0.0f;
                gait_prepare_max_leg_dq = 0.0f;
                gait_replay_max_tilt = 0.0f;
                gait_replay_max_leg_dq = 0.0f;
                motion_start_hold_cycles = 0;
                rl.rl_init_done = false;
                std::cout << std::endl << LOGGER::INFO
                          << "Forward motion received; replaying AMP gait starter before policy handoff"
                          << std::endl;
                ApplyGaitStarter();
                return;
            }
            else
            {
                // Keep the prior COMPLETE command while the actor produces
                // its first frame. In particular, do not drop leg Kp yet.
                startup_join_for_motion=startup_join_enabled;
                if(startup_join_for_motion && !startup_handoff.Begin(previous_applied_command,
                    rl.params.Get<double>("startup_handoff_release_seconds",VA15StartupHandoff::FadeSeconds)))
                {
                    trial.Stop(startup_handoff.reason);
                    rl.control.x=rl.control.y=rl.control.yaw=0.f;
                    rl.rl_init_done=policy_running=false;
                    *fsm_command=previous_applied_command;
                    std::cout<<LOGGER::WARNING<<"[Startup handoff] refused before inference: "<<startup_handoff.reason<<std::endl;
                    return;
                }
                // This checkpoint already contains the training-side
                // FilteredJointPositionAction(alpha=0.20).  Start from the
                // filtered learned target directly.  Keeping the old
                // stand-policy blend here makes the actor observe an action
                // that is repeatedly withheld, so it winds up its output and
                // hits the leg target envelope when the blend is released.
                motion_start_hold_cycles = 0;
                leg_offset_scale = 0.0f;
                // The first moving policy frame is too far from the loaded
                // MuJoCo stand to apply directly. Preserve its learned gait
                // shape while a smooth additive offset joins it to the
                // measured stand pose.  A 1.5 s release matches the command
                // ramp and avoids both the direct rear-leg collapse and the
                // sharp 0.75 s release used by the older checkpoint.
                front_handoff_cycles = kFrontHandoffCycles;
                rear_handoff_cycles = kRearHandoffCycles;
                rear_handoff_start_q = standby_hold_q;
                gait_handoff_offset_initialized = false;
                {
                    std::lock_guard<std::mutex> lock(rl.model_mutex);
                    rl.obs.actions.assign(
                        rl.params.Get<int>("num_of_dofs"), 0.0f);
                    rl.reset_policy_action_state = true;
                }
                rl.rl_init_done = true;
                policy_running = true;
                std::cout << std::endl << LOGGER::INFO
                          << "Motion command received; policy history reset and inference started"
                          << std::endl;
            }
        }
        else if (!operator_motion_requested && policy_running && !controlled_stop.Active())
        {
            // Begin looking for a support phase as soon as the requested
            // command reaches zero, while RunModel smoothly ramps the policy
            // observation down. Waiting until obs.commands is already zero
            // lets this checkpoint enter its learned knee-down idle pose.
            ++policy_stop_wait_cycles;
            const float body_planar_speed = rl.RequiresRealStopFeedback()
                ? fsm_state->real_stop_feedback.planar_speed : rl.obs.lin_vel.size() >= 2
                ? std::hypot(rl.obs.lin_vel[0], rl.obs.lin_vel[1])
                : std::numeric_limits<float>::infinity();
            const float body_angular_speed = std::sqrt(
                fsm_state->imu.gyroscope[0] * fsm_state->imu.gyroscope[0] +
                fsm_state->imu.gyroscope[1] * fsm_state->imu.gyroscope[1] +
                fsm_state->imu.gyroscope[2] * fsm_state->imu.gyroscope[2]);
            const int leg_indices[] = {
                0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12};
            float max_leg_speed = 0.0f;
            for (const int index : leg_indices)
            {
                max_leg_speed = std::max(
                    max_leg_speed,
                    std::fabs(fsm_state->motor_state.dq[index]));
            }
            const auto projected_gravity = QuatRotateInverse(
                fsm_state->imu.quaternion, {0.0f, 0.0f, -1.0f});
            const float body_tilt = std::hypot(
                projected_gravity[0], projected_gravity[1]);
            const bool all_feet_contact = rl.RequiresRealStopFeedback()
                ? fsm_state->real_stop_feedback.contacts_valid &&
                    std::all_of(fsm_state->real_stop_feedback.contacts.begin(),
                        fsm_state->real_stop_feedback.contacts.end(), [](bool v) { return v; }) :
                rl.obs.foot_contacts.size() == 4 &&
                std::all_of(
                    rl.obs.foot_contacts.begin(),
                    rl.obs.foot_contacts.end(),
                    [](const int contact) { return contact != 0; });
            const float stop_min_base_height = rl.params.Get<float>(
                "stop_min_base_height", 0.48f);
            const float stop_max_body_tilt = rl.params.Get<float>(
                "stop_max_body_tilt", 0.15f);
            const float support_height = rl.RequiresRealStopFeedback()
                ? fsm_state->real_stop_feedback.base_height : rl.obs.base_height;
            const bool upright_four_foot_support =
                all_feet_contact &&
                !rl.obs.undesired_ground_contact &&
                support_height >= stop_min_base_height &&
                body_tilt <= stop_max_body_tilt;
            const bool physically_settled = rl.RequiresRealStopFeedback()
                ? RealStopSettled() : upright_four_foot_support &&
                body_planar_speed < 0.04f &&
                body_angular_speed < 0.18f &&
                max_leg_speed < 0.55f;
            policy_stop_stable_cycles = physically_settled
                ? policy_stop_stable_cycles + 1
                : 0;

            const bool stable_window_complete =
                policy_stop_stable_cycles >= kPolicyStopStableCycles;
            const bool wait_timed_out =
                policy_stop_wait_cycles >= kPolicyStopMaxWaitCycles;
            if (stable_window_complete)
            {
                if (!rl.RequiresRealStopFeedback() && rl.params.Get<bool>(
                        "return_to_getup_state_after_stop", true))
                {
                    // The GetUp state owns the only deployment-verified
                    // long-duration stand controller. A captured locomotion
                    // target can look stable for 0.15 s yet slowly pitch the
                    // loaded B2+Z1 forward. Hand control back only after the
                    // policy has decelerated into an upright four-foot phase.
                    rl.rl_init_done = false;
                    policy_running = false;
                    captured_support_hold_active = false;
                    rl.motion_heading_anchor_valid = false;
                    rl.obs.commands.assign(3, 0.0f);
                    policy_stop_wait_cycles = 0;
                    policy_stop_stable_cycles = 0;
                    std::cout << std::endl << LOGGER::INFO
                              << "Upright four-foot stop confirmed"
                              << ": base_h=" << rl.obs.base_height
                              << " tilt=" << body_tilt
                              << "; returning to the stable GetUp/stand state"
                              << std::endl;
                    rl.fsm.RequestStateChange(getdown_requested
                        ? "RLFSMStateGetDown" : "RLFSMStateGetUp");
                    return;
                }
                // Capture only a verified upright four-foot posture and keep
                // its last load-bearing policy target. Moving toward generic
                // nominal joint angles would move planted feet and generate a
                // horizontal contact impulse.
                rl.rl_init_done = false;
                policy_running = false;
                rl.motion_heading_anchor_valid = false;
                motion_start_hold_cycles = 0;
                gait_starter_prepare_cycles = 0;
                rl.obs.commands.assign(3, 0.0f);
                // Convert the measured actuator torque at the stable capture
                // instant into an equivalent static PD position error. This
                // preserves supporting force continuously without retaining
                // the low-speed policy frame's forward stepping target.
                const auto fixed_kp =
                    DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
                const auto hold_kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
                // Preserve the supporting torque on the first stopped frame,
                // then move to the same deterministic stand target that is
                // stable before locomotion. Holding the captured gait phase
                // forever lets the asymmetric foot geometry slowly pitch the
                // base forward even though all feet touched at capture time.
                stop_settle_start_q = fsm_state->motor_state.q;
                const auto stand_target = DeploymentStandPosition(rl);
                standby_hold_q = stand_target;
                const int leg_indices[] = {
                    0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12};
                for (const int index : leg_indices)
                {
                    stop_settle_start_q[index] =
                        fsm_state->motor_state.q[index] +
                        (fsm_state->motor_state.tau_est[index] +
                         hold_kd[index] *
                             fsm_state->motor_state.dq[index]) /
                            fixed_kp[index];
                }
                // Preserve the arm/TCP endpoint target. MuJoCo's Z1 uses
                // implicit position actuators and does not need this torque
                // conversion.
                const int arm_indices[] = {8, 13, 14, 15, 16, 17};
                for (const int index : arm_indices)
                {
                    standby_hold_q[index] =
                        fsm_command->motor_command.q[index];
                    stop_settle_start_q[index] = standby_hold_q[index];
                }
                stop_settle_cycles = kStopSettleHoldCycles;
                captured_support_hold_active = true;
                filtered_leg_q = standby_hold_q;
                filtered_arm_q = standby_hold_q;
                latest_policy_q = standby_hold_q;
                latest_policy_dq.assign(standby_hold_q.size(), 0.0f);
                leg_offset_scale = 1.0f;
                std::vector<float> discarded_output;
                while (rl.output_dof_pos_queue.try_pop(discarded_output)) {}
                while (rl.output_dof_vel_queue.try_pop(discarded_output)) {}
                while (rl.output_dof_tau_queue.try_pop(discarded_output)) {}
                std::cout << std::endl << LOGGER::INFO
                          << "Upright four-foot stop posture captured"
                          << ": base_h=" << support_height
                          << " tilt=" << body_tilt
                          << " feet=[" << StopContactsString() << "]"
                          << " nonfoot_contact="
                          << (rl.RequiresRealStopFeedback() ? "unknown" :
                              (rl.obs.undesired_ground_contact ? "1" : "0"))
                          << " body_v=" << body_planar_speed
                          << " gyro=" << body_angular_speed
                          << " max_leg_dq=" << max_leg_speed
                          << "; blending the load-bearing target to the stable stand pose"
                          << std::endl;
                policy_stop_wait_cycles = 0;
                policy_stop_stable_cycles = 0;
            }
            else if (wait_timed_out)
            {
                // Never turn an invalid knee/body contact into the permanent
                // hold pose merely because a timer expired. Keep the policy
                // active and report exactly which support condition is absent.
                std::cout << std::endl << LOGGER::WARNING
                          << (rl.RequiresRealStopFeedback()
                              ? "[Real stop gate] Stop blocked; no forced lowering"
                              : "Still searching for an upright four-foot stop posture")
                          << ": base_h=" << support_height
                          << " tilt=" << body_tilt
                          << " feet=[" << StopContactsString() << "] nonfoot_contact="
                          << (rl.RequiresRealStopFeedback() ? "unknown" :
                              (rl.obs.undesired_ground_contact ? "1" : "0"))
                          << " body_v=" << body_planar_speed
                          << " gyro=" << body_angular_speed
                          << " max_leg_dq=" << max_leg_speed
                          << " pending_B=" << getdown_requested
                          << " experimental_feedback_valid=" << va15_trial::FeedbackValid(fsm_state->real_stop_feedback)
                          << " unavailable=" << fsm_state->real_stop_feedback.unavailable_reason
                          << std::endl;
                policy_stop_wait_cycles = 0;
            }
        }
        else if (motion_requested)
        {
            policy_stop_wait_cycles = 0;
            policy_stop_stable_cycles = 0;
        }

        constexpr float kOffsetReleaseStep = 1.0f / 100.0f; // 0.5 s at 200 Hz
        constexpr float kOffsetReturnStep = 1.0f / 300.0f;  // 1.5 s at 200 Hz
        if (policy_running && motion_start_hold_cycles == 0)
        {
            leg_offset_scale = std::max(
                0.0f, leg_offset_scale - kOffsetReleaseStep);
        }
        else
        {
            leg_offset_scale = std::min(
                1.0f, leg_offset_scale + kOffsetReturnStep);
        }

        ++diagnostic_control_cycles;
        if (diagnostic_control_cycles % 200 == 0)
        {
            const auto projected_gravity = QuatRotateInverse(fsm_state->imu.quaternion, {0.0f, 0.0f, -1.0f});
            const bool real = rl.RequiresRealStopFeedback();
            const float unknown = std::numeric_limits<float>::quiet_NaN();
            std::cout << std::endl << LOGGER::INFO << std::fixed << std::setprecision(2)
                      << "RL Controller [" << rl.config_name << "] "
                      << "x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw
                      << " policy_x:" << (rl.obs.commands.empty() ? 0.0f : rl.obs.commands[0])
                      << " policy_y:" << (rl.obs.commands.size() > 1 ? rl.obs.commands[1] : 0.0f)
                      << " policy_yaw:" << (rl.obs.commands.size() > 2 ? rl.obs.commands[2] : 0.0f)
                      << " body_vx:" << (real ? unknown : rl.obs.lin_vel.empty() ? 0.0f : rl.obs.lin_vel[0])
                      << " body_vy:" << (real ? unknown : rl.obs.lin_vel.size() > 1 ? rl.obs.lin_vel[1] : 0.0f)
                      << " slip_deg:"
                      << (real ? unknown : rl.obs.lin_vel.size() > 1 &&
                                  std::fabs(rl.obs.lin_vel[0]) > 0.03f
                              ? std::atan2(
                                    rl.obs.lin_vel[1], rl.obs.lin_vel[0]) *
                                    57.2957795f
                              : 0.0f)
                      << " base_h:" << (real ? (fsm_state->real_stop_feedback.height_valid
                          ? fsm_state->real_stop_feedback.base_height : unknown) : rl.obs.base_height)
                      << " feet:[" << StopContactsString()
                      << "] nonfoot_contact:"
                      << (real ? "unknown" : (rl.obs.undesired_ground_contact ? "1" : "0"))
                      << " experimental_support_valid:" << va15_trial::FeedbackValid(fsm_state->real_stop_feedback)
                      << " real_planar_speed:" << (fsm_state->real_stop_feedback.velocity_valid
                          ? fsm_state->real_stop_feedback.planar_speed : unknown)
                      << " stance_offset:" << leg_offset_scale
                      << " gyro_z:" << fsm_state->imu.gyroscope[2]
                      << " grav:[" << projected_gravity[0] << ", " << projected_gravity[1] << ", " << projected_gravity[2] << "]"
                      << " hip:[" << fsm_state->motor_state.q[0] << ", " << fsm_state->motor_state.q[1]
                      << ", " << fsm_state->motor_state.q[2] << ", " << fsm_state->motor_state.q[3] << "]"
                      << " thigh:[" << fsm_state->motor_state.q[4] << ", " << fsm_state->motor_state.q[5]
                      << ", " << fsm_state->motor_state.q[6] << ", " << fsm_state->motor_state.q[7] << "]"
                      << " calf:[" << fsm_state->motor_state.q[9] << ", " << fsm_state->motor_state.q[10]
                      << ", " << fsm_state->motor_state.q[11] << ", " << fsm_state->motor_state.q[12] << "]"
                      << " arm:[" << fsm_state->motor_state.q[8] << ", " << fsm_state->motor_state.q[13]
                      << ", " << fsm_state->motor_state.q[14] << ", " << fsm_state->motor_state.q[15]
                      << ", " << fsm_state->motor_state.q[16] << ", " << fsm_state->motor_state.q[17] << "]"
                      << " target_hip:[" << fsm_command->motor_command.q[0] << ", "
                      << fsm_command->motor_command.q[1] << ", "
                      << fsm_command->motor_command.q[2] << ", "
                      << fsm_command->motor_command.q[3] << "]"
                      << " target_thigh:[" << fsm_command->motor_command.q[4] << ", "
                      << fsm_command->motor_command.q[5] << ", "
                      << fsm_command->motor_command.q[6] << ", "
                      << fsm_command->motor_command.q[7] << "]"
                      << " target_calf:[" << fsm_command->motor_command.q[9] << ", "
                      << fsm_command->motor_command.q[10] << ", "
                      << fsm_command->motor_command.q[11] << ", "
                      << fsm_command->motor_command.q[12] << "]"
                      << " target_arm:[" << fsm_command->motor_command.q[8] << ", "
                      << fsm_command->motor_command.q[13] << ", "
                      << fsm_command->motor_command.q[14] << ", "
                      << fsm_command->motor_command.q[15] << ", "
                      << fsm_command->motor_command.q[16] << ", "
                      << fsm_command->motor_command.q[17] << "]"
                      << std::defaultfloat << std::endl;
        }
        ApplyPolicyControl();

        if(startup_join_for_motion && policy_running)
        {
            const auto previous_stage=startup_handoff.stage;
            const auto result=startup_handoff.Step(*fsm_state,*fsm_command,previous_applied_command,
                policy_frame_received_this_cycle,va15_trial::HealthyFeedback(fsm_state->real_stop_feedback),
                RealStopSettled(),trial.Stopping()||getdown_requested||!operator_motion_requested,
                rl.params.Get<float>("dt"),
                rl.params.Get<std::vector<float>>("leg_target_safety_lower"),
                rl.params.Get<std::vector<float>>("leg_target_safety_upper"),
                rl.params.Get<std::vector<float>>("torque_limits"));
            front_handoff_cycles=rear_handoff_cycles=startup_handoff.Active()
                ? static_cast<int>(std::ceil(std::max(0.,startup_handoff.release_seconds-startup_handoff.elapsed)/
                    rl.params.Get<float>("dt"))) : 0;
            if(startup_handoff.stage!=previous_stage)
                std::cout<<LOGGER::INFO<<"[Startup handoff] "<<startup_handoff.reason
                    <<" wait_s="<<startup_handoff.waited<<" requested_pd_join_error="<<startup_handoff.initial_pd_error
                    <<"; NOT hardware-qualified"<<std::endl;
            if(result==VA15StartupHandoff::Result::Held)return;
            if(result==VA15StartupHandoff::Result::Refused || result==VA15StartupHandoff::Result::Cancelled)
            {
                trial.Stop(startup_handoff.reason);
                rl.control.x=rl.control.y=rl.control.yaw=0.f;
                if(!startup_handoff.ever_applied)
                {
                    rl.rl_init_done=policy_running=false;
                    rl.obs.commands.assign(3,0.f);
                }
                // Helper returned the last applied frame. Moving refusals
                // still require the existing supported-stop/lowering gate.
                return;
            }
        }

        // Hold the torque-equivalent static target with extra damping. It has
        // the same actuator torque as the captured frame on its first cycle,
        // but no longer contains a future gait-step position target.
        if (!operator_motion_requested && !policy_running &&
            captured_support_hold_active &&
            standby_hold_q.size() ==
                static_cast<std::size_t>(
                    rl.params.Get<int>("num_of_dofs")))
        {
            const auto fixed_kp =
                DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
            const auto fixed_kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
            float stand_blend = 1.0f;
            if (stop_settle_cycles > 0)
            {
                const float progress =
                    1.0f - static_cast<float>(stop_settle_cycles) /
                               static_cast<float>(kStopSettleHoldCycles);
                stand_blend =
                    progress * progress * (3.0f - 2.0f * progress);
                --stop_settle_cycles;
            }
            const int leg_indices[] = {
                0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12};
            for (const int index : leg_indices)
            {
                fsm_command->motor_command.q[index] =
                    (1.0f - stand_blend) * stop_settle_start_q[index] +
                    stand_blend * standby_hold_q[index];
                fsm_command->motor_command.dq[index] = 0.0f;
                fsm_command->motor_command.kp[index] = fixed_kp[index];
                fsm_command->motor_command.kd[index] = fixed_kd[index];
                fsm_command->motor_command.tau[index] = 0.0f;
            }
        }

        // Preserve the policy's full frame-to-frame gait motion. Only remove
        // the positional discontinuity between the last starter target and
        // the first learned target by adding a decaying joint-space offset.
        if (!startup_join_for_motion && motion_requested &&
            (front_handoff_cycles > 0 || rear_handoff_cycles > 0))
        {
            if (!gait_handoff_offset_initialized &&
                policy_frame_received_this_cycle)
            {
                gait_handoff_offset_q.assign(
                    fsm_command->motor_command.q.size(), 0.0f);
                const int leg_indices[] = {
                    0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12};
                for (const int index : leg_indices)
                {
                    gait_handoff_offset_q[index] =
                        rear_handoff_start_q[index] -
                        fsm_command->motor_command.q[index];
                }
                gait_handoff_offset_initialized = true;
            }
        }

        if (!startup_join_for_motion && motion_requested && front_handoff_cycles > 0 &&
            gait_handoff_offset_initialized)
        {
            const float progress =
                1.0f - static_cast<float>(front_handoff_cycles) /
                           static_cast<float>(kFrontHandoffCycles);
            const float remaining =
                1.0f - progress * progress * (3.0f - 2.0f * progress);
            const int front_indices[] = {0, 1, 4, 5, 9, 10};
            for (const int index : front_indices)
            {
                fsm_command->motor_command.q[index] +=
                    remaining * gait_handoff_offset_q[index];
                fsm_command->motor_command.kd[index] =
                    remaining * 8.0f +
                    (1.0f - remaining) *
                        fsm_command->motor_command.kd[index];
            }
            --front_handoff_cycles;
            if (front_handoff_cycles == 0)
            {
                std::cout << std::endl << LOGGER::INFO
                          << "Front-leg gait handoff offset released"
                          << std::endl;
            }
        }

        // The gait starter is needed to put the policy into its learned
        // locomotion phase, but releasing both rear legs in one control frame
        // causes a short support collapse. Let the front legs enter the learned
        // gait immediately while releasing only the rear thigh/calf targets
        // over a short, smooth interval.
        if (!startup_join_for_motion && motion_requested && rear_handoff_cycles > 0 &&
            gait_handoff_offset_initialized)
        {
            const float progress =
                1.0f - static_cast<float>(rear_handoff_cycles) /
                           static_cast<float>(kRearHandoffCycles);
            const float smooth_progress =
                progress * progress * (3.0f - 2.0f * progress);
            const float remaining = 1.0f - smooth_progress;
            const int rear_support_indices[] = {2, 3, 6, 7, 11, 12};
            for (const int index : rear_support_indices)
            {
                fsm_command->motor_command.q[index] +=
                    remaining * gait_handoff_offset_q[index];
                fsm_command->motor_command.kd[index] =
                    remaining * 8.0f +
                    smooth_progress * fsm_command->motor_command.kd[index];
            }
            --rear_handoff_cycles;
            if (rear_handoff_cycles == 0)
            {
                std::cout << std::endl << LOGGER::INFO
                          << "Rear-leg gait handoff offset released"
                          << std::endl;
            }
        }

        if (motion_start_hold_cycles > 0)
        {
            --motion_start_hold_cycles;
        }

        // The legs are already blended once by leg_offset_scale above. A
        // second multi-second leg blend produces a half-stance/half-gait
        // command and destroys the learned trot timing. Keep the slower,
        // independent handoff only for the arm.
        if (motion_requested && motion_start_hold_cycles == 0 &&
            arm_blend_cycles > 0)
        {
            for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
            {
                const bool is_arm = (i == 8 || i >= 13);
                if (!is_arm)
                {
                    continue;
                }
                const float blend =
                    1.0f - static_cast<float>(arm_blend_cycles) /
                               static_cast<float>(kArmBlendCycles);
                fsm_command->motor_command.q[i] =
                    (1.0f - blend) * standby_hold_q[i] +
                    blend * fsm_command->motor_command.q[i];
            }
            if (arm_blend_cycles > 0)
            {
                --arm_blend_cycles;
                if (arm_blend_cycles == 0)
                {
                    std::cout << std::endl << LOGGER::INFO
                              << "Policy arm handoff completed" << std::endl;
                }
            }
        }

        if (rl.params.Get<bool>("enable_leg_target_slew_filter", false))
        {
            // Optional hardware safeguard. It is disabled for Sim2Sim because
            // the policy was trained without a target slew filter.
            float command_magnitude = 0.0f;
            for (const float command : rl.obs.commands)
            {
                command_magnitude =
                    std::max(command_magnitude, std::fabs(command));
            }
            command_magnitude = std::min(command_magnitude, 1.0f);
            constexpr float kControlDt = 0.005f;
            const float max_leg_target_rate =
                2.0f + 6.0f * command_magnitude;
            const float max_leg_target_step =
                max_leg_target_rate * kControlDt;
            const int leg_filter_indices[] = {
                0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12};
            for (const int index : leg_filter_indices)
            {
                const float target_delta = std::clamp(
                    fsm_command->motor_command.q[index] - filtered_leg_q[index],
                    -max_leg_target_step,
                    max_leg_target_step);
                filtered_leg_q[index] += target_delta;
                fsm_command->motor_command.q[index] = filtered_leg_q[index];
            }
        }

        // Filter the six Z1 targets separately so frame-to-frame policy
        // variation does not excite the arm.
        if (rl.params.Get<bool>("enable_arm_target_filter", false))
        {
            const float arm_target_filter_alpha = std::clamp(
                rl.params.Get<float>("arm_target_filter_alpha", 0.02f),
                0.0f,
                1.0f);
            const int arm_indices[] = {8, 13, 14, 15, 16, 17};
            for (const int index : arm_indices)
            {
                filtered_arm_q[index] +=
                    arm_target_filter_alpha *
                    (fsm_command->motor_command.q[index] - filtered_arm_q[index]);
                fsm_command->motor_command.q[index] = filtered_arm_q[index];
            }
        }

        SyncAppliedActionObservation(!policy_running);

        const int leg_indices[] = {0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12};
        float max_leg_error = 0.0f;
        float max_leg_dq = 0.0f;
        float max_leg_tau = 0.0f;
        float min_leg_kp = std::numeric_limits<float>::max();
        for (const int index : leg_indices)
        {
            const float target_step = std::fabs(
                fsm_command->motor_command.q[index] - previous_leg_target_q[index]);
            const float actual_step = std::fabs(
                fsm_state->motor_state.q[index] - previous_leg_actual_q[index]);
            leg_target_travel += target_step;
            leg_actual_travel += actual_step;
            per_joint_target_travel[index] += target_step;
            per_joint_actual_travel[index] += actual_step;
            previous_leg_target_q[index] = fsm_command->motor_command.q[index];
            previous_leg_actual_q[index] = fsm_state->motor_state.q[index];
            max_leg_error = std::max(
                max_leg_error,
                std::fabs(fsm_command->motor_command.q[index] -
                          fsm_state->motor_state.q[index]));
            max_leg_dq = std::max(
                max_leg_dq, std::fabs(fsm_state->motor_state.dq[index]));
            max_leg_tau = std::max(
                max_leg_tau, std::fabs(fsm_state->motor_state.tau_est[index]));
            min_leg_kp = std::min(
                min_leg_kp, fsm_command->motor_command.kp[index]);
        }
        if (rl.episode_length_buf > 0 &&
            rl.episode_length_buf % 200 == 0 &&
            rl.episode_length_buf != last_motion_diagnostic_episode)
        {
            last_motion_diagnostic_episode = rl.episode_length_buf;
            std::cout << std::endl << LOGGER::INFO << std::fixed
                      << std::setprecision(3)
                      << "Policy motion diagnostic: target_travel="
                      << leg_target_travel
                      << " actual_travel=" << leg_actual_travel
                      << " max_q_error=" << max_leg_error
                      << " max_dq=" << max_leg_dq
                      << " max_tau=" << max_leg_tau
                      << " min_kp=" << min_leg_kp
                      << std::defaultfloat << std::endl;
            std::cout << LOGGER::INFO << std::fixed << std::setprecision(3)
                      << "Per-leg target/actual travel: "
                      << "FL=" << (per_joint_target_travel[0] + per_joint_target_travel[4] + per_joint_target_travel[9])
                      << "/" << (per_joint_actual_travel[0] + per_joint_actual_travel[4] + per_joint_actual_travel[9])
                      << " FR=" << (per_joint_target_travel[1] + per_joint_target_travel[5] + per_joint_target_travel[10])
                      << "/" << (per_joint_actual_travel[1] + per_joint_actual_travel[5] + per_joint_actual_travel[10])
                      << " RL=" << (per_joint_target_travel[2] + per_joint_target_travel[6] + per_joint_target_travel[11])
                      << "/" << (per_joint_actual_travel[2] + per_joint_actual_travel[6] + per_joint_actual_travel[11])
                      << " RR=" << (per_joint_target_travel[3] + per_joint_target_travel[7] + per_joint_target_travel[12])
                      << "/" << (per_joint_actual_travel[3] + per_joint_actual_travel[7] + per_joint_actual_travel[12])
                      << std::defaultfloat << std::endl;
            leg_target_travel = 0.0f;
            leg_actual_travel = 0.0f;
            std::fill(per_joint_target_travel.begin(), per_joint_target_travel.end(), 0.0f);
            std::fill(per_joint_actual_travel.begin(), per_joint_actual_travel.end(), 0.0f);
        }
    }

    void ApplyPolicyControl()
    {
        policy_frame_received_this_cycle = false;
        std::vector<float> output_dof_pos;
        std::vector<float> output_dof_vel;
        std::vector<float> output_dof_tau;
        // Keep only the newest policy frame. The control loop runs at 200 Hz
        // while inference runs at 50 Hz, so the same raw frame must be reused
        // between inferences instead of repeatedly blending an already-blended
        // command back toward the standby pose.
        while (rl.output_dof_pos_queue.try_pop(output_dof_pos))
        {
            if (output_dof_pos.size() == latest_policy_q.size())
            {
                latest_policy_q = output_dof_pos;
                policy_frame_received_this_cycle = true;
            }
        }
        while (rl.output_dof_vel_queue.try_pop(output_dof_vel))
        {
            if (output_dof_vel.size() == latest_policy_dq.size())
            {
                latest_policy_dq = output_dof_vel;
            }
        }
        // This FSM uses position/velocity commands and intentionally does not
        // apply the feed-forward torque produced by ComputeOutput().  Drain the
        // matching queue nevertheless; otherwise every 50 Hz inference leaves
        // one unused frame behind and memory grows during long deployments.
        while (rl.output_dof_tau_queue.try_pop(output_dof_tau)) {}

        const auto rl_kp = rl.params.Get<std::vector<float>>("rl_kp");
        const auto rl_kd = rl.params.Get<std::vector<float>>("rl_kd");
        const auto fixed_kp =
            DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
        const auto fixed_kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
        const auto default_dof_pos =
            rl.params.Get<std::vector<float>>("default_dof_pos");
        const float rf_rl_action_scale =
            rl.params.Get<float>("rf_rl_action_scale", 1.0f);
        float command_magnitude = 0.0f;
        for (const float command : rl.obs.commands)
        {
            command_magnitude =
                std::max(command_magnitude, std::fabs(command));
        }
        const float linear_speed = rl.obs.commands.size() >= 2
            ? std::hypot(rl.obs.commands[0], rl.obs.commands[1])
            : 0.0f;
        const float diagonal_scale_start =
            rl.params.Get<float>("rf_rl_action_scale_start", 1.0f);
        const float diagonal_scale_full = std::max(
            diagonal_scale_start + 1.0e-3f,
            rl.params.Get<float>("rf_rl_action_scale_full", 1.0f));
        const float diagonal_scale_blend = std::clamp(
            (linear_speed - diagonal_scale_start) /
                (diagonal_scale_full - diagonal_scale_start),
            0.0f, 1.0f);
        const float effective_rf_rl_action_scale =
            1.0f - diagonal_scale_blend *
                (1.0f - rf_rl_action_scale);
        const bool scale_high_swing_diagonal =
            effective_rf_rl_action_scale < 0.999f;
        const float rear_left_action_scale =
            rl.params.Get<float>("rear_left_action_scale", 1.0f);
        const float rear_left_scale_start =
            rl.params.Get<float>("rear_left_action_scale_start", 1.0f);
        const float rear_left_scale_full = std::max(
            rear_left_scale_start + 1.0e-3f,
            rl.params.Get<float>("rear_left_action_scale_full", 1.0f));
        const float rear_left_scale_blend = std::clamp(
            (linear_speed - rear_left_scale_start) /
                (rear_left_scale_full - rear_left_scale_start),
            0.0f, 1.0f);
        const float effective_rear_left_action_scale =
            1.0f - rear_left_scale_blend *
                (1.0f - rear_left_action_scale);
        const float damping_start =
            rl.params.Get<float>("high_speed_damping_start", 1.0f);
        const float damping_full = std::max(
            damping_start + 1.0e-3f,
            rl.params.Get<float>("high_speed_damping_full", 1.0f));
        const float damping_blend = std::clamp(
            (linear_speed - damping_start) /
                (damping_full - damping_start),
            0.0f, 1.0f);
        const float high_speed_kd_add =
            rl.params.Get<float>("high_speed_leg_kd_add", 0.0f) *
            damping_blend;
        const float high_speed_rf_rl_kd_add =
            rl.params.Get<float>("high_speed_rf_rl_kd_add", 0.0f) *
            damping_blend;

        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            // Hold a deterministic B2 stance at zero speed. As locomotion is
            // requested, release the complete learned whole-body targets.
            fsm_command->motor_command.q[i] =
                leg_offset_scale * standby_hold_q[i] +
                (1.0f - leg_offset_scale) * latest_policy_q[i];
            fsm_command->motor_command.dq[i] = latest_policy_dq[i];
            fsm_command->motor_command.kp[i] =
                leg_offset_scale * fixed_kp[i] +
                (1.0f - leg_offset_scale) * rl_kp[i];
            fsm_command->motor_command.kd[i] =
                leg_offset_scale * fixed_kd[i] +
                (1.0f - leg_offset_scale) * rl_kd[i];
            const bool is_arm = (i == 8 || i >= 13);
            if (!is_arm)
            {
                fsm_command->motor_command.kd[i] += high_speed_kd_add;
                const bool is_rf_rl_joint =
                    i == 1 || i == 2 || i == 5 || i == 6 ||
                    i == 10 || i == 11;
                if (is_rf_rl_joint)
                {
                    fsm_command->motor_command.kd[i] +=
                        high_speed_rf_rl_kd_add;
                }
            }
            fsm_command->motor_command.tau[i] = 0.0f;
        }

        if (scale_high_swing_diagonal)
        {
            // RF + RL is the over-active diagonal in MuJoCo. Reduce only its
            // learned excursion around the training default; the other
            // diagonal and the whole-body gait phase remain untouched.
            const int rf_rl_indices[] = {1, 5, 10, 2, 6, 11};
            for (const int index : rf_rl_indices)
            {
                fsm_command->motor_command.q[index] =
                    default_dof_pos[index] +
                    effective_rf_rl_action_scale *
                        (fsm_command->motor_command.q[index] -
                         default_dof_pos[index]);
            }
        }

        if (effective_rear_left_action_scale < 0.999f)
        {
            // Long-horizon deployment diagnostics isolate the excess motion
            // to RL, not the complete RF+RL trot diagonal. Correct only the
            // three RL joints and leave both front legs unchanged.
            const int rear_left_indices[] = {2, 6, 11};
            for (const int index : rear_left_indices)
            {
                fsm_command->motor_command.q[index] =
                    default_dof_pos[index] +
                    effective_rear_left_action_scale *
                        (fsm_command->motor_command.q[index] -
                         default_dof_pos[index]);
            }
        }

        // A rare long-horizon policy outlier can request a negative thigh
        // angle and a knee below -2.2 rad for one frame. It is still finite,
        // so generic NaN/torque protection cannot catch it, but the resulting
        // contact impulse is perceived as a violent twitch. Clamp only the
        // final leg position target to the verified trot envelope; arm targets
        // and normal gait phase remain untouched.
        if (rl.params.Get<bool>("enable_leg_target_safety_envelope", false))
        {
            const auto target_lower = rl.params.Get<std::vector<float>>(
                "leg_target_safety_lower");
            const auto target_upper = rl.params.Get<std::vector<float>>(
                "leg_target_safety_upper");
            if (target_lower.size() == fsm_command->motor_command.q.size() &&
                target_upper.size() == fsm_command->motor_command.q.size())
            {
                const int leg_indices[] = {
                    0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12};
                for (const int index : leg_indices)
                {
                    fsm_command->motor_command.q[index] = std::clamp(
                        fsm_command->motor_command.q[index],
                        target_lower[index], target_upper[index]);
                }
            }
        }
    }

    void ApplyGaitStarter()
    {
        const float control_dt = rl.params.Get<float>("dt");
        const float duration = rl.motion_loader->GetDuration();
        const float policy_warmup =
            rl.params.Get<float>("gait_starter_policy_warmup");
        rl.motion_loader->Update(std::min(gait_starter_time, duration));

        const auto reference_q = rl.motion_loader->GetJointPos();
        const auto reference_dq = rl.motion_loader->GetJointVel();
        auto supported_reference_q = reference_q;
        const auto rl_kp = rl.params.Get<std::vector<float>>("rl_kp");
        const auto rl_kd = rl.params.Get<std::vector<float>>("rl_kd");
        const auto fixed_kp =
            DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
        const auto fixed_kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
        if (reference_q.size() !=
            static_cast<std::size_t>(rl.params.Get<int>("num_of_dofs")))
        {
            throw std::runtime_error(
                "Forward gait starter joint count does not match policy DOFs");
        }

        // The recorded AMP starter contains a deep rear-leg crouch. Replaying
        // that pose literally makes both rear legs yield before the policy is
        // ready. Keep the original front-leg motion and timing, but retain a
        // minimum amount of rear-leg support during preparation and replay.
        const float rear_thigh_min =
            rl.params.Get<float>("gait_starter_rear_thigh_min");
        const float rear_calf_min =
            rl.params.Get<float>("gait_starter_rear_calf_min");
        supported_reference_q[6] =
            std::max(supported_reference_q[6], rear_thigh_min);
        supported_reference_q[7] =
            std::max(supported_reference_q[7], rear_thigh_min);
        supported_reference_q[11] =
            std::max(supported_reference_q[11], rear_calf_min);
        supported_reference_q[12] =
            std::max(supported_reference_q[12], rear_calf_min);

        if (gait_starter_symmetric_start_q.empty())
        {
            gait_starter_symmetric_start_q = supported_reference_q;
            const float front_hip_magnitude =
                0.5f * (std::fabs(supported_reference_q[0]) +
                        std::fabs(supported_reference_q[1]));
            gait_starter_symmetric_start_q[0] = front_hip_magnitude;
            gait_starter_symmetric_start_q[1] = -front_hip_magnitude;
            const float front_thigh_average =
                0.5f * (supported_reference_q[4] +
                        supported_reference_q[5]);
            gait_starter_symmetric_start_q[4] = front_thigh_average;
            gait_starter_symmetric_start_q[5] = front_thigh_average;
            const float front_calf_average =
                0.5f * (supported_reference_q[9] +
                        supported_reference_q[10]);
            gait_starter_symmetric_start_q[9] = front_calf_average;
            gait_starter_symmetric_start_q[10] = front_calf_average;
        }

        const auto projected_gravity = QuatRotateInverse(
            fsm_state->imu.quaternion, {0.0f, 0.0f, -1.0f});
        const float current_tilt = std::sqrt(
            projected_gravity[0] * projected_gravity[0] +
            projected_gravity[1] * projected_gravity[1]);
        float current_max_leg_dq = 0.0f;
        const int diagnostic_leg_indices[] = {
            0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12};
        for (const int index : diagnostic_leg_indices)
        {
            current_max_leg_dq = std::max(
                current_max_leg_dq,
                std::fabs(fsm_state->motor_state.dq[index]));
        }

        if (gait_starter_prepare_cycles > 0)
        {
            gait_prepare_max_tilt =
                std::max(gait_prepare_max_tilt, current_tilt);
            gait_prepare_max_leg_dq =
                std::max(gait_prepare_max_leg_dq, current_max_leg_dq);
            const float progress =
                1.0f - static_cast<float>(gait_starter_prepare_cycles) /
                           static_cast<float>(kGaitStarterPrepareCycles);
            for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
            {
                const bool is_arm = (i == 8 || i >= 13);
                fsm_command->motor_command.q[i] = is_arm
                    ? standby_hold_q[i]
                    : (1.0f - progress) * gait_starter_prepare_start_q[i] +
                          progress * gait_starter_symmetric_start_q[i];
                fsm_command->motor_command.dq[i] = 0.0f;
                fsm_command->motor_command.kp[i] = fixed_kp[i];
                fsm_command->motor_command.kd[i] = fixed_kd[i];
                fsm_command->motor_command.tau[i] = 0.0f;
            }
            SyncAppliedActionObservation(true);
            --gait_starter_prepare_cycles;
            if (gait_starter_prepare_cycles == 0)
            {
                previous_leg_target_q = fsm_command->motor_command.q;
                previous_leg_actual_q = fsm_state->motor_state.q;
                std::cout << std::endl << LOGGER::INFO
                          << "Gait starter preparation completed; max_tilt="
                          << gait_prepare_max_tilt
                          << ", max_leg_dq=" << gait_prepare_max_leg_dq
                          << "; beginning AMP reference replay"
                          << std::endl;
            }
            return;
        }

        gait_replay_max_tilt =
            std::max(gait_replay_max_tilt, current_tilt);
        gait_replay_max_leg_dq =
            std::max(gait_replay_max_leg_dq, current_max_leg_dq);

        const float replay_blend_duration =
            rl.params.Get<float>("gait_starter_replay_blend_duration", 0.5f);
        const float replay_blend_progress = std::min(
            1.0f,
            gait_starter_time / std::max(replay_blend_duration, 1.0e-3f));
        const float replay_smooth_progress =
            replay_blend_progress * replay_blend_progress *
            (3.0f - 2.0f * replay_blend_progress);
        auto physical_reference_q = supported_reference_q;
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            const bool is_arm = (i == 8 || i >= 13);
            if (!is_arm)
            {
                physical_reference_q[i] =
                    (1.0f - replay_smooth_progress) *
                        gait_starter_symmetric_start_q[i] +
                    replay_smooth_progress * supported_reference_q[i];
            }
        }

        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            const bool is_arm = (i == 8 || i >= 13);
            const bool is_rear_support =
                (i == 6 || i == 7 || i == 11 || i == 12);
            fsm_command->motor_command.q[i] =
                is_arm ? standby_hold_q[i] : physical_reference_q[i];
            // The AMP reset used by training initializes q and dq together.
            // Feed the recorded finite-difference velocity to the PD target so
            // the physical joints do not lag the moving replay by 0.3--0.5 rad
            // before policy handoff. Arm joints remain at their static hold.
            fsm_command->motor_command.dq[i] = is_arm
                ? 0.0f
                : reference_dq[i];
            fsm_command->motor_command.kp[i] =
                is_arm
                    ? fixed_kp[i]
                    : (is_rear_support ? fixed_kp[i] : rl_kp[i]);
            fsm_command->motor_command.kd[i] =
                (is_arm || is_rear_support)
                    ? fixed_kd[i] : rl_kd[i];
            fsm_command->motor_command.tau[i] = 0.0f;
        }

        // Before inference starts, seed last_action from the physical replay
        // target. Once policy warmup is active, RunModel owns last_action and
        // stores the actor's raw preceding output, exactly like Isaac Lab's
        // mdp.last_action. Overwriting it here at 200 Hz with an inverse-PD
        // pseudo action gives the actor an impossible action history and makes
        // both the legs and Z1 oscillate at handoff.
        if (!gait_starter_policy_warmup)
        {
            SyncActionObservationFromJointTargets(physical_reference_q);
        }

        if (!gait_starter_policy_warmup &&
            gait_starter_time + control_dt >=
                std::max(0.0f, duration - policy_warmup))
        {
            rl.obs.commands =
                rl.params.Get<std::vector<float>>(
                    "gait_starter_policy_command");
            std::vector<float> discarded_output;
            while (rl.output_dof_pos_queue.try_pop(discarded_output)) {}
            while (rl.output_dof_vel_queue.try_pop(discarded_output)) {}
            while (rl.output_dof_tau_queue.try_pop(discarded_output)) {}
            {
                std::lock_guard<std::mutex> lock(rl.model_mutex);
                // Isaac's AMP reference reset changes the physical state but
                // resets both FilteredJointPositionAction and last_action to
                // zero. Do the same at this exact moving-frame handoff.
                rl.obs.actions.assign(
                    rl.params.Get<int>("num_of_dofs"), 0.0f);
                rl.reset_policy_action_state = true;
            }
            rl.rl_init_done = true;
            policy_running = true;
            gait_starter_policy_warmup = true;
            std::cout << std::endl << LOGGER::INFO
                      << "Gait starter reached a moving AMP state; policy inference warming up"
                      << std::endl;
        }

        gait_starter_time += control_dt;
        if (gait_starter_time >= duration)
        {
            gait_starter_active = false;
            leg_offset_scale = 0.0f;
            front_handoff_cycles = kFrontHandoffCycles;
            rear_handoff_cycles = kRearHandoffCycles;
            rear_handoff_start_q = fsm_command->motor_command.q;
            gait_handoff_offset_initialized = false;
            motion_start_hold_cycles = 0;
            arm_blend_cycles = kArmBlendCycles;
            previous_leg_target_q = fsm_command->motor_command.q;
            previous_leg_actual_q = fsm_state->motor_state.q;
            std::cout << std::endl << LOGGER::INFO
                      << "AMP gait starter completed; replay_max_tilt="
                      << gait_replay_max_tilt
                      << ", replay_max_leg_dq=" << gait_replay_max_leg_dq
                      << "; policy now has full leg control"
                      << std::endl;
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
        rl.motion_heading_anchor_valid = false;
        policy_started = false;
        startup_handoff.Reset();
        startup_join_for_motion=false;
        policy_running = false;
        gait_starter_active = false;
        gait_starter_policy_warmup = false;
        gait_starter_prepare_cycles = 0;
        captured_support_hold_active = false;
        stop_settle_cycles = 0;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
            rl.control.current_gamepad == Input::Gamepad::B)
        {
            if (!getdown_requested)
            {
                std::cout << std::endl << LOGGER::NOTE
                          << "Key 9 requested: remove the welding gun now; "
                          << "waiting for a smooth upright four-foot stop "
                          << "before get-down." << std::endl;
            }
            getdown_requested = true;
            rl.control.x = 0.0f;
            rl.control.y = 0.0f;
            rl.control.yaw = 0.0f;
        }
        if (getdown_requested)
        {
            if (controlled_stop.Active())
                return controlled_stop.Ready() ? "RLFSMStateGetDown" : state_name_;
            const bool policy_stopped = !policy_running && !gait_starter_active;
            const bool handoff_finished = handoff_hold_cycles <= 0;
            const bool support_settled =
                !captured_support_hold_active || stop_settle_cycles <= 0;
            if (policy_stopped && handoff_finished && support_settled)
            {
                return "RLFSMStateGetDown";
            }
            return state_name_;
        }
        // Never use A/Y to jump around the moving-stop gate.
        if (rl.RequiresRealStopFeedback() && (policy_running || gait_starter_active ||
            (captured_support_hold_active && stop_settle_cycles > 0)))
            return state_name_;
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }
        return state_name_;
    }
};

} // namespace b2_z1_no_gun_fsm

class B2Z1NoGunFSMFactory : public FSMFactory
{
public:
    B2Z1NoGunFSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<b2_z1_no_gun_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<b2_z1_no_gun_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<b2_z1_no_gun_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<b2_z1_no_gun_fsm::RLFSMStateRLLocomotion>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "b2_z1_no_gun"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(B2Z1NoGunFSMFactory, "RLFSMStatePassive")

#endif // B2_Z1_NO_GUN_FSM_HPP
