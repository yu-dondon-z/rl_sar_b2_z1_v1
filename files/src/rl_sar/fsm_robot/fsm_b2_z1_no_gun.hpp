/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef B2_Z1_NO_GUN_FSM_HPP
#define B2_Z1_NO_GUN_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <utility>

namespace b2_z1_no_gun_fsm
{

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
    int post_getup_hold_cycles = 0;
    int settle_window_cycles = 0;
    std::vector<float> settle_window_start_q;
    bool locomotion_requested = false;
    bool post_getup_hold_reported = false;
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
        post_getup_hold_cycles = 0;
        settle_window_cycles = 0;
        settle_window_start_q.clear();
        locomotion_requested = false;
        post_getup_hold_reported = false;
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
    }

    void Run() override
    {
        const auto stand_pos =
            rl.params.Get<std::vector<float>>("default_dof_pos") +
            rl.params.Get<std::vector<float>>("sim_dof_pos_offset");
        if(stand_from_passive)
        {

            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 1.0f, "Pre Getting up", true))
            {
                return;
            }
            if (Interpolate(percent_getup, pre_running_pos, stand_pos, 2.0f, "Getting up", true))
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

        // Interpolate stops writing once complete. Explicitly hold the final
        // stance with extra damping while waiting for a stable policy handoff.
        const auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.q[i] = stand_pos[i];
            fsm_command->motor_command.dq[i] = 0.0f;
            fsm_command->motor_command.kp[i] = kp[i];
            fsm_command->motor_command.kd[i] = 8.0f;
            fsm_command->motor_command.tau[i] = 0.0f;
        }

        ++post_getup_hold_cycles;
        float max_leg_velocity = 0.0f;
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            if (!IsArmJoint(i))
            {
                max_leg_velocity = std::max(
                    max_leg_velocity, std::fabs(fsm_state->motor_state.dq[i]));
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

                if (max_leg_position_delta <= kStableLegPositionDelta)
                {
                    post_getup_hold_reported = true;
                    std::cout << std::endl << LOGGER::INFO
                              << "Post-getup settling completed; max_leg_dq="
                              << max_leg_velocity
                              << ", max_leg_q_delta=" << max_leg_position_delta
                              << ", policy handoff enabled" << std::endl;
                }
                else
                {
                    std::cout << std::endl << LOGGER::INFO
                              << "Post-getup settling: max_leg_dq="
                              << max_leg_velocity
                              << ", max_leg_q_delta=" << max_leg_position_delta
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
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
        for (int index : {8, 13, 14, 15, 16, 17})
        {
            fsm_command->motor_command.q[index] = fsm_state->motor_state.q[index];
            fsm_command->motor_command.dq[index] = 0.0f;
            fsm_command->motor_command.kp[index] = 0.0f;
            fsm_command->motor_command.kd[index] = 1.0f;
            fsm_command->motor_command.tau[index] = 0.0f;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
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
    int stop_settle_cycles = 0;
    std::vector<float> stop_settle_start_q;
    std::vector<float> standby_hold_q;
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
    int handoff_hold_cycles = 0;
    int arm_blend_cycles = 0;
    static constexpr int kPolicyWarmupCycles = 400; // 2.0 s at 200 Hz
    static constexpr int kArmBlendCycles = 1000;    // 5.0 s at 200 Hz
    static constexpr int kGaitStarterPrepareCycles = 300; // 1.5 s at 200 Hz
    static constexpr int kFrontHandoffCycles = 50; // 0.25 s at 200 Hz
    static constexpr int kRearHandoffCycles = 120; // 0.6 s at 200 Hz
    static constexpr int kStopSettleHoldCycles = 40; // 0.2 s at 200 Hz
    static constexpr int kStopStandTransitionCycles = 800; // 4.0 s at 200 Hz
    std::vector<float> previous_leg_target_q;
    std::vector<float> previous_leg_actual_q;
    std::vector<float> per_joint_target_travel;
    std::vector<float> per_joint_actual_travel;
    float leg_target_travel = 0.0f;
    float leg_actual_travel = 0.0f;
    unsigned long long diagnostic_control_cycles = 0;
    unsigned long long last_motion_diagnostic_episode = ~0ULL;

    void SyncActionObservationFromJointTargets(
        const std::vector<float> &joint_targets)
    {
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
        stop_settle_cycles = 0;
        stop_settle_start_q.clear();
        rl.rl_init_done = false;
        rl.control.x = 0.0f;
        rl.control.y = 0.0f;
        rl.control.yaw = 0.0f;
        rl.control.current_keyboard = Input::Keyboard::None;
        standby_hold_q = fsm_state->motor_state.q;
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
        rl.config_name = "robot_lab";
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
            const auto stand_target =
                rl.params.Get<std::vector<float>>("default_dof_pos") +
                rl.params.Get<std::vector<float>>("sim_dof_pos_offset");
            for (int index = 0;
                 index < rl.params.Get<int>("num_of_dofs"); ++index)
            {
                standby_hold_q[index] = stand_target[index];
                filtered_leg_q[index] = stand_target[index];
                filtered_arm_q[index] = stand_target[index];
                latest_policy_q[index] = stand_target[index];
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
        if (handoff_hold_cycles > 0)
        {
            const auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
            for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
            {
                fsm_command->motor_command.q[i] = standby_hold_q[i];
                fsm_command->motor_command.dq[i] = 0.0f;
                fsm_command->motor_command.kp[i] = kp[i];
                fsm_command->motor_command.kd[i] = 8.0f;
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

        // Blend from the exact GetUp stance to the raw learned gait once
        // motion is requested. Release quickly enough that small real-robot
        // commands are not trapped in a half-stance/half-gait compromise.
        const bool operator_motion_requested =
            std::fabs(rl.control.x) > 0.03f ||
            std::fabs(rl.control.y) > 0.03f ||
            std::fabs(rl.control.yaw) > 0.03f;
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
                rl.obs.commands.assign(3, 0.0f);
                standby_hold_q = fsm_state->motor_state.q;
                stop_settle_start_q = standby_hold_q;
                stop_settle_cycles =
                    kStopSettleHoldCycles + kStopStandTransitionCycles;
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
                          << "Gait starter cancelled; holding support phase before smooth stand transition"
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
            stop_settle_cycles = 0;
            rl.obs.commands = {
                rl.control.x, rl.control.y, rl.control.yaw};
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
            const bool forward_only =
                rl.control.x > 0.03f &&
                std::fabs(rl.control.y) <= 0.03f &&
                std::fabs(rl.control.yaw) <= 0.03f;
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
                motion_start_hold_cycles = 20; // 0.1 s at 200 Hz
                rl.rl_init_done = true;
                policy_running = true;
                std::cout << std::endl << LOGGER::INFO
                          << "Motion command received; policy history reset and inference started"
                          << std::endl;
            }
        }
        else if (!motion_requested && policy_running)
        {
            // Stop producing disconnected actions while returning to the
            // zero-speed hold. Capture the posture that is physically present
            // at the stop instant instead of pulling every leg toward the
            // nominal stand pose while the robot is in an arbitrary gait
            // phase. The latter destroys the current support polygon and can
            // tip an otherwise stable robot during deceleration.
            rl.rl_init_done = false;
            policy_running = false;
            motion_start_hold_cycles = 0;
            gait_starter_prepare_cycles = 0;
            rl.obs.commands.assign(3, 0.0f);
            standby_hold_q = fsm_state->motor_state.q;
            stop_settle_start_q = standby_hold_q;
            stop_settle_cycles =
                kStopSettleHoldCycles + kStopStandTransitionCycles;
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
                      << "Zero command received; holding support phase before smooth stand transition"
                      << std::endl;
        }

        constexpr float kOffsetReleaseStep = 1.0f / 100.0f; // 0.5 s at 200 Hz
        constexpr float kOffsetReturnStep = 1.0f / 300.0f;  // 1.5 s at 200 Hz
        if (motion_requested && motion_start_hold_cycles == 0)
        {
            leg_offset_scale = std::max(
                0.0f, leg_offset_scale - kOffsetReleaseStep);
        }
        else
        {
            leg_offset_scale = std::min(
                1.0f, leg_offset_scale + kOffsetReturnStep);
        }

        if (!motion_requested && stop_settle_cycles > 0)
        {
            const int total_stop_cycles =
                kStopSettleHoldCycles + kStopStandTransitionCycles;
            const int elapsed_stop_cycles =
                total_stop_cycles - stop_settle_cycles;
            if (elapsed_stop_cycles >= kStopSettleHoldCycles)
            {
                const float transition_progress = std::min(
                    1.0f,
                    static_cast<float>(
                        elapsed_stop_cycles - kStopSettleHoldCycles + 1) /
                        static_cast<float>(kStopStandTransitionCycles));
                const float smooth_progress =
                    transition_progress * transition_progress *
                    (3.0f - 2.0f * transition_progress);
                const auto stand_target =
                    rl.params.Get<std::vector<float>>("default_dof_pos") +
                    rl.params.Get<std::vector<float>>("sim_dof_pos_offset");
                for (int i = 0;
                     i < rl.params.Get<int>("num_of_dofs"); ++i)
                {
                    const bool is_arm = (i == 8 || i >= 13);
                    if (!is_arm)
                    {
                        standby_hold_q[i] =
                            (1.0f - smooth_progress) *
                                stop_settle_start_q[i] +
                            smooth_progress * stand_target[i];
                    }
                }
            }
            --stop_settle_cycles;
            if (stop_settle_cycles == 0)
            {
                std::cout << std::endl << LOGGER::INFO
                          << "Zero-speed smooth stand transition completed"
                          << std::endl;
            }
        }

        ++diagnostic_control_cycles;
        if (diagnostic_control_cycles % 200 == 0)
        {
            const auto projected_gravity = QuatRotateInverse(fsm_state->imu.quaternion, {0.0f, 0.0f, -1.0f});
            std::cout << std::endl << LOGGER::INFO << std::fixed << std::setprecision(2)
                      << "RL Controller [" << rl.config_name << "] "
                      << "x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw
                      << " policy_x:" << (rl.obs.commands.empty() ? 0.0f : rl.obs.commands[0])
                      << " policy_y:" << (rl.obs.commands.size() > 1 ? rl.obs.commands[1] : 0.0f)
                      << " policy_yaw:" << (rl.obs.commands.size() > 2 ? rl.obs.commands[2] : 0.0f)
                      << " body_vx:" << (rl.obs.lin_vel.empty() ? 0.0f : rl.obs.lin_vel[0])
                      << " body_vy:" << (rl.obs.lin_vel.size() > 1 ? rl.obs.lin_vel[1] : 0.0f)
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

        // Preserve the policy's full frame-to-frame gait motion. Only remove
        // the positional discontinuity between the last starter target and
        // the first learned target by adding a decaying joint-space offset.
        if (motion_requested &&
            (front_handoff_cycles > 0 || rear_handoff_cycles > 0))
        {
            if (!gait_handoff_offset_initialized)
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

        if (motion_requested && front_handoff_cycles > 0)
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
        if (motion_requested && rear_handoff_cycles > 0)
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
        std::vector<float> output_dof_pos;
        std::vector<float> output_dof_vel;
        // Keep only the newest policy frame. The control loop runs at 200 Hz
        // while inference runs at 50 Hz, so the same raw frame must be reused
        // between inferences instead of repeatedly blending an already-blended
        // command back toward the standby pose.
        while (rl.output_dof_pos_queue.try_pop(output_dof_pos))
        {
            if (output_dof_pos.size() == latest_policy_q.size())
            {
                latest_policy_q = output_dof_pos;
            }
        }
        while (rl.output_dof_vel_queue.try_pop(output_dof_vel))
        {
            if (output_dof_vel.size() == latest_policy_dq.size())
            {
                latest_policy_dq = output_dof_vel;
            }
        }

        const auto rl_kp = rl.params.Get<std::vector<float>>("rl_kp");
        const auto rl_kd = rl.params.Get<std::vector<float>>("rl_kd");
        const auto fixed_kp =
            rl.params.Get<std::vector<float>>("fixed_kp");
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
                leg_offset_scale * 8.0f +
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
    }

    void ApplyGaitStarter()
    {
        const float control_dt = rl.params.Get<float>("dt");
        const float duration = rl.motion_loader->GetDuration();
        const float policy_warmup =
            rl.params.Get<float>("gait_starter_policy_warmup");
        rl.motion_loader->Update(std::min(gait_starter_time, duration));

        const auto reference_q = rl.motion_loader->GetJointPos();
        auto supported_reference_q = reference_q;
        const auto rl_kp = rl.params.Get<std::vector<float>>("rl_kp");
        const auto rl_kd = rl.params.Get<std::vector<float>>("rl_kd");
        const auto fixed_kp =
            rl.params.Get<std::vector<float>>("fixed_kp");
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
                fsm_command->motor_command.kd[i] = 8.0f;
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
            fsm_command->motor_command.dq[i] = 0.0f;
            fsm_command->motor_command.kp[i] =
                is_arm
                    ? fixed_kp[i]
                    : (is_rear_support ? fixed_kp[i] : rl_kp[i]);
            fsm_command->motor_command.kd[i] =
                (is_arm || is_rear_support) ? 8.0f : rl_kd[i];
            fsm_command->motor_command.tau[i] = 0.0f;
        }

        // Seed the policy with the complete recorded gait phase even though
        // the physical rear legs retain extra support during the starter.
        // Otherwise RF+RL begins the handoff from an artificial frozen phase
        // and produces an unnecessarily high first swing.
        SyncActionObservationFromJointTargets(reference_q);

        if (!gait_starter_policy_warmup &&
            gait_starter_time >= std::max(0.0f, duration - policy_warmup))
        {
            rl.obs.commands =
                rl.params.Get<std::vector<float>>(
                    "gait_starter_policy_command");
            std::vector<float> discarded_output;
            while (rl.output_dof_pos_queue.try_pop(discarded_output)) {}
            while (rl.output_dof_vel_queue.try_pop(discarded_output)) {}
            while (rl.output_dof_tau_queue.try_pop(discarded_output)) {}
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
        policy_started = false;
        policy_running = false;
        gait_starter_active = false;
        gait_starter_policy_warmup = false;
        gait_starter_prepare_cycles = 0;
        stop_settle_cycles = 0;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
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
