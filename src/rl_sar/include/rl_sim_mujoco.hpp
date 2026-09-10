/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SIM_HPP
#define RL_SIM_HPP

// #define PLOT
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_all.hpp"

#include <csignal>
#include <vector>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <memory>
#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>

#include <mujoco/mujoco.h>
#include "joystick.hh"
#include "mujoco_utils.hpp"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class Button
{
public:
    Button() {}

    void update(bool state)
    {
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
};

class RL_Sim : public RL
{
public:
    RL_Sim(int argc, char **argv);
    ~RL_Sim();

    std::unique_ptr<mj::Simulate> sim;
    static RL_Sim* instance;

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_joystick;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;
    std::shared_ptr<LoopFunc> loop_camera;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // mujoco
    mjData *mj_data;
    mjModel *mj_model;
    std::string scene_name;
    std::atomic<bool> camera_follow_configured{false};
    int camera_follow_wait_cycles = 0;
    void ConfigureCameraFollow();

    // MuJoCo-only trajectory follower.  The loaded path is anchored in the
    // world when trajectory mode starts, then converted each cycle into the
    // velocity and EE-target observations expected by the existing policy.
    struct TrajectoryPoint
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double s = 0.0;
    };
    bool trajectory_mode_available = false;
    bool trajectory_control_active = false;
    bool trajectory_end_hold_active = false;
    bool trajectory_loaded = false;
    bool trajectory_world_initialized = false;
    std::vector<TrajectoryPoint> trajectory_local_points;
    std::vector<TrajectoryPoint> trajectory_world_points;
    std::size_t trajectory_nearest_segment = 0;
    TrajectoryPoint trajectory_current_target;
    int trajectory_base_body_id = -1;
    int trajectory_floor_geom_id = -1;
    std::vector<int> trajectory_ground_geom_ids;
    std::array<int, 4> trajectory_foot_geom_ids{{-1, -1, -1, -1}};
    int trajectory_target_visual_mocap_id = -1;
    int trajectory_ee_site_id = -1;
    std::array<int, 6> trajectory_arm_dof_ids{{-1, -1, -1, -1, -1, -1}};
    double trajectory_control_start_time = -1.0;
    std::vector<int> trajectory_path_visual_mocap_ids;
    // Vision runs in a separate process. The controller only exchanges small
    // atomic text files, so camera rendering/inference can never block the
    // 50 Hz policy loop.
    bool vision_trajectory_enabled = false;
    bool vision_trajectory_fresh = false;
    bool vision_rear_target_active = false;
    bool vision_policy_target_initialized = false;
    std::array<double, 3> vision_policy_target_b{{-0.775, 0.0, -0.35}};
    std::array<double, 3> vision_policy_target_start_b{{-0.775, 0.0, -0.35}};
    bool vision_heading_command_initialized = false;
    double vision_heading_command = 0.0;
    bool vision_lateral_command_initialized = false;
    double vision_lateral_command = 0.0;
    std::string vision_exchange_dir;
    double vision_state_export_period = 0.06;
    double vision_trajectory_stale_timeout = 0.40;
    double vision_last_state_export_time = -1.0;
    double trajectory_last_tracking_diagnostic_time = -1.0;
    double trajectory_path_lateral_error = 0.0;
    double trajectory_path_heading_error = 0.0;
    std::uint64_t vision_state_sequence = 0;
    bool vision_trajectory_mtime_valid = false;
    std::filesystem::file_time_type vision_trajectory_mtime;
    std::chrono::steady_clock::time_point vision_last_trajectory_received;
    bool LoadTrajectory();
    void ExportVisionState();
    void IngestVisionTrajectory();
    bool StartTrajectoryControl();
    void StopTrajectoryControl(
        const std::string& reason, bool hold_at_endpoint = false);
    void UpdateTrajectoryControl();
    void UpdateTrajectoryVisualization();
    TrajectoryPoint SampleTrajectory(double arc_length) const;

    // joystick
    std::unique_ptr<Joystick> sys_js;
    JoystickEvent sys_js_event;

    Button sys_js_button[20];
    int sys_js_axis[10] = {0};
    bool sys_js_active = false;
    float axis_deadzone = 0.05f;
    int sys_js_max_value = (1 << (16 - 1));
    void SetupSysJoystick(const std::string& device, int bits);
    void GetSysJoystick();

    // others
    // Keep actor history and deployment-only actuator smoothing separate.
    // Historical direct-action checkpoints observe raw_policy_actions as
    // mdp.last_action while filtered_policy_actions drives MuJoCo/hardware.
    std::vector<float> raw_policy_actions;
    std::vector<float> filtered_policy_actions;
    std::string gazebo_model_name;
    std::map<std::string, float> joint_positions;
    std::map<std::string, float> joint_velocities;
    std::map<std::string, float> joint_efforts;
    void StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names);
};

#endif // RL_SIM_HPP
