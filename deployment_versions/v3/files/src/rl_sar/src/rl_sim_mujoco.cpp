/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim_mujoco.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

RL_Sim* RL_Sim::instance = nullptr;

RL_Sim::RL_Sim(int argc, char **argv)
{
    // Set static instance pointer early for signal handler
    instance = this;

    if (argc < 3)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " robot_name scene_name" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }
    else
    {
        this->robot_name = argv[1];
        this->scene_name = argv[2];
    }

    this->ang_vel_axis = "body";

    // now launch mujoco
    std::cout << LOGGER::INFO << "[MuJoCo] Launching..." << std::endl;

    // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
    if (rosetta_error_msg)
    {
        DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
        std::exit(1);
    }
#endif

    // print version, check compatibility
    std::cout << LOGGER::INFO << "[MuJoCo] Version: " << mj_versionString() << std::endl;
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    // scan for libraries in the plugin directory to load additional plugins
    scanPluginLibraries();

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    // simulate object encapsulates the UI
    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    std::string filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/../rl_sar_zoo/" + this->robot_name + "_description/mjcf/" + this->scene_name + ".xml";

    // start physics thread
    std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename.c_str());
    physicsthreadhandle.detach();

    while (1)
    {
        if (d)
        {
            std::cout << LOGGER::INFO << "[MuJoCo] Data prepared" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    this->mj_model = m;
    this->mj_data = d;
    this->SetupSysJoystick("/dev/input/js0", 16); // 16 bits joystick

    // read params from yaml
    this->ReadYaml(this->robot_name, "base.yaml");

    this->trajectory_mode_available =
        this->robot_name == "b2_z1_no_gun" &&
        this->params.Get<bool>("trajectory_mode_enabled", false);
    if (this->trajectory_mode_available)
    {
        this->vision_trajectory_enabled =
            this->scene_name == "scene_vision" &&
            this->params.Get<bool>("vision_trajectory_enabled", false);
        this->vision_exchange_dir = this->params.Get<std::string>(
            "vision_exchange_dir", "/tmp/b2_z1_mujoco_vision");
        this->vision_state_export_period = std::max(
            0.02, this->params.Get<double>(
                "vision_state_export_period", 0.06));
        this->vision_trajectory_stale_timeout = std::max(
            0.10, this->params.Get<double>(
                "vision_trajectory_stale_timeout", 0.40));
        if (this->vision_trajectory_enabled)
        {
            std::error_code error;
            std::filesystem::create_directories(
                this->vision_exchange_dir, error);
            if (error)
            {
                std::cout << LOGGER::ERROR
                          << "[Vision] Cannot create exchange directory: "
                          << this->vision_exchange_dir << ": "
                          << error.message() << std::endl;
                this->vision_trajectory_enabled = false;
            }
        }
        this->trajectory_base_body_id =
            mj_name2id(this->mj_model, mjOBJ_BODY, "base_link");
        this->trajectory_floor_geom_id =
            mj_name2id(this->mj_model, mjOBJ_GEOM, "floor");
        this->trajectory_ground_geom_ids.clear();
        if (this->trajectory_floor_geom_id >= 0)
        {
            this->trajectory_ground_geom_ids.push_back(
                this->trajectory_floor_geom_id);
        }
        // A physical crack is built from separate left/right road slabs.
        // Treat every slab as ground for four-foot support and non-foot
        // contact diagnostics, while retaining the lower crack floor too.
        for (int geom_id = 0; geom_id < this->mj_model->ngeom; ++geom_id)
        {
            const char* geom_name = mj_id2name(
                this->mj_model, mjOBJ_GEOM, geom_id);
            if (geom_name != nullptr &&
                (std::strncmp(geom_name, "road_left_", 10) == 0 ||
                 std::strncmp(geom_name, "road_right_", 11) == 0))
            {
                this->trajectory_ground_geom_ids.push_back(geom_id);
            }
        }
        const char* foot_geom_names[] = {
            "FL_foot_collision", "FR_foot_collision",
            "RL_foot_collision", "RR_foot_collision"};
        for (std::size_t index = 0;
             index < this->trajectory_foot_geom_ids.size(); ++index)
        {
            this->trajectory_foot_geom_ids[index] = mj_name2id(
                this->mj_model, mjOBJ_GEOM, foot_geom_names[index]);
        }
        const int target_body_id = mj_name2id(
            this->mj_model, mjOBJ_BODY, "trajectory_target_visual");
        if (target_body_id >= 0)
        {
            this->trajectory_target_visual_mocap_id =
                this->mj_model->body_mocapid[target_body_id];
        }
        this->trajectory_ee_site_id = mj_name2id(
            this->mj_model, mjOBJ_SITE, "ee_site");
        const char* arm_joint_names[] = {
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"};
        for (std::size_t index = 0;
             index < this->trajectory_arm_dof_ids.size(); ++index)
        {
            const int joint_id = mj_name2id(
                this->mj_model, mjOBJ_JOINT, arm_joint_names[index]);
            if (joint_id >= 0)
            {
                this->trajectory_arm_dof_ids[index] =
                    this->mj_model->jnt_dofadr[joint_id];
            }
        }
        const int visual_point_count = this->params.Get<int>(
            "trajectory_visual_point_count", 24);
        for (int index = 0; index < visual_point_count; ++index)
        {
            std::ostringstream name;
            name << "trajectory_path_visual_" << std::setw(2)
                 << std::setfill('0') << index;
            const int body_id = mj_name2id(
                this->mj_model, mjOBJ_BODY, name.str().c_str());
            if (body_id >= 0)
            {
                const int mocap_id = this->mj_model->body_mocapid[body_id];
                if (mocap_id >= 0)
                {
                    this->trajectory_path_visual_mocap_ids.push_back(mocap_id);
                }
            }
        }
        if (!this->vision_trajectory_enabled)
        {
            this->LoadTrajectory();
        }
        std::cout << LOGGER::INFO << "[Trajectory] "
                  << (this->vision_trajectory_enabled
                          ? "Live RHACrackNet trajectory mode"
                          : "Static CSV trajectory mode")
                  << ". Press T to start/stop path following; "
                  << "Space is an emergency stop." << std::endl;
    }

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // RenderLoop resets the camera during startup. Configure tracking from a
    // short auxiliary loop after the renderer has consumed the loaded model.
    if (this->params.Get<bool>("mujoco_follow_camera", false))
    {
        this->loop_camera = std::make_shared<LoopFunc>(
            "loop_camera", 0.1,
            std::bind(&RL_Sim::ConfigureCameraFollow, this));
        this->loop_camera->start();
    }

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

    // joystick
    this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Sim::GetSysJoystick, this));
    this->loop_joystick->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;

    // start simulation UI loop (blocking call)
    sim->RenderLoop();
}

RL_Sim::~RL_Sim()
{
    // Clear static instance pointer
    instance = nullptr;

    this->loop_keyboard->shutdown();
    this->loop_joystick->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    if (this->loop_camera)
    {
        this->loop_camera->shutdown();
    }
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::ConfigureCameraFollow()
{
    if (camera_follow_configured.load() || !sim || !mj_model || !mj_data)
    {
        return;
    }

    // Give RenderLoop time to reset its defaults and finish LoadOnRenderThread.
    if (++camera_follow_wait_cycles < 5)
    {
        return;
    }

    const std::string body_name = this->params.Get<std::string>(
        "mujoco_follow_camera_body", "base_link");
    const int body_id =
        mj_name2id(mj_model, mjOBJ_BODY, body_name.c_str());
    if (body_id < 0)
    {
        std::cout << LOGGER::WARNING
                  << "[MuJoCo] Follow camera body not found: "
                  << body_name << std::endl;
        camera_follow_configured.store(true);
        return;
    }

    std::lock_guard<mj::SimulateMutex> lock(sim->mtx);
    if (sim->loadrequest != 0)
    {
        return;
    }

    sim->cam.type = mjCAMERA_TRACKING;
    sim->cam.trackbodyid = body_id;
    sim->cam.fixedcamid = -1;
    sim->cam.distance = this->params.Get<float>(
        "mujoco_follow_camera_distance", 4.0f);
    sim->cam.azimuth = this->params.Get<float>(
        "mujoco_follow_camera_azimuth", 135.0f);
    sim->cam.elevation = this->params.Get<float>(
        "mujoco_follow_camera_elevation", -20.0f);
    const float height_offset = this->params.Get<float>(
        "mujoco_follow_camera_height", 0.15f);
    sim->cam.lookat[0] = mj_data->xpos[3 * body_id + 0];
    sim->cam.lookat[1] = mj_data->xpos[3 * body_id + 1];
    sim->cam.lookat[2] =
        mj_data->xpos[3 * body_id + 2] + height_offset;

    // Keep the built-in Rendering > Camera > Tracking selector functional.
    sim->pert.select = body_id;
    sim->camera = 1;
    camera_follow_configured.store(true);
    std::cout << LOGGER::INFO
              << "[MuJoCo] Tracking camera enabled for body '"
              << body_name << "'" << std::endl;
}

bool RL_Sim::LoadTrajectory()
{
    this->trajectory_local_points.clear();
    this->trajectory_world_points.clear();
    this->trajectory_loaded = false;
    this->trajectory_world_initialized = false;

    const std::string relative_path = this->params.Get<std::string>(
        "trajectory_file", "robot_lab/trajectories/test_centerline.csv");
    const std::string path = std::string(POLICY_DIR) + "/" +
        this->robot_name + "/" + relative_path;
    std::ifstream input(path);
    if (!input.is_open())
    {
        std::cout << LOGGER::ERROR
                  << "[Trajectory] Cannot open: " << path << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(input, line))
    {
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos)
        {
            line.erase(comment);
        }
        for (char& character : line)
        {
            if (character == ',' || character == ';' || character == '\t')
            {
                character = ' ';
            }
        }
        std::istringstream values(line);
        TrajectoryPoint point;
        if (!(values >> point.x >> point.y >> point.z))
        {
            continue;
        }
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z))
        {
            continue;
        }
        if (!this->trajectory_local_points.empty())
        {
            const auto& previous = this->trajectory_local_points.back();
            if (std::hypot(point.x - previous.x, point.y - previous.y) <
                1.0e-6)
            {
                continue;
            }
        }
        this->trajectory_local_points.push_back(point);
    }

    if (this->trajectory_local_points.size() < 2)
    {
        std::cout << LOGGER::ERROR
                  << "[Trajectory] At least two valid x,y,z points are required: "
                  << path << std::endl;
        return false;
    }

    double arc_length = 0.0;
    this->trajectory_local_points.front().s = 0.0;
    for (std::size_t index = 1;
         index < this->trajectory_local_points.size(); ++index)
    {
        const auto& previous = this->trajectory_local_points[index - 1];
        auto& point = this->trajectory_local_points[index];
        arc_length += std::hypot(
            point.x - previous.x, point.y - previous.y);
        point.s = arc_length;
    }
    this->trajectory_loaded = arc_length > 1.0e-3;
    std::cout << LOGGER::INFO << "[Trajectory] Loaded "
              << this->trajectory_local_points.size() << " points, length="
              << arc_length << " m from " << path << std::endl;
    return this->trajectory_loaded;
}

void RL_Sim::ExportVisionState()
{
    if (!this->vision_trajectory_enabled || !this->mj_model ||
        !this->mj_data || !this->simulation_running)
    {
        return;
    }
    if (this->vision_last_state_export_time >= 0.0 &&
        this->mj_data->time - this->vision_last_state_export_time <
            this->vision_state_export_period)
    {
        return;
    }

    const std::filesystem::path directory(this->vision_exchange_dir);
    const std::filesystem::path destination = directory / "latest_state.txt";
    const std::filesystem::path temporary = directory /
        (".latest_state." + std::to_string(::getpid()) + ".tmp");
    std::ofstream output(temporary, std::ios::trunc);
    if (!output.is_open())
    {
        return;
    }
    output << ++this->vision_state_sequence << ' '
           << std::setprecision(17) << this->mj_data->time << ' '
           << this->mj_model->nq << '\n';
    for (int index = 0; index < this->mj_model->nq; ++index)
    {
        output << std::setprecision(17) << this->mj_data->qpos[index]
               << (index + 1 == this->mj_model->nq ? '\n' : ' ');
    }
    output.close();
    if (!output)
    {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return;
    }
    std::error_code error;
    std::filesystem::rename(temporary, destination, error);
    if (!error)
    {
        this->vision_last_state_export_time = this->mj_data->time;
    }
    else
    {
        std::filesystem::remove(temporary, error);
    }
}

void RL_Sim::IngestVisionTrajectory()
{
    if (!this->vision_trajectory_enabled)
    {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    const std::filesystem::path path =
        std::filesystem::path(this->vision_exchange_dir) /
        "latest_trajectory.csv";
    std::error_code error;
    const auto modified = std::filesystem::last_write_time(path, error);
    if (!error && (!this->vision_trajectory_mtime_valid ||
                   modified != this->vision_trajectory_mtime))
    {
        std::ifstream input(path);
        std::vector<TrajectoryPoint> points;
        std::string line;
        while (std::getline(input, line))
        {
            const std::size_t comment = line.find('#');
            if (comment != std::string::npos)
            {
                line.erase(comment);
            }
            for (char& character : line)
            {
                if (character == ',' || character == ';' ||
                    character == '\t')
                {
                    character = ' ';
                }
            }
            std::istringstream values(line);
            TrajectoryPoint point;
            if (!(values >> point.x >> point.y >> point.z) ||
                !std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z))
            {
                continue;
            }
            if (!points.empty() &&
                std::hypot(point.x - points.back().x,
                           point.y - points.back().y) < 0.01)
            {
                continue;
            }
            points.push_back(point);
        }
        if (points.size() >= 3)
        {
            double arc_length = 0.0;
            points.front().s = 0.0;
            for (std::size_t index = 1; index < points.size(); ++index)
            {
                arc_length += std::hypot(
                    points[index].x - points[index - 1].x,
                    points[index].y - points[index - 1].y);
                points[index].s = arc_length;
            }
            if (arc_length >= 0.30)
            {
                this->trajectory_world_points = std::move(points);
                this->trajectory_nearest_segment = 0;
                this->trajectory_loaded = true;
                this->trajectory_world_initialized = true;
                this->vision_trajectory_fresh = true;
                this->vision_last_trajectory_received = now;
                this->vision_trajectory_mtime = modified;
                this->vision_trajectory_mtime_valid = true;
            }
        }
    }

    if (this->vision_trajectory_fresh)
    {
        const double age = std::chrono::duration<double>(
            now - this->vision_last_trajectory_received).count();
        if (age > this->vision_trajectory_stale_timeout)
        {
            this->vision_trajectory_fresh = false;
            std::cout << LOGGER::WARNING
                      << "[Vision] Trajectory is stale; commanding a smooth stop."
                      << std::endl;
        }
    }
}

bool RL_Sim::StartTrajectoryControl()
{
    if (!this->trajectory_mode_available || !this->mj_model ||
        !this->mj_data || this->trajectory_base_body_id < 0)
    {
        std::cout << LOGGER::WARNING
                  << "[Trajectory] This simulation does not provide trajectory mode."
                  << std::endl;
        return false;
    }
    if (!this->model)
    {
        std::cout << LOGGER::WARNING
                  << "[Trajectory] Enter locomotion mode with key 1 before pressing T."
                  << std::endl;
        return false;
    }
    if (this->vision_trajectory_enabled)
    {
        this->IngestVisionTrajectory();
        if (!this->vision_trajectory_fresh ||
            this->trajectory_world_points.size() < 3)
        {
            std::cout << LOGGER::WARNING
                      << "[Vision] No fresh validated crack trajectory yet. "
                      << "Wait for the vision window to show VALID, then press T."
                      << std::endl;
            return false;
        }
        this->trajectory_nearest_segment = 0;
        this->trajectory_control_active = true;
        this->trajectory_end_hold_active = false;
        const auto nominal_target = this->params.Get<std::vector<float>>(
            "target_commands");
        this->vision_policy_target_start_b = {
            nominal_target[0], nominal_target[1], nominal_target[2]};
        // T is a mid-episode controller switch, not an environment reset.
        // Immediately before T, the policy already observes nominal_target.
        // Starting from the measured TCP would make the observation jump by
        // the policy's steady-state tracking error (previously about 34 cm in
        // z), even though it makes the visual Cartesian error zero.  That
        // discontinuity couples into both the arm and legs and causes the
        // characteristic backward squat.  Preserve the pre-T policy command
        // for a genuinely bumpless transfer; live vision subsequently changes
        // only the filtered lateral component, matching training.
        this->vision_policy_target_b = this->vision_policy_target_start_b;
        this->vision_policy_target_initialized = true;
        this->vision_heading_command = 0.0;
        this->vision_heading_command_initialized = true;
        this->vision_lateral_command = 0.0;
        this->vision_lateral_command_initialized = true;
        this->trajectory_last_tracking_diagnostic_time = -1.0;
        this->trajectory_control_start_time = this->mj_data->time;
        this->UpdateTrajectoryControl();
        this->UpdateTrajectoryVisualization();
        std::cout << LOGGER::INFO
                  << "[Vision] Live crack following started." << std::endl;
        return true;
    }
    if (!this->trajectory_loaded && !this->LoadTrajectory())
    {
        return false;
    }

    const mjtNum* base_pos = this->mj_data->xpos +
        3 * this->trajectory_base_body_id;
    const mjtNum* base_rot = this->mj_data->xmat +
        9 * this->trajectory_base_body_id;
    double forward_x = base_rot[0];
    double forward_y = base_rot[3];
    const double forward_norm = std::hypot(forward_x, forward_y);
    if (forward_norm < 1.0e-8)
    {
        forward_x = 1.0;
        forward_y = 0.0;
    }
    else
    {
        forward_x /= forward_norm;
        forward_y /= forward_norm;
    }
    const double left_x = -forward_y;
    const double left_y = forward_x;
    const bool relative_to_start = this->params.Get<bool>(
        "trajectory_relative_to_start", true);

    this->trajectory_world_points.clear();
    this->trajectory_world_points.reserve(
        this->trajectory_local_points.size());
    for (const auto& local : this->trajectory_local_points)
    {
        TrajectoryPoint world;
        if (relative_to_start)
        {
            world.x = base_pos[0] + forward_x * local.x + left_x * local.y;
            world.y = base_pos[1] + forward_y * local.x + left_y * local.y;
            world.z = base_pos[2] + local.z;
        }
        else
        {
            world = local;
        }
        this->trajectory_world_points.push_back(world);
    }

    double arc_length = 0.0;
    this->trajectory_world_points.front().s = 0.0;
    for (std::size_t index = 1;
         index < this->trajectory_world_points.size(); ++index)
    {
        const auto& previous = this->trajectory_world_points[index - 1];
        auto& point = this->trajectory_world_points[index];
        arc_length += std::hypot(
            point.x - previous.x, point.y - previous.y);
        point.s = arc_length;
    }

    this->trajectory_nearest_segment = 0;
    this->trajectory_world_initialized = true;
    this->trajectory_control_active = true;
    this->trajectory_end_hold_active = false;
    this->trajectory_last_tracking_diagnostic_time = -1.0;
    this->trajectory_control_start_time = this->mj_data->time;
    this->UpdateTrajectoryControl();
    this->UpdateTrajectoryVisualization();
    std::cout << LOGGER::INFO
              << "[Trajectory] Started. Anchored path length=" << arc_length
              << " m" << std::endl;
    return true;
}

void RL_Sim::StopTrajectoryControl(
    const std::string& reason, bool hold_at_endpoint)
{
    if (!this->trajectory_control_active &&
        !this->trajectory_end_hold_active)
    {
        return;
    }
    this->trajectory_control_active = false;
    this->trajectory_end_hold_active = hold_at_endpoint;
    this->trajectory_control_start_time = -1.0;
    this->vision_rear_target_active = false;
    this->vision_policy_target_initialized = false;
    this->vision_heading_command_initialized = false;
    this->vision_heading_command = 0.0;
    this->vision_lateral_command_initialized = false;
    this->vision_lateral_command = 0.0;
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
    if (!hold_at_endpoint)
    {
        this->motion_heading_anchor_valid = false;
        this->ClearRuntimeTargetCommands();
        if (this->trajectory_target_visual_mocap_id >= 0)
        {
            mjtNum* position = this->mj_data->mocap_pos +
                3 * this->trajectory_target_visual_mocap_id;
            position[2] = -10.0;
        }
    }
    std::cout << LOGGER::INFO << "[Trajectory] Stopped: " << reason
              << (hold_at_endpoint ? "; standing hold enabled" : "")
              << std::endl;
}

RL_Sim::TrajectoryPoint RL_Sim::SampleTrajectory(double arc_length) const
{
    if (this->trajectory_world_points.empty())
    {
        return {};
    }
    arc_length = std::clamp(
        arc_length, 0.0, this->trajectory_world_points.back().s);
    const auto upper = std::lower_bound(
        this->trajectory_world_points.begin(),
        this->trajectory_world_points.end(), arc_length,
        [](const TrajectoryPoint& point, double value)
        {
            return point.s < value;
        });
    if (upper == this->trajectory_world_points.begin())
    {
        return *upper;
    }
    if (upper == this->trajectory_world_points.end())
    {
        return this->trajectory_world_points.back();
    }
    const auto& end = *upper;
    const auto& start = *(upper - 1);
    const double segment_length = std::max(end.s - start.s, 1.0e-9);
    const double blend = (arc_length - start.s) / segment_length;
    return {
        start.x + blend * (end.x - start.x),
        start.y + blend * (end.y - start.y),
        start.z + blend * (end.z - start.z),
        arc_length};
}

void RL_Sim::UpdateTrajectoryControl()
{
    if (!this->trajectory_control_active ||
        !this->trajectory_world_initialized ||
        this->trajectory_world_points.size() < 2)
    {
        return;
    }
    if (this->vision_trajectory_enabled &&
        !this->vision_trajectory_fresh)
    {
        // Do not automatically resume when the next camera frame happens to
        // become valid.  That used to create a second gait start without an
        // operator command and could topple the robot.  T must be pressed
        // again after the vision window is VALID.
        this->StopTrajectoryControl(
            "live crack trajectory lost; press T after vision recovers",
            false);
        return;
    }

    const mjtNum* base_pos = this->mj_data->xpos +
        3 * this->trajectory_base_body_id;
    const mjtNum* base_rot = this->mj_data->xmat +
        9 * this->trajectory_base_body_id;

    // Search only forward from the previous segment.  This prevents noisy
    // base motion from making the trajectory target jump backwards.
    double best_distance_squared = std::numeric_limits<double>::infinity();
    double nearest_arc_length =
        this->trajectory_world_points[this->trajectory_nearest_segment].s;
    std::size_t best_segment = this->trajectory_nearest_segment;
    const std::size_t search_end = this->vision_trajectory_enabled
        ? this->trajectory_world_points.size() - 1
        : std::min(
              this->trajectory_world_points.size() - 1,
              this->trajectory_nearest_segment +
                  static_cast<std::size_t>(80));
    for (std::size_t index = this->trajectory_nearest_segment;
         index < search_end; ++index)
    {
        const auto& start = this->trajectory_world_points[index];
        const auto& end = this->trajectory_world_points[index + 1];
        const double segment_x = end.x - start.x;
        const double segment_y = end.y - start.y;
        const double length_squared =
            segment_x * segment_x + segment_y * segment_y;
        if (length_squared < 1.0e-12)
        {
            continue;
        }
        const double projection = std::clamp(
            ((base_pos[0] - start.x) * segment_x +
             (base_pos[1] - start.y) * segment_y) / length_squared,
            0.0, 1.0);
        const double projected_x = start.x + projection * segment_x;
        const double projected_y = start.y + projection * segment_y;
        const double error_x = base_pos[0] - projected_x;
        const double error_y = base_pos[1] - projected_y;
        const double distance_squared =
            error_x * error_x + error_y * error_y;
        if (distance_squared < best_distance_squared)
        {
            best_distance_squared = distance_squared;
            best_segment = index;
            nearest_arc_length = start.s +
                projection * (end.s - start.s);
        }
    }
    this->trajectory_nearest_segment = best_segment;

    const double total_length = this->trajectory_world_points.back().s;
    const double remaining_length = total_length - nearest_arc_length;
    const double end_standoff = this->vision_trajectory_enabled
        ? 0.10
        : std::max(
              0.1,
              this->params.Get<double>("trajectory_end_standoff", 0.70));
    bool rear_target_found = false;
    if (this->vision_trajectory_enabled)
    {
        const double minimum_forward_path = std::max(
            0.25, this->params.Get<double>(
                "vision_minimum_forward_path", 0.65));
        if (remaining_length <= minimum_forward_path)
        {
            // The image boundary is not the physical end of the crack. Wait
            // for a new frame to extend the path instead of entering the
            // irreversible static-CSV endpoint hold.
            this->trajectory_current_target =
                this->trajectory_world_points.back();
            this->control.x = 0.0f;
            this->control.y = 0.0f;
            this->control.yaw = 0.0f;
            this->UpdateTrajectoryVisualization();
            return;
        }
    }
    else if (remaining_length <= end_standoff)
    {
        // Keep the final EE command and marker visible while the existing FSM
        // settles the legs into its fixed zero-speed stance. The explicit end
        // hold below also prevents a stale keyboard/gamepad command from
        // restarting motion after autonomous completion.
        this->trajectory_current_target =
            this->trajectory_world_points.back();
        this->UpdateTrajectoryVisualization();
        this->StopTrajectoryControl("end of path reached", true);
        return;
    }

    const double base_lookahead = std::max(
        0.1, this->params.Get<double>("trajectory_base_lookahead", 0.40));
    const double ee_lookahead = std::max(
        end_standoff,
        this->params.Get<double>("trajectory_ee_lookahead", 0.70));
    const TrajectoryPoint base_target = this->SampleTrajectory(
        nearest_arc_length + base_lookahead);
    const TrajectoryPoint path_forward_target = this->SampleTrajectory(
        nearest_arc_length + ee_lookahead);
    TrajectoryPoint ee_target = path_forward_target;

    double forward_x = base_rot[0];
    double forward_y = base_rot[3];
    const double forward_norm = std::hypot(forward_x, forward_y);
    if (forward_norm > 1.0e-8)
    {
        forward_x /= forward_norm;
        forward_y /= forward_norm;
    }
    else
    {
        forward_x = 1.0;
        forward_y = 0.0;
    }
    const double left_x = -forward_y;
    const double left_y = forward_x;
    const double lookahead_delta_x = base_target.x - base_pos[0];
    const double lookahead_delta_y = base_target.y - base_pos[1];
    const double lookahead_forward =
        lookahead_delta_x * forward_x + lookahead_delta_y * forward_y;
    const double lookahead_lateral =
        lookahead_delta_x * left_x + lookahead_delta_y * left_y;

    if (this->vision_trajectory_enabled)
    {
        // The front camera guides locomotion, while the rear-mounted Z1 tracks
        // the same world crack only after it has passed underneath B2. This
        // mirrors RearVisionTrajectoryCommand used by the training run.
        const double rear_min = this->params.Get<double>(
            "vision_rear_work_x_min", -1.0);
        const double rear_max = this->params.Get<double>(
            "vision_rear_work_x_max", -0.55);
        const double desired_rear_x = 0.5 * (rear_min + rear_max);
        double best_rear_error = std::numeric_limits<double>::infinity();
        for (const auto& point : this->trajectory_world_points)
        {
            const double delta_x = point.x - base_pos[0];
            const double delta_y = point.y - base_pos[1];
            const double point_x_b =
                delta_x * forward_x + delta_y * forward_y;
            if (point_x_b < rear_min || point_x_b > rear_max)
            {
                continue;
            }
            const double error = std::fabs(point_x_b - desired_rear_x);
            if (error < best_rear_error)
            {
                best_rear_error = error;
                ee_target = point;
                rear_target_found = true;
            }
        }
        if (!rear_target_found)
        {
            // Do not feed the current unloaded flange/tool location back as
            // a command.  At policy entry the long virtual TCP can still be
            // far below its trained workspace (about z=-0.69 m in the base
            // frame), which is an out-of-distribution whole-body command.
            // Hold the exact rear waiting pose used during training until a
            // cached crack point actually reaches the rear work envelope.
            const auto waiting_target = this->params.Get<std::vector<float>>(
                "target_commands");
            // waiting_target is expressed in the full instantaneous base
            // frame, exactly like RearVisionTrajectoryCommand in Isaac Lab.
            // Convert it to world coordinates with the complete base
            // rotation, not only its yaw component.
            ee_target = {
                base_pos[0] + base_rot[0] * waiting_target[0] +
                    base_rot[1] * waiting_target[1] +
                    base_rot[2] * waiting_target[2],
                base_pos[1] + base_rot[3] * waiting_target[0] +
                    base_rot[4] * waiting_target[1] +
                    base_rot[5] * waiting_target[2],
                base_pos[2] + base_rot[6] * waiting_target[0] +
                    base_rot[7] * waiting_target[1] +
                    base_rot[8] * waiting_target[2],
                0.0};
        }
        if (rear_target_found != this->vision_rear_target_active)
        {
            this->vision_rear_target_active = rear_target_found;
            std::cout << (rear_target_found ? LOGGER::INFO : LOGGER::WARNING)
                      << (rear_target_found
                              ? "[Vision] Rear TCP acquired a cached crack point."
                              : "[Vision] Rear TCP is holding its current pose until a crack point reaches the rear workspace.")
                      << std::endl;
        }
    }
    double direction_x = base_target.x - base_pos[0];
    double direction_y = base_target.y - base_pos[1];
    const double direction_norm = std::hypot(direction_x, direction_y);
    if (direction_norm > 1.0e-8)
    {
        direction_x /= direction_norm;
        direction_y /= direction_norm;
    }

    const double tangent_sample_distance = 0.10;
    const TrajectoryPoint tangent_start = this->SampleTrajectory(
        std::max(0.0, nearest_arc_length + ee_lookahead -
                          tangent_sample_distance));
    const TrajectoryPoint tangent_end = this->SampleTrajectory(
        nearest_arc_length + ee_lookahead + tangent_sample_distance);
    double tangent_x = tangent_end.x - tangent_start.x;
    double tangent_y = tangent_end.y - tangent_start.y;
    const double tangent_norm = std::hypot(tangent_x, tangent_y);
    if (tangent_norm > 1.0e-8)
    {
        tangent_x /= tangent_norm;
        tangent_y /= tangent_norm;
    }
    else
    {
        tangent_x = direction_x;
        tangent_y = direction_y;
    }
    if (this->vision_trajectory_enabled)
    {
        // Estimate heading from a metre-scale local chord around the base
        // look-ahead point. Using the complete visible path makes a distant
        // bend steer the robot before it reaches that bend; the old 0.2 m
        // chord, on the other hand, amplified pixel/depth noise.
        const double heading_chord = std::max(
            0.40, this->params.Get<double>(
                "vision_heading_chord_length", 1.00));
        const double heading_center = std::clamp(
            nearest_arc_length + base_lookahead,
            0.0, total_length);
        const TrajectoryPoint vision_start = this->SampleTrajectory(
            heading_center - 0.5 * heading_chord);
        const TrajectoryPoint vision_end = this->SampleTrajectory(
            heading_center + 0.5 * heading_chord);
        tangent_x = vision_end.x - vision_start.x;
        tangent_y = vision_end.y - vision_start.y;
        double vision_tangent_norm = std::hypot(tangent_x, tangent_y);
        if (vision_tangent_norm > 1.0e-8)
        {
            tangent_x /= vision_tangent_norm;
            tangent_y /= vision_tangent_norm;
            if (tangent_x * forward_x + tangent_y * forward_y < 0.0)
            {
                tangent_x = -tangent_x;
                tangent_y = -tangent_y;
            }
        }
        else
        {
            tangent_x = forward_x;
            tangent_y = forward_y;
        }
    }

    const double cruise_speed = std::max(
        0.0, this->params.Get<double>("trajectory_cruise_speed", 0.25));
    const double slowdown_distance = std::max(
        0.05, this->params.Get<double>("trajectory_slowdown_distance", 0.50));
    const double extra_remaining = remaining_length - end_standoff;
    const double speed_scale = std::clamp(
        extra_remaining / slowdown_distance, 0.0, 1.0);
    // Approach the endpoint slowly but do not let the command fall below the
    // FSM's motion-request threshold before the geometric endpoint condition
    // above has actually fired. This gives the legs time to dissipate forward
    // momentum without prematurely entering the standing hold.
    const double minimum_approach_speed = std::max(
        0.031,
        this->params.Get<double>(
            "trajectory_minimum_approach_speed", 0.04));
    const double requested_speed = std::min(
        cruise_speed,
        std::max(cruise_speed * speed_scale, minimum_approach_speed));
    const double trajectory_elapsed =
        this->trajectory_control_start_time >= 0.0
            ? std::max(0.0, this->mj_data->time -
                                this->trajectory_control_start_time)
            : 0.0;
    const double start_speed_ramp_time = std::max(
        0.1, this->params.Get<double>(
                 "trajectory_start_speed_ramp_time", 1.5));
    const double start_speed_blend = this->vision_trajectory_enabled
        ? std::clamp(trajectory_elapsed / start_speed_ramp_time, 0.0, 1.0)
        : 1.0;

    const auto command_limits = this->params.Get<std::vector<float>>(
        "policy_command_limits", {0.30f, 0.30f, 0.45f});
    this->control.x = std::clamp(
        static_cast<float>(requested_speed *
            (direction_x * forward_x + direction_y * forward_y)),
        -command_limits[0], command_limits[0]);
    this->control.y = std::clamp(
        static_cast<float>(requested_speed *
            (direction_x * left_x + direction_y * left_y)),
        -command_limits[1], command_limits[1]);
    const double tangent_heading_error = std::atan2(
        tangent_x * left_x + tangent_y * left_y,
        tangent_x * forward_x + tangent_y * forward_y);
    // Match RearVisionTrajectoryCommand used during training.  Tangent-only
    // steering can align B2 parallel to a crack while leaving it permanently
    // offset from the crack.  The look-ahead lateral term pulls the body
    // centre back onto the path without introducing an untrained vy command.
    double path_heading_error = tangent_heading_error;
    double cross_track_heading = 0.0;
    if (this->vision_trajectory_enabled)
    {
        const double cross_track_gain = std::max(
            0.0, this->params.Get<double>(
                "trajectory_cross_track_gain", 0.8));
        const double raw_cross_track_heading = std::atan2(
            cross_track_gain * lookahead_lateral,
            std::max(lookahead_forward, 0.10));
        // The deployed checkpoint is stable when it first receives the path
        // tangent, but a simultaneous full cross-track correction can upset
        // the first trot diagonal.  Bring this outer-loop term in only after
        // locomotion has settled, and keep it much slower than the learned
        // yaw envelope.  It still removes a persistent parallel offset.
        const double cross_track_limit = std::max(
            0.0, this->params.Get<double>(
                "trajectory_cross_track_heading_limit", 0.04));
        const double cross_track_ramp_time = std::max(
            0.1, this->params.Get<double>(
                "trajectory_cross_track_ramp_time", 6.0));
        const double cross_track_blend = std::clamp(
            trajectory_elapsed / cross_track_ramp_time, 0.0, 1.0);
        cross_track_heading = cross_track_blend * std::clamp(
            raw_cross_track_heading, -cross_track_limit,
            cross_track_limit);
        const double combined_heading =
            tangent_heading_error + cross_track_heading;
        path_heading_error = std::atan2(
            std::sin(combined_heading), std::cos(combined_heading));
    }
    this->trajectory_path_lateral_error = lookahead_lateral;
    this->trajectory_path_heading_error = path_heading_error;
    const double heading_gain = this->params.Get<double>(
        "trajectory_heading_gain", 1.0);
    double requested_yaw = std::clamp(
        heading_gain * path_heading_error,
        -static_cast<double>(command_limits[2]),
        static_cast<double>(command_limits[2]));
    if (this->vision_trajectory_enabled)
    {
        // RearVisionTrajectoryCommand trained this checkpoint with vy=0: the
        // crack tangent and cross-track error are both converted to a yaw
        // command. Keep deployment inside that learned vx+wz distribution.
        // Keep the deployment-verified 0.30 m/s gait command.  Reducing it
        // during the same transition that introduces steering moved this
        // checkpoint into its weak low-speed regime and caused knee collapse.
        // This checkpoint's learned support transition is reliable at its
        // trained 0.30 m/s command but buckles in a synthetic 0.1--0.2 m/s
        // startup ramp.  Keep the command on the trained manifold; the FSM's
        // joint-space handoff removes the physical target discontinuity.
        this->control.x = std::clamp(
            static_cast<float>(requested_speed * start_speed_blend),
            0.0f, command_limits[0]);
        this->control.y = 0.0f;

        // A freshly segmented polyline is replaced at camera rate, so its
        // local tangent may change sign from one frame to the next. Smooth
        // and rate-limit the outer-loop yaw before it reaches the policy.
        const double vision_yaw_limit = std::min(
            static_cast<double>(command_limits[2]),
            std::max(0.0, this->params.Get<double>(
                "vision_heading_command_limit", 0.18)));
        requested_yaw = std::clamp(
            requested_yaw, -vision_yaw_limit, vision_yaw_limit);
        // First establish a repeatable trot with zero yaw.  Turning during
        // the policy/gait-starter handoff destabilizes this sim-to-sim export
        // even for a very small requested yaw.  Fade steering in only after
        // the diagonal gait and arm handoff have both settled.
        const double steering_delay = std::max(
            0.0, this->params.Get<double>(
                "vision_steering_delay", 8.0));
        const double steering_ramp = std::max(
            0.1, this->params.Get<double>(
                "vision_steering_ramp_time", 10.0));
        const double steering_blend = std::clamp(
            (trajectory_elapsed - steering_delay) / steering_ramp,
            0.0, 1.0);
        requested_yaw *= steering_blend;
        const double controller_dt = std::max(
            1.0e-4, static_cast<double>(this->params.Get<float>("dt")));

        // The exact RearVisionTrajectoryCommand used for this checkpoint
        // always emitted [vx, 0, wz].  The legacy deployment added a body-y
        // correction here, which moved the actor outside its training
        // command distribution and disturbed diagonal foot timing.  Keep the
        // old path available only as an explicit compatibility option.
        if (this->params.Get<bool>(
                "vision_enable_lateral_velocity_correction", false))
        {
            const double lateral_gain = std::max(
                0.0, this->params.Get<double>(
                         "vision_lateral_correction_gain", 0.20));
            const double lateral_limit = std::max(
                0.0, this->params.Get<double>(
                         "vision_lateral_command_limit", 0.08));
            const double requested_lateral = std::clamp(
                lateral_gain * lookahead_lateral,
                -lateral_limit, lateral_limit) * steering_blend;
            if (!this->vision_lateral_command_initialized)
            {
                this->vision_lateral_command = 0.0;
                this->vision_lateral_command_initialized = true;
            }
            const double lateral_filter_tau = std::max(
                controller_dt,
                this->params.Get<double>(
                    "vision_lateral_filter_time_constant", 0.80));
            const double lateral_filter_alpha =
                1.0 - std::exp(-controller_dt / lateral_filter_tau);
            const double filtered_lateral =
                this->vision_lateral_command + lateral_filter_alpha *
                    (requested_lateral - this->vision_lateral_command);
            const double lateral_rate_limit = std::max(
                0.0, this->params.Get<double>(
                         "vision_lateral_command_rate_limit", 0.04));
            const double max_lateral_step = lateral_rate_limit * controller_dt;
            this->vision_lateral_command = std::clamp(
                filtered_lateral,
                this->vision_lateral_command - max_lateral_step,
                this->vision_lateral_command + max_lateral_step);
            this->control.y = static_cast<float>(
                std::clamp(this->vision_lateral_command,
                           -static_cast<double>(command_limits[1]),
                           static_cast<double>(command_limits[1])));
        }
        else
        {
            this->vision_lateral_command = 0.0;
            this->vision_lateral_command_initialized = true;
            this->control.y = 0.0f;
        }

        if (!this->vision_heading_command_initialized)
        {
            this->vision_heading_command = 0.0;
            this->vision_heading_command_initialized = true;
        }
        const double heading_filter_tau = std::max(
            controller_dt,
            this->params.Get<double>(
                "vision_heading_filter_time_constant", 0.40));
        const double heading_filter_alpha =
            1.0 - std::exp(-controller_dt / heading_filter_tau);
        const double filtered_yaw =
            this->vision_heading_command +
            heading_filter_alpha *
                (requested_yaw - this->vision_heading_command);
        const double heading_rate_limit = std::max(
            0.0, this->params.Get<double>(
                "vision_heading_command_rate_limit", 0.30));
        const double max_heading_step = heading_rate_limit * controller_dt;
        this->vision_heading_command = std::clamp(
            filtered_yaw,
            this->vision_heading_command - max_heading_step,
            this->vision_heading_command + max_heading_step);
        requested_yaw = this->vision_heading_command;
    }
    this->control.yaw = static_cast<float>(requested_yaw);

    auto target_commands = this->params.Get<std::vector<float>>(
        "target_commands");
    if (this->vision_trajectory_enabled)
    {
        // RearVisionTrajectoryCommand trained this checkpoint from a
        // world-fixed crack point.  Reconstruct the same observation by
        // lifting the crack to the requested tool height in world z, then
        // applying the complete inverse base rotation.  This keeps the TCP at
        // a consistent physical height as B2 changes base height, roll and
        // pitch instead of pinning z in the moving base frame.
        std::array<double, 3> desired_target_b{{
            target_commands[0], target_commands[1], target_commands[2]}};
        const double lateral_limit = std::clamp(
            this->params.Get<double>("vision_policy_target_y_limit", 0.15),
            0.0, 0.15);
        if (rear_target_found)
        {
            const double target_delta_x = ee_target.x - base_pos[0];
            const double target_delta_y = ee_target.y - base_pos[1];
            if (this->params.Get<bool>(
                    "vision_policy_target_track_world_xyz", true))
            {
                const double ground_z = this->params.Get<double>(
                    "vision_ground_height_world", 0.0);
                const double tool_height = this->params.Get<double>(
                    "vision_tool_height_above_ground", 0.09);
                const double target_delta_z =
                    ground_z + tool_height - base_pos[2];
                desired_target_b[0] =
                    base_rot[0] * target_delta_x +
                    base_rot[3] * target_delta_y +
                    base_rot[6] * target_delta_z;
                desired_target_b[1] =
                    base_rot[1] * target_delta_x +
                    base_rot[4] * target_delta_y +
                    base_rot[7] * target_delta_z;
                desired_target_b[2] =
                    base_rot[2] * target_delta_x +
                    base_rot[5] * target_delta_y +
                    base_rot[8] * target_delta_z;
            }
            else
            {
                // Compatibility with checkpoints trained using fixed x/z.
                desired_target_b[1] =
                    target_delta_x * left_x + target_delta_y * left_y;
            }
        }

        const auto target_x_range = this->params.Get<std::vector<float>>(
            "vision_policy_target_x_range", {-1.0f, -0.55f});
        const auto target_z_range = this->params.Get<std::vector<float>>(
            "vision_policy_target_z_range", {-0.38f, -0.30f});
        if (target_x_range.size() >= 2)
        {
            desired_target_b[0] = std::clamp(
                desired_target_b[0],
                static_cast<double>(target_x_range[0]),
                static_cast<double>(target_x_range[1]));
        }
        desired_target_b[1] = std::clamp(
            desired_target_b[1], -lateral_limit, lateral_limit);
        if (target_z_range.size() >= 2)
        {
            desired_target_b[2] = std::clamp(
                desired_target_b[2],
                static_cast<double>(target_z_range[0]),
                static_cast<double>(target_z_range[1]));
        }

        if (!this->vision_policy_target_initialized)
        {
            this->vision_policy_target_b = {
                target_commands[0], target_commands[1], target_commands[2]};
            this->vision_policy_target_initialized = true;
        }
        const double controller_dt = std::max(
            1.0e-4, static_cast<double>(this->params.Get<float>("dt")));
        const double filter_tau = std::max(
            controller_dt,
            this->params.Get<double>(
                "vision_policy_target_filter_time_constant", 0.25));
        const double filter_alpha = 1.0 - std::exp(-controller_dt / filter_tau);
        const double target_transition_time = std::max(
            controller_dt,
            this->params.Get<double>(
                "vision_policy_target_transition_time", 4.0));
        double target_transition_blend = std::clamp(
            trajectory_elapsed / target_transition_time, 0.0, 1.0);
        // Match the zero-slope smoothstep in RearVisionTrajectoryCommand.
        target_transition_blend = target_transition_blend *
            target_transition_blend * (3.0 - 2.0 * target_transition_blend);
        const auto max_xyz_rate = this->params.Get<std::vector<float>>(
            "vision_policy_target_max_xyz_rate", {0.12f, 0.20f, 0.08f});
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            const double transitioned_target =
                this->vision_policy_target_start_b[axis] +
                target_transition_blend *
                    (desired_target_b[axis] -
                     this->vision_policy_target_start_b[axis]);
            const double filtered_target =
                this->vision_policy_target_b[axis] + filter_alpha *
                    (transitioned_target - this->vision_policy_target_b[axis]);
            const double fallback_rate = axis == 1
                ? this->params.Get<double>(
                      "vision_policy_target_max_lateral_rate", 0.20)
                : (axis == 0 ? 0.12 : 0.08);
            const double rate = std::max(
                0.0, axis < max_xyz_rate.size()
                         ? static_cast<double>(max_xyz_rate[axis])
                         : fallback_rate);
            const double max_step = rate * controller_dt;
            this->vision_policy_target_b[axis] = std::clamp(
                filtered_target,
                this->vision_policy_target_b[axis] - max_step,
                this->vision_policy_target_b[axis] + max_step);
        }
        target_commands[0] =
            static_cast<float>(this->vision_policy_target_b[0]);
        target_commands[1] =
            static_cast<float>(this->vision_policy_target_b[1]);
        target_commands[2] =
            static_cast<float>(this->vision_policy_target_b[2]);

        // Preserve the desired Cartesian point for visualization and error
        // measurement. The policy itself receives a small calibrated
        // feed-forward offset that cancels the repeatable MuJoCo TCP bias.
        const std::vector<float> desired_target_commands = target_commands;
        const auto tcp_compensation = this->params.Get<std::vector<float>>(
            "vision_policy_tcp_feedforward_compensation",
            {0.0f, 0.0f, 0.0f});
        if (tcp_compensation.size() >= 3)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                target_commands[axis] += tcp_compensation[axis];
            }
        }

        // Visualize the uncompensated desired TCP. The yellow path still
        // shows the raw detected crack independently. The green
        // point remains the true desired TCP rather than the compensated
        // policy observation, so red-to-green distance is meaningful.
        this->trajectory_current_target = {
            base_pos[0] + base_rot[0] * desired_target_commands[0] +
                base_rot[1] * desired_target_commands[1] +
                base_rot[2] * desired_target_commands[2],
            base_pos[1] + base_rot[3] * desired_target_commands[0] +
                base_rot[4] * desired_target_commands[1] +
                base_rot[5] * desired_target_commands[2],
            base_pos[2] + base_rot[6] * desired_target_commands[0] +
                base_rot[7] * desired_target_commands[1] +
                base_rot[8] * desired_target_commands[2],
            0.0};
    }
    else
    {
        // Static trajectories use a world target, so retain the exact full
        // base-frame transform for that separately trained operating mode.
        const double target_delta_x = ee_target.x - base_pos[0];
        const double target_delta_y = ee_target.y - base_pos[1];
        const double target_delta_z = ee_target.z - base_pos[2];
        target_commands[0] = static_cast<float>(
            base_rot[0] * target_delta_x +
            base_rot[3] * target_delta_y +
            base_rot[6] * target_delta_z);
        target_commands[1] = static_cast<float>(
            base_rot[1] * target_delta_x +
            base_rot[4] * target_delta_y +
            base_rot[7] * target_delta_z);
        target_commands[2] = static_cast<float>(
            base_rot[2] * target_delta_x +
            base_rot[5] * target_delta_y +
            base_rot[8] * target_delta_z);
        this->trajectory_current_target = ee_target;
    }
    this->SetRuntimeTargetCommands(target_commands);

    const double target_heading = std::atan2(tangent_y, tangent_x);
    this->motion_heading_quat = {
        static_cast<float>(std::cos(0.5 * target_heading)), 0.0f, 0.0f,
        static_cast<float>(std::sin(0.5 * target_heading))};
    this->motion_heading_anchor_valid = true;
    this->UpdateTrajectoryVisualization();
}

void RL_Sim::UpdateTrajectoryVisualization()
{
    if (!this->mj_data || !this->trajectory_world_initialized ||
        this->trajectory_world_points.empty())
    {
        return;
    }
    const double total_length = this->trajectory_world_points.back().s;
    const std::size_t marker_count =
        this->trajectory_path_visual_mocap_ids.size();
    for (std::size_t index = 0; index < marker_count; ++index)
    {
        const double blend = marker_count > 1
            ? static_cast<double>(index) /
                  static_cast<double>(marker_count - 1)
            : 0.0;
        const TrajectoryPoint point = this->SampleTrajectory(
            blend * total_length);
        mjtNum* position = this->mj_data->mocap_pos +
            3 * this->trajectory_path_visual_mocap_ids[index];
        position[0] = point.x;
        position[1] = point.y;
        position[2] = point.z;
    }
    if ((this->trajectory_control_active ||
         this->trajectory_end_hold_active) &&
        this->trajectory_target_visual_mocap_id >= 0)
    {
        mjtNum* position = this->mj_data->mocap_pos +
            3 * this->trajectory_target_visual_mocap_id;
        position[0] = this->trajectory_current_target.x;
        position[1] = this->trajectory_current_target.y;
        position[2] = this->trajectory_current_target.z;
    }

    // Report the quantity that matters for deployment: the actual extended
    // dummy_target TCP (red) relative to the command consumed by the policy
    // (green).  This distinguishes policy tracking error from perception/path
    // error without adding anything to the real-time policy observation.
    if (this->trajectory_control_active && this->trajectory_ee_site_id >= 0 &&
        (this->trajectory_last_tracking_diagnostic_time < 0.0 ||
         this->mj_data->time -
                 this->trajectory_last_tracking_diagnostic_time >=
             1.0))
    {
        const mjtNum* actual =
            this->mj_data->site_xpos + 3 * this->trajectory_ee_site_id;
        const double error_x = actual[0] - this->trajectory_current_target.x;
        const double error_y = actual[1] - this->trajectory_current_target.y;
        const double error_z = actual[2] - this->trajectory_current_target.z;
        std::cout << LOGGER::INFO << std::fixed << std::setprecision(3)
                  << "[TCP] target=[" << this->trajectory_current_target.x
                  << ", " << this->trajectory_current_target.y << ", "
                  << this->trajectory_current_target.z << "] actual=["
                  << actual[0] << ", " << actual[1] << ", " << actual[2]
                  << "] error_xyz=[" << error_x << ", " << error_y << ", "
                  << error_z << "] error_norm="
                  << std::sqrt(error_x * error_x + error_y * error_y +
                               error_z * error_z)
                  << " path_lateral="
                  << this->trajectory_path_lateral_error
                  << " path_heading="
                  << this->trajectory_path_heading_error
                  << std::defaultfloat << std::endl;
        this->trajectory_last_tracking_diagnostic_time = this->mj_data->time;
    }
}

void RL_Sim::GetState(RobotState<float> *state)
{
    if (mj_data)
    {
        state->imu.quaternion[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 0];
        state->imu.quaternion[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 1];
        state->imu.quaternion[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 2];
        state->imu.quaternion[3] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 3];

        state->imu.gyroscope[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 4];
        state->imu.gyroscope[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 5];
        state->imu.gyroscope[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 6];

        // MuJoCo's frame-linear-velocity sensor is expressed in the world
        // frame. Convert it to the base frame so deployment diagnostics show
        // the same velocity convention as Isaac Lab.
        const int frame_velocity_offset =
            3 * this->params.Get<int>("num_of_dofs") + 13;
        this->obs.lin_vel = QuatRotateInverse(
            state->imu.quaternion,
            {
                static_cast<float>(mj_data->sensordata[frame_velocity_offset + 0]),
                static_cast<float>(mj_data->sensordata[frame_velocity_offset + 1]),
                static_cast<float>(mj_data->sensordata[frame_velocity_offset + 2]),
            });

        if (this->trajectory_mode_available)
        {
            this->obs.base_height = this->trajectory_base_body_id >= 0
                ? static_cast<float>(
                    mj_data->xpos[3 * this->trajectory_base_body_id + 2])
                : 0.0f;
            this->obs.foot_contacts.assign(4, 0);
            this->obs.undesired_ground_contact = false;
            if (!this->trajectory_ground_geom_ids.empty())
            {
                for (int contact_index = 0;
                     contact_index < mj_data->ncon; ++contact_index)
                {
                    const mjContact& contact = mj_data->contact[contact_index];
                    int robot_geom = -1;
                    const bool geom1_is_ground = std::find(
                        this->trajectory_ground_geom_ids.begin(),
                        this->trajectory_ground_geom_ids.end(),
                        contact.geom1) !=
                        this->trajectory_ground_geom_ids.end();
                    const bool geom2_is_ground = std::find(
                        this->trajectory_ground_geom_ids.begin(),
                        this->trajectory_ground_geom_ids.end(),
                        contact.geom2) !=
                        this->trajectory_ground_geom_ids.end();
                    if (geom1_is_ground)
                    {
                        robot_geom = contact.geom2;
                    }
                    else if (geom2_is_ground)
                    {
                        robot_geom = contact.geom1;
                    }
                    if (robot_geom < 0)
                    {
                        continue;
                    }

                    bool is_foot = false;
                    for (std::size_t foot_index = 0;
                         foot_index < this->trajectory_foot_geom_ids.size();
                         ++foot_index)
                    {
                        if (robot_geom ==
                            this->trajectory_foot_geom_ids[foot_index])
                        {
                            this->obs.foot_contacts[foot_index] = 1;
                            is_foot = true;
                            break;
                        }
                    }
                    if (!is_foot)
                    {
                        this->obs.undesired_ground_contact = true;
                    }
                }
            }
        }

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            state->motor_state.q[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i]];
            state->motor_state.dq[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + this->params.Get<int>("num_of_dofs")];
            state->motor_state.tau_est[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + 2 * this->params.Get<int>("num_of_dofs")];
        }
    }
}

void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    if (mj_data)
    {
        std::array<float, 6> arm_residual_q{{0, 0, 0, 0, 0, 0}};
        const bool arm_dofs_available = std::all_of(
            this->trajectory_arm_dof_ids.begin(),
            this->trajectory_arm_dof_ids.end(),
            [](int dof_id) { return dof_id >= 0; });
        if (this->robot_name == "b2_z1_no_gun" &&
            this->trajectory_control_active && this->rl_init_done &&
            this->trajectory_ee_site_id >= 0 && arm_dofs_available &&
            this->params.Get<bool>(
                "enable_vision_tcp_residual_correction", false))
        {
            const mjtNum* actual = this->mj_data->site_xpos +
                3 * this->trajectory_ee_site_id;
            double error[3] = {
                this->trajectory_current_target.x - actual[0],
                this->trajectory_current_target.y - actual[1],
                this->trajectory_current_target.z - actual[2]};
            // Height is the repeatable sim-to-sim residual in this setup.
            // Do not let lateral segmentation motion or the deliberately
            // moving rear-x target inject extra joint corrections into Z1.
            if (this->params.Get<bool>(
                    "vision_tcp_residual_vertical_only", true))
            {
                error[0] = 0.0;
                error[1] = 0.0;
            }
            // Reject perception/startup outliers; this controller is only for
            // the measured centimetre-scale deployment residual.
            for (double& component : error)
            {
                component = std::clamp(component, -0.12, 0.12);
            }

            std::vector<mjtNum> jacobian(
                3 * static_cast<std::size_t>(this->mj_model->nv), 0.0);
            mj_jacSite(
                this->mj_model, this->mj_data, jacobian.data(), nullptr,
                this->trajectory_ee_site_id);
            double j[3][6]{};
            for (int row = 0; row < 3; ++row)
            {
                for (int column = 0; column < 6; ++column)
                {
                    j[row][column] = jacobian[
                        row * this->mj_model->nv +
                        this->trajectory_arm_dof_ids[column]];
                }
            }

            const double damping = std::max(
                1.0e-4, this->params.Get<double>(
                    "vision_tcp_residual_damping", 0.08));
            double a[3][3]{};
            for (int row = 0; row < 3; ++row)
            {
                for (int column = 0; column < 3; ++column)
                {
                    for (int joint = 0; joint < 6; ++joint)
                    {
                        a[row][column] +=
                            j[row][joint] * j[column][joint];
                    }
                }
                a[row][row] += damping * damping;
            }
            const double determinant =
                a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
            if (std::fabs(determinant) > 1.0e-10)
            {
                const double inverse[3][3] = {
                    {(a[1][1] * a[2][2] - a[1][2] * a[2][1]) / determinant,
                     (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / determinant,
                     (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / determinant},
                    {(a[1][2] * a[2][0] - a[1][0] * a[2][2]) / determinant,
                     (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / determinant,
                     (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / determinant},
                    {(a[1][0] * a[2][1] - a[1][1] * a[2][0]) / determinant,
                     (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / determinant,
                     (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / determinant}};
                double cartesian_step[3]{};
                for (int row = 0; row < 3; ++row)
                {
                    for (int column = 0; column < 3; ++column)
                    {
                        cartesian_step[row] +=
                            inverse[row][column] * error[column];
                    }
                }
                const double delay = std::max(
                    0.0, this->params.Get<double>(
                        "vision_tcp_residual_delay", 0.0));
                const double ramp_time = std::max(
                    0.1, this->params.Get<double>(
                        "vision_tcp_residual_ramp_time", 1.50));
                const double ramp = this->trajectory_control_start_time >= 0.0
                    ? std::clamp(
                          (this->mj_data->time -
                           this->trajectory_control_start_time - delay) /
                              ramp_time,
                          0.0, 1.0)
                    : 0.0;
                const double gain = std::clamp(
                    this->params.Get<double>(
                        "vision_tcp_residual_gain", 0.55),
                    0.0, 2.0) * ramp;
                const double max_joint_offset = std::max(
                    0.0, this->params.Get<double>(
                        "vision_tcp_residual_max_joint_offset", 0.10));
                std::array<double, 6> raw_delta{};
                double maximum_absolute_delta = 0.0;
                for (int joint = 0; joint < 6; ++joint)
                {
                    double delta = 0.0;
                    for (int row = 0; row < 3; ++row)
                    {
                        delta += j[row][joint] * cartesian_step[row];
                    }
                    raw_delta[joint] = gain * delta;
                    maximum_absolute_delta = std::max(
                        maximum_absolute_delta,
                        std::fabs(raw_delta[joint]));
                }
                // Preserve the damped least-squares joint direction.  An
                // independent clamp changes the Cartesian direction and was
                // observed to cancel most of the requested downward motion.
                const double joint_scale =
                    maximum_absolute_delta > max_joint_offset &&
                            maximum_absolute_delta > 1.0e-12
                        ? max_joint_offset / maximum_absolute_delta
                        : 1.0;
                for (int joint = 0; joint < 6; ++joint)
                {
                    arm_residual_q[joint] = static_cast<float>(
                        raw_delta[joint] * joint_scale);
                }
            }
        }

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            const int actuator_index =
                this->params.Get<std::vector<int>>("joint_mapping")[i];
            const bool is_b2_z1_arm =
                this->robot_name == "b2_z1_no_gun" &&
                (i == 8 || i >= 13);
            if (is_b2_z1_arm)
            {
                // Z1 is an implicit position actuator in Isaac Lab. Sending
                // an explicit high-gain torque into its very small distal
                // inertias makes MuJoCo numerically chatter. The matching
                // MJCF actuators below accept desired joint position here.
                // A MuJoCo position actuator ignores the FSM's software kp.
                // Passive/GetDown states intentionally set kp=0 and leave q
                // unspecified; forwarding that zero made joint2/3/4 snap from
                // their home pose to zero at startup. Emulate a passive arm
                // safely by holding the measured joint position until an FSM
                // state explicitly enables position control.
                int arm_index = i == 8 ? 0 : i - 12;
                mj_data->ctrl[actuator_index] =
                    command->motor_command.kp[i] > 1.0e-6f
                        ? command->motor_command.q[i] +
                              arm_residual_q[arm_index]
                        : mj_data->sensordata[actuator_index];
            }
            else
            {
                mj_data->ctrl[actuator_index] =
                    command->motor_command.tau[i] +
                    command->motor_command.kp[i] *
                        (command->motor_command.q[i] -
                         mj_data->sensordata[actuator_index]) +
                    command->motor_command.kd[i] *
                        (command->motor_command.dq[i] -
                         mj_data->sensordata[
                             actuator_index +
                             this->params.Get<int>("num_of_dofs")]);
            }
        }
    }
}

void RL_Sim::RobotControl()
{
    // Lock the sim mutex once for the entire control cycle to prevent race conditions
    const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

    this->GetState(&this->robot_state);
    this->ExportVisionState();
    this->IngestVisionTrajectory();

    const bool toggle_trajectory =
        this->control.current_keyboard == Input::Keyboard::T;
    const bool emergency_stop_trajectory =
        (this->trajectory_control_active ||
         this->trajectory_end_hold_active) &&
        this->control.current_keyboard == Input::Keyboard::Space;
    if (toggle_trajectory)
    {
        if (this->trajectory_control_active)
        {
            this->StopTrajectoryControl("operator toggle");
        }
        else
        {
            this->StartTrajectoryControl();
        }
    }
    if (emergency_stop_trajectory)
    {
        this->StopTrajectoryControl("operator emergency stop");
    }
    if (this->trajectory_end_hold_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
    }

    this->StateController(&this->robot_state, &this->robot_command);

    // Apply after StateController so manual W/A/S/D processing cannot
    // overwrite the autonomous follower while trajectory mode is active.
    this->UpdateTrajectoryControl();
    if (this->trajectory_end_hold_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
    }

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        if (this->mj_model && this->mj_data)
        {
            this->StopTrajectoryControl("simulation reset");
            mj_resetData(this->mj_model, this->mj_data);
            mj_forward(this->mj_model, this->mj_data);
            this->trajectory_world_initialized = false;
            this->trajectory_loaded = false;
            this->vision_trajectory_fresh = false;
            this->vision_rear_target_active = false;
            this->vision_policy_target_initialized = false;
            this->vision_heading_command_initialized = false;
            this->vision_heading_command = 0.0;
            this->vision_lateral_command_initialized = false;
            this->vision_lateral_command = 0.0;
            this->trajectory_last_tracking_diagnostic_time = -1.0;
            this->trajectory_control_start_time = -1.0;
            this->vision_trajectory_mtime_valid = false;
            this->vision_last_state_export_time = -1.0;
        }
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
            sim->run = 0;
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            sim->run = 1;
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Sim::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js = std::make_unique<Joystick>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::ERROR << "Joystick [" << device << "] open failed." << std::endl;
        // exit(1);
    }

    this->sys_js_max_value = (1 << (bits - 1));
}

void RL_Sim::GetSysJoystick()
{
    // Clear all button event states
    for (int i = 0; i < 20; ++i)
    {
        this->sys_js_button[i].on_press = false;
        this->sys_js_button[i].on_release = false;
    }

    // Check if joystick is valid before using
    if (!this->sys_js)
    {
        return;
    }

    while (this->sys_js->sample(&this->sys_js_event))
    {
        if (this->sys_js_event.isButton())
        {
            this->sys_js_button[this->sys_js_event.number].update(this->sys_js_event.value);
        }
        else if (this->sys_js_event.isAxis())
        {
            double normalized = double(this->sys_js_event.value) / this->sys_js_max_value;
            if (std::abs(normalized) < this->axis_deadzone)
            {
                this->sys_js_axis[this->sys_js_event.number] = 0;
            }
            else
            {
                this->sys_js_axis[this->sys_js_event.number] = this->sys_js_event.value;
            }
        }
    }

    if (this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::A);
    if (this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::B);
    if (this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::X);
    if (this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->sys_js_button[4].on_press) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->sys_js_button[4].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->sys_js_button[4].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->sys_js_button[4].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->sys_js_button[4].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[4].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[5].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->sys_js_button[5].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->sys_js_button[5].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->sys_js_button[5].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->sys_js_button[5].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[5].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[4].pressed && this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::LB_RB);

    float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (has_input)
    {
        this->control.x = ly;
        this->control.y = lx;
        this->control.yaw = rx;
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->sys_js_active = false;
    }
}

void RL_Sim::RunModel()
{
    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.base_quat = this->robot_state.imu.quaternion;
        const std::vector<float> requested_commands = {
            this->control.x, this->control.y, this->control.yaw};
        // The trajectory follower already computes body-frame velocity and
        // path-heading feedback. Applying the manual straight-line heading
        // hold again would rotate and correct the command twice.
        std::vector<float> target_commands =
            this->trajectory_control_active
                ? requested_commands
                : this->ApplyMotionHeadingHold(requested_commands);
        const float minimum_locomotion_speed =
            this->params.Get<float>(
                "minimum_policy_locomotion_speed", 0.0f);
        const float requested_planar_speed =
            std::hypot(target_commands[0], target_commands[1]);
        // Manual low-speed commands use a verified locomotion floor. The
        // autonomous follower must be allowed to decelerate below that floor
        // near its endpoint; otherwise it arrives at 0.20 m/s and has to make
        // an abrupt stop that can overturn the B2+Z1 assembly.
        if (!this->trajectory_control_active &&
            requested_planar_speed > 1.0e-4f &&
            requested_planar_speed < minimum_locomotion_speed)
        {
            const float scale =
                minimum_locomotion_speed / requested_planar_speed;
            target_commands[0] *= scale;
            target_commands[1] *= scale;
        }
        const auto policy_command_limits =
            this->params.Get<std::vector<float>>(
                "policy_command_limits",
                {1000.0f, 1000.0f, 1000.0f});
        for (std::size_t i = 0; i < target_commands.size(); ++i)
        {
            target_commands[i] = std::clamp(
                target_commands[i],
                -policy_command_limits[i],
                policy_command_limits[i]);
        }

        const auto command_accel_limits =
            this->params.Get<std::vector<float>>(
                "command_accel_limits",
                {1000.0f, 1000.0f, 1000.0f});
        const auto command_decel_limits =
            this->params.Get<std::vector<float>>(
                "command_decel_limits",
                {1000.0f, 1000.0f, 1000.0f});
        const float policy_dt =
            this->params.Get<float>("dt") *
            static_cast<float>(this->params.Get<int>("decimation"));
        for (std::size_t i = 0; i < this->obs.commands.size(); ++i)
        {
            const float current = this->obs.commands[i];
            const float target = target_commands[i];
            const bool accelerating =
                current * target >= 0.0f &&
                std::fabs(target) > std::fabs(current);
            const float rate = accelerating
                ? command_accel_limits[i]
                : command_decel_limits[i];
            const float max_step =
                std::max(0.0f, rate) * policy_dt;
            this->obs.commands[i] += std::clamp(
                target - current, -max_step, max_step);
        }
        //not currently available for non-ros mujoco version
        // if (this->control.navigation_mode)
        // {
        //     this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
        // }
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        const std::vector<float> policy_actions = this->Forward();
        if (this->params.Get<bool>(
                "use_raw_policy_action_observation", false))
        {
            // JointPositionAction in the historical training run exposes the
            // actor's raw preceding output through mdp.last_action.  Keep the
            // deployment-only output smoother out of this observation.
            this->obs.actions = this->raw_policy_actions;
        }
        else if (!this->params.Get<bool>(
                     "sync_applied_action_observation", false))
        {
            this->obs.actions = policy_actions;
        }
        this->ComputeOutput(policy_actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // Match the real B2+Z1 deployment safety boundary. This does not hide
        // a gait instability: it prevents a fallen robot from continuing to
        // execute policy targets and damaging itself or the environment.
        this->AttitudeProtect(this->robot_state.imu.quaternion, 45.0f, 45.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est(this->params.Get<int>("num_of_dofs"), 0.0f);
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Sim::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    // RSL-RL clips the actor output before env.step(). Consequently both
    // action_manager.action (mdp.last_action) and the custom action term see
    // this bounded value. Reproduce that ordering before the EMA filter.
    const float policy_input_clip =
        this->params.Get<float>("policy_input_clip", 0.0f);
    if (policy_input_clip > 0.0f)
    {
        for (float &action : actions)
        {
            action = std::clamp(
                action, -policy_input_clip, policy_input_clip);
        }
    }
    this->raw_policy_actions = actions;

    // The actuator-side filter needs its own state.  obs.actions may contain
    // the raw last policy action to match training and therefore cannot also
    // serve as the previous applied/filtered action.
    std::vector<float> previous_applied_actions;
    if (this->reset_policy_action_state)
    {
        previous_applied_actions.assign(actions.size(), 0.0f);
        this->filtered_policy_actions = previous_applied_actions;
        this->reset_policy_action_state = false;
    }
    else
    {
        // When the FSM feeds the final actuator target back as last_action,
        // that value must also be the EMA's previous state.  Handoff/safety
        // shaping occurs after Forward(), so a separate hidden filter state
        // would otherwise diverge from the policy observation.
        previous_applied_actions = this->params.Get<bool>(
            "sync_applied_action_observation", false)
            ? this->obs.actions
            : this->filtered_policy_actions;
    }
    if (previous_applied_actions.size() != actions.size())
    {
        previous_applied_actions = this->obs.actions;
    }
    const float action_filter_alpha =
        this->params.Get<float>("action_filter_alpha", 1.0f);
    if (action_filter_alpha < 1.0f)
    {
        const float alpha =
            std::clamp(action_filter_alpha, 0.0f, 1.0f);
        if (previous_applied_actions.size() == actions.size())
        {
            for (std::size_t i = 0; i < actions.size(); ++i)
            {
                actions[i] =
                    alpha * actions[i] +
                    (1.0f - alpha) * previous_applied_actions[i];
            }
        }
    }

    // Constrain only Z1 target slew in the applied-action path.
    const float arm_target_rate_limit =
        this->params.Get<float>("arm_target_rate_limit", 0.0f);
    if (arm_target_rate_limit > 0.0f &&
        previous_applied_actions.size() == actions.size())
    {
        const float policy_period =
            this->params.Get<float>("dt") *
            this->params.Get<int>("decimation");
        const auto action_scale =
            this->params.Get<std::vector<float>>("action_scale");
        const auto arm_indices =
            this->params.Get<std::vector<int>>("arm_action_indices");
        for (const int index : arm_indices)
        {
            if (index < 0 ||
                static_cast<std::size_t>(index) >= actions.size() ||
                static_cast<std::size_t>(index) >= action_scale.size())
            {
                continue;
            }
            const float scale = std::max(std::fabs(action_scale[index]), 1.0e-6f);
            const float max_action_step =
                arm_target_rate_limit * policy_period / scale;
            actions[index] = std::clamp(
                actions[index],
                previous_applied_actions[index] - max_action_step,
                previous_applied_actions[index] + max_action_step);
        }
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        actions = clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    this->filtered_policy_actions = actions;
    return actions;
}

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(mj_data->sensordata[i]);
        // this->plot_target_joint_pos[i].push_back();  // TODO
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}

// Signal handler for Ctrl+C
void signalHandler(int signum)
{
    std::cout << LOGGER::INFO << "Received signal " << signum << ", exiting..." << std::endl;
    if (RL_Sim::instance && RL_Sim::instance->sim)
    {
        RL_Sim::instance->sim->exitrequest.store(1);
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);
    RL_Sim rl_sar(argc, argv);
    return 0;
}
