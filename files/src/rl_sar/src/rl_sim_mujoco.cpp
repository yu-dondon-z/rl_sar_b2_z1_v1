/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim_mujoco.hpp"

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
                mj_data->ctrl[actuator_index] =
                    command->motor_command.q[i];
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

    this->StateController(&this->robot_state, &this->robot_command);

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        if (this->mj_model && this->mj_data)
        {
            mj_resetData(this->mj_model, this->mj_data);
            mj_forward(this->mj_model, this->mj_data);
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
        std::vector<float> target_commands = {
            this->control.x, this->control.y, this->control.yaw};
        const float minimum_locomotion_speed =
            this->params.Get<float>(
                "minimum_policy_locomotion_speed", 0.0f);
        const float requested_planar_speed =
            std::hypot(target_commands[0], target_commands[1]);
        if (requested_planar_speed > 1.0e-4f &&
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
        this->obs.base_quat = this->robot_state.imu.quaternion;
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

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

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

    this->raw_policy_actions = actions;
    // The actuator-side filter needs its own state.  obs.actions may contain
    // the raw last policy action to match training and therefore cannot also
    // serve as the previous applied/filtered action.
    std::vector<float> previous_applied_actions =
        this->filtered_policy_actions;
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
