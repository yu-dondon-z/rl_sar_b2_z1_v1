/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim.hpp"

namespace
{
constexpr float kB2Z1LinearSpeedLimit = 0.5f;
constexpr float kB2Z1LateralSpeedLimit = 0.4f;
constexpr float kB2Z1YawSpeedLimit = 0.0f;
}

RL_Sim::RL_Sim(int argc, char **argv)
{
#if defined(USE_ROS1)
    this->ang_vel_axis = "world";
    ros::NodeHandle nh;
    nh.param<std::string>("ros_namespace", this->ros_namespace, "");
    nh.param<std::string>("robot_name", this->robot_name, "");
#elif defined(USE_ROS2)
    ros2_node = std::make_shared<rclcpp::Node>("rl_sim_node");
    this->ang_vel_axis = "body";
    this->ros_namespace = ros2_node->get_namespace();
    // get params from param_node
    param_client = ros2_node->create_client<rcl_interfaces::srv::GetParameters>("/param_node/get_parameters");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()) {
            std::cout << LOGGER::ERROR << "Interrupted while waiting for param_node service. Exiting." << std::endl;
            return;
        }
        std::cout << LOGGER::WARNING << "Waiting for param_node service to be available..." << std::endl;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_name", "gazebo_model_name"};
    // Use a timeout for the future
    auto future = param_client->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(ros2_node->get_node_base_interface(), future, std::chrono::seconds(5));
    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (result->values.size() < 2)
        {
            std::cout << LOGGER::ERROR << "Failed to get all parameters from param_node" << std::endl;
        }
        else
        {
            this->robot_name = result->values[0].string_value;
            this->gazebo_model_name = result->values[1].string_value;
            std::cout << LOGGER::INFO << "Get param robot_name: " << this->robot_name << std::endl;
            std::cout << LOGGER::INFO << "Get param gazebo_model_name: " << this->gazebo_model_name << std::endl;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "Failed to call param_node service" << std::endl;
    }
#endif

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
#if defined(USE_ROS1)
    this->joint_publishers_commands.resize(this->params.Get<int>("num_of_dofs"));
#elif defined(USE_ROS2)
    this->robot_command_publisher_msg.motor_command.resize(this->params.Get<int>("num_of_dofs"));
    this->robot_state_subscriber_msg.motor_state.resize(this->params.Get<int>("num_of_dofs"));
#endif
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

#if defined(USE_ROS1)
    auto joint_controller_names_vec = this->params.Get<std::vector<std::string>>("joint_controller_names");  // avoid dangling reference
    this->StartJointController(this->ros_namespace, joint_controller_names_vec);
    // publisher
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        const std::string &joint_controller_name = joint_controller_names_vec[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/command";
        this->joint_publishers[joint_controller_name] =
            nh.advertise<robot_msgs::MotorCommand>(topic_name, 10);
    }

    // subscriber
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Sim::CmdvelCallback, this);
    this->joy_subscriber = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &RL_Sim::JoyCallback, this);
    this->model_state_subscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &RL_Sim::ModelStatesCallback, this);
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        const std::string &joint_controller_name = joint_controller_names_vec[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/state";
        this->joint_subscribers[joint_controller_name] =
            nh.subscribe<robot_msgs::MotorState>(topic_name, 10,
                [this, joint_controller_name](const robot_msgs::MotorState::ConstPtr &msg)
                {
                    this->JointStatesCallback(msg, joint_controller_name);
                }
            );
        this->joint_positions[joint_controller_name] = 0.0f;
        this->joint_velocities[joint_controller_name] = 0.0f;
        this->joint_efforts[joint_controller_name] = 0.0f;
    }

    // service
    nh.param<std::string>("gazebo_model_name", this->gazebo_model_name, "");
    this->gazebo_pause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    this->gazebo_unpause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    this->gazebo_reset_world_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
#elif defined(USE_ROS2)
    this->StartJointController(this->ros_namespace, this->params.Get<std::vector<std::string>>("joint_names"));
    // publisher
    this->robot_command_publisher = ros2_node->create_publisher<robot_msgs::msg::RobotCommand>(
        this->ros_namespace + "robot_joint_controller/command", rclcpp::SystemDefaultsQoS());

    // subscriber
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
    this->joy_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SystemDefaultsQoS(),
        [this] (const sensor_msgs::msg::Joy::SharedPtr msg) {this->JoyCallback(msg);}
    );
    this->gazebo_imu_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(), [this] (const sensor_msgs::msg::Imu::SharedPtr msg) {this->GazeboImuCallback(msg);}
    );
    this->model_state_subscriber = ros2_node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/model_states", rclcpp::SystemDefaultsQoS(),
        [this] (const gazebo_msgs::msg::ModelStates::SharedPtr msg) {this->ModelStatesCallback(msg);}
    );
    this->robot_state_subscriber = ros2_node->create_subscription<robot_msgs::msg::RobotState>(
        this->ros_namespace + "robot_joint_controller/state", rclcpp::SystemDefaultsQoS(),
        [this] (const robot_msgs::msg::RobotState::SharedPtr msg) {this->RobotStateCallback(msg);}
    );

    // service
    this->gazebo_pause_physics_client = ros2_node->create_client<std_srvs::srv::Empty>("/pause_physics");
    this->gazebo_unpause_physics_client = ros2_node->create_client<std_srvs::srv::Empty>("/unpause_physics");
    this->gazebo_reset_world_client = ros2_node->create_client<std_srvs::srv::Empty>("/reset_world");

    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
#endif

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

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
}

RL_Sim::~RL_Sim()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names)
{
#if defined(USE_ROS1)
    pid_t pid0 = fork();
    if (pid0 == 0)
    {
        std::string cmd = "rosrun controller_manager spawner joint_state_controller ";
        for (const auto& name : names)
        {
            cmd += name + " ";
        }
        cmd += "__ns:=" + ros_namespace;
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
#elif defined(USE_ROS2)
    const char* ros_distro = std::getenv("ROS_DISTRO");
    std::string spawner = (ros_distro && std::string(ros_distro) == "foxy") ? "spawner.py" : "spawner";

    std::filesystem::path tmp_path = std::filesystem::temp_directory_path() / "robot_joint_controller_params.yaml";
    {
        std::ofstream tmp_file(tmp_path);
        if (!tmp_file)
        {
            throw std::runtime_error("Failed to create temporary parameter file");
        }

        tmp_file << "/robot_joint_controller:\n";
        tmp_file << "    ros__parameters:\n";
        tmp_file << "        joints:\n";
        for (const auto& name : names)
        {
            tmp_file << "            - " << name << "\n";
        }
    }

    pid_t pid = fork();
    if (pid == 0)
    {
        std::string cmd = "ros2 run controller_manager " + spawner + " robot_joint_controller ";
        cmd += "-p " + tmp_path.string() + " ";
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
    else if (pid > 0)
    {
        int status;
        waitpid(pid, &status, 0);

        if (WIFEXITED(status) && WEXITSTATUS(status) != 0)
        {
            throw std::runtime_error("Failed to start joint controller");
        }

        std::filesystem::remove(tmp_path);
    }
    else
    {
        throw std::runtime_error("fork() failed");
    }
#endif
}

void RL_Sim::GetState(RobotState<float> *state)
{
#if defined(USE_ROS1)
    const auto &orientation = this->pose.orientation;
    const auto &angular_velocity = this->vel.angular;
#elif defined(USE_ROS2)
    std::lock_guard<std::mutex> state_lock(this->robot_state_msg_mutex_);
    const auto &orientation = this->gazebo_imu.orientation;
    const auto &angular_velocity = this->gazebo_imu.angular_velocity;
#endif

    state->imu.quaternion[0] = orientation.w;
    state->imu.quaternion[1] = orientation.x;
    state->imu.quaternion[2] = orientation.y;
    state->imu.quaternion[3] = orientation.z;

    state->imu.gyroscope[0] = angular_velocity.x;
    state->imu.gyroscope[1] = angular_velocity.y;
    state->imu.gyroscope[2] = angular_velocity.z;

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
#if defined(USE_ROS1)
        state->motor_state.q[i] = this->joint_positions[this->params.Get<std::vector<std::string>>("joint_controller_names")[this->params.Get<std::vector<int>>("joint_mapping")[i]]];
        state->motor_state.dq[i] = this->joint_velocities[this->params.Get<std::vector<std::string>>("joint_controller_names")[this->params.Get<std::vector<int>>("joint_mapping")[i]]];
        state->motor_state.tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[this->params.Get<std::vector<int>>("joint_mapping")[i]]];
#elif defined(USE_ROS2)
        state->motor_state.q[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].q;
        state->motor_state.dq[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq;
        state->motor_state.tau_est[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau_est;
#endif
    }
}

void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
#if defined(USE_ROS1)
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].q = command->motor_command.q[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq = command->motor_command.dq[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp = command->motor_command.kp[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd = command->motor_command.kd[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau = command->motor_command.tau[i];
#elif defined(USE_ROS2)
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].q = command->motor_command.q[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq = command->motor_command.dq[i];
        float sim_kp = command->motor_command.kp[i];
        float sim_kd = command->motor_command.kd[i];
        const bool is_b2_z1_leg =
            this->robot_name == "b2_z1_no_gun" &&
            !(i == 8 || i >= 13);
        // Gazebo's explicit effort controller tracks the Isaac Lab target
        // trajectory substantially more softly than the trained actuator.
        // Compensate only the RL gain level (160/5); fixed GetUp gains and Z1
        // remain unchanged, and this path is never used on the real robot.
        if (is_b2_z1_leg && sim_kp > 0.0f && sim_kp <= 170.0f)
        {
            sim_kp *= 1.25f;
            sim_kd *= 1.20f;
        }
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp = sim_kp;
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd = sim_kd;
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau = command->motor_command.tau[i];
#endif
    }

#if defined(USE_ROS1)
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->joint_publishers[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]].publish(this->joint_publishers_commands[i]);
    }
#elif defined(USE_ROS2)
    this->robot_command_publisher->publish(this->robot_command_publisher_msg);
#endif
}

void RL_Sim::RobotControl()
{
    this->GetState(&this->robot_state);

    // Stop active control before a divergent physics state reaches Gazebo's
    // renderer. Ogre aborts when a link transform produces an invalid AABB.
    if (this->rl_init_done)
    {
        bool unsafe = false;
        std::string reason;
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            const float q = this->robot_state.motor_state.q[i];
            const float dq = this->robot_state.motor_state.dq[i];
            const bool is_arm = (i == 8 || i >= 13);
            // Do not drop the supporting leg torques for a short Z1 wrist
            // transient. The arm is held separately in the no-gun FSM; only
            // a clearly divergent arm speed should stop the whole robot.
            const float velocity_limit = is_arm ? 25.0f : 35.0f;
            if (!std::isfinite(q) || !std::isfinite(dq))
            {
                unsafe = true;
                reason = "non-finite joint state at index " + std::to_string(i);
                break;
            }
            if (std::fabs(dq) > velocity_limit)
            {
                unsafe = true;
                reason = "joint velocity limit exceeded at index " +
                    std::to_string(i) + " (dq=" + std::to_string(dq) +
                    ", limit=" + std::to_string(velocity_limit) + ")";
                break;
            }
            const bool is_hip = i < 4;
            const bool is_thigh = i >= 4 && i <= 7;
            const bool is_calf = i >= 9 && i <= 12;
            const float lower =
                is_hip ? -0.87f : (is_thigh ? -0.94f : (is_calf ? -2.82f : -3.2f));
            const float upper =
                is_hip ? 0.87f : (is_thigh ? 4.69f : (is_calf ? -0.43f : 3.2f));
            if (q < lower - 0.15f || q > upper + 0.15f)
            {
                unsafe = true;
                reason = "joint position limit exceeded at index " +
                    std::to_string(i) + " (q=" + std::to_string(q) + ")";
                break;
            }
        }

        const auto &quat = this->robot_state.imu.quaternion;
        const float quat_norm = std::sqrt(
            quat[0] * quat[0] + quat[1] * quat[1] +
            quat[2] * quat[2] + quat[3] * quat[3]);
        if (!std::isfinite(quat_norm) || quat_norm < 0.5f)
        {
            unsafe = true;
            reason = "invalid IMU quaternion";
        }
        else if (!unsafe)
        {
            const auto euler = QuaternionToEuler(quat);
            constexpr float kMaxTiltRadians = 0.959931f; // 55 degrees
            if (std::fabs(euler[0]) > kMaxTiltRadians ||
                std::fabs(euler[1]) > kMaxTiltRadians)
            {
                unsafe = true;
                reason = "body tilt exceeded 55 degrees";
            }
        }

        if (unsafe)
        {
            static bool safety_reported = false;
            if (!safety_reported)
            {
                std::cout << std::endl << LOGGER::ERROR
                          << "Simulation safety stop: " << reason
                          << "; switching to passive mode" << std::endl;
                safety_reported = true;
            }
            this->control.x = 0.0f;
            this->control.y = 0.0f;
            this->control.yaw = 0.0f;
            this->control.SetKeyboard(Input::Keyboard::P);
        }
    }

    this->StateController(&this->robot_state, &this->robot_command);

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
#if defined(USE_ROS1)
        std_srvs::Empty empty;
        this->gazebo_reset_world_client.call(empty);
#elif defined(USE_ROS2)
        auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
#endif
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
#if defined(USE_ROS1)
            std_srvs::Empty empty;
            this->gazebo_pause_physics_client.call(empty);
#elif defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_pause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
#if defined(USE_ROS1)
            std_srvs::Empty empty;
            this->gazebo_unpause_physics_client.call(empty);
#elif defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_unpause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

#if defined(USE_ROS1)
void RL_Sim::ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    this->vel = msg->twist[2];
    this->pose = msg->pose[2];
}
#elif defined(USE_ROS2)
void RL_Sim::GazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> state_lock(this->robot_state_msg_mutex_);
    this->gazebo_imu = *msg;
}

void RL_Sim::ModelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
    const auto model_it = std::find(msg->name.begin(), msg->name.end(), this->gazebo_model_name);
    if (model_it == msg->name.end())
    {
        return;
    }

    const size_t index = static_cast<size_t>(std::distance(msg->name.begin(), model_it));
    if (index >= msg->twist.size())
    {
        return;
    }

    const auto &twist = msg->twist[index];
    if (!std::isfinite(twist.linear.x) ||
        !std::isfinite(twist.linear.y) ||
        !std::isfinite(twist.linear.z))
    {
        return;
    }

    std::lock_guard<std::mutex> state_lock(this->robot_state_msg_mutex_);
    this->model_twist_world_ = twist;
    this->received_model_state_ = true;
}
#endif

void RL_Sim::CmdvelCallback(
#if defined(USE_ROS1)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}

void RL_Sim::JoyCallback(
#if defined(USE_ROS1)
    const sensor_msgs::Joy::ConstPtr &msg
#elif defined(USE_ROS2)
    const sensor_msgs::msg::Joy::SharedPtr msg
#endif
)
{
    this->joy_msg = *msg;

    // joystick control
    // Description of buttons and axes(F710):
    // |__ buttons[]: A=0, B=1, X=2, Y=3, LB=4, RB=5, back=6, start=7, power=8, stickL=9, stickR=10
    // |__ axes[]: Lx=0, Ly=1, Rx=3, Ry=4, LT=2, RT=5, DPadX=6, DPadY=7

    if (this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::A);
    if (this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::B);
    if (this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::X);
    if (this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->joy_msg.buttons[4]) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::LB_RB);

    this->control.x = this->joy_msg.axes[1]; // LY
    this->control.y = this->joy_msg.axes[0]; // LX
    this->control.yaw = this->joy_msg.axes[3]; // RX
}

#if defined(USE_ROS1)
void RL_Sim::JointStatesCallback(const robot_msgs::MotorState::ConstPtr &msg, const std::string &joint_controller_name)
{
    this->joint_positions[joint_controller_name] = msg->q;
    this->joint_velocities[joint_controller_name] = msg->dq;
    this->joint_efforts[joint_controller_name] = msg->tau_est;
}
#elif defined(USE_ROS2)
void RL_Sim::RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg)
{
    if (msg->motor_state.size() != static_cast<std::size_t>(this->params.Get<int>("num_of_dofs")))
    {
        RCLCPP_WARN_THROTTLE(
            this->ros2_node->get_logger(), *this->ros2_node->get_clock(), 1000,
            "Ignoring RobotState with %zu motors; expected %d",
            msg->motor_state.size(), this->params.Get<int>("num_of_dofs"));
        return;
    }

    bool all_positions_zero = true;
    for (const auto &motor : msg->motor_state)
    {
        if (!std::isfinite(motor.q) || !std::isfinite(motor.dq))
        {
            RCLCPP_WARN_THROTTLE(
                this->ros2_node->get_logger(), *this->ros2_node->get_clock(), 1000,
                "Ignoring RobotState containing non-finite joint feedback");
            return;
        }
        if (std::fabs(motor.q) > 1e-6f)
        {
            all_positions_zero = false;
        }
    }

    std::lock_guard<std::mutex> state_lock(this->robot_state_msg_mutex_);
    if (received_valid_robot_state_ && all_positions_zero)
    {
        RCLCPP_WARN_THROTTLE(
            this->ros2_node->get_logger(), *this->ros2_node->get_clock(), 1000,
            "Ignoring all-zero RobotState reset frame");
        return;
    }
    this->robot_state_subscriber_msg = *msg;
    received_valid_robot_state_ = received_valid_robot_state_ || !all_positions_zero;
}
#endif

void RL_Sim::RunModel()
{
    if (this->rl_init_done && simulation_running)
    {
        RobotState<float> state_snapshot;
        state_snapshot.motor_state.resize(this->params.Get<int>("num_of_dofs"));
        this->GetState(&state_snapshot);

        this->episode_length_buf += 1;
        this->obs.ang_vel = state_snapshot.imu.gyroscope;
        // Guard against NaN from IMU
        if (!std::isfinite(state_snapshot.imu.gyroscope[0]) ||
            !std::isfinite(state_snapshot.imu.gyroscope[1]) ||
            !std::isfinite(state_snapshot.imu.gyroscope[2]))
        {
            this->obs.ang_vel = this->last_valid_gyro_;
        }
        else
        {
            this->last_valid_gyro_ = this->obs.ang_vel;
        }
        this->obs.base_quat = state_snapshot.imu.quaternion;
        // Guard against NaN from IMU
        if (!std::isfinite(state_snapshot.imu.quaternion[0]) ||
            !std::isfinite(state_snapshot.imu.quaternion[1]) ||
            !std::isfinite(state_snapshot.imu.quaternion[2]) ||
            !std::isfinite(state_snapshot.imu.quaternion[3]))
        {
            this->obs.base_quat = this->last_valid_quat_;
        }
        else
        {
            this->last_valid_quat_ = this->obs.base_quat;
        }

        std::vector<float> target_commands = {
            std::clamp(this->control.x, -kB2Z1LinearSpeedLimit, kB2Z1LinearSpeedLimit),
            std::clamp(this->control.y, -kB2Z1LateralSpeedLimit, kB2Z1LateralSpeedLimit),
            std::clamp(this->control.yaw, -kB2Z1YawSpeedLimit, kB2Z1YawSpeedLimit)};
        if (this->control.navigation_mode)
        {
            target_commands = {
                std::clamp(static_cast<float>(this->cmd_vel.linear.x),
                           -kB2Z1LinearSpeedLimit, kB2Z1LinearSpeedLimit),
                std::clamp(static_cast<float>(this->cmd_vel.linear.y),
                           -kB2Z1LateralSpeedLimit, kB2Z1LateralSpeedLimit),
                std::clamp(static_cast<float>(this->cmd_vel.angular.z),
                           -kB2Z1YawSpeedLimit, kB2Z1YawSpeedLimit)};
        }
        // Ramp commands at 1.0 units/s. The policy keeps its original joint
        // trajectory timing while avoiding an instantaneous gait switch.
        constexpr float kCommandStep = 0.02f; // 20 ms policy period
        for (size_t i = 0; i < this->obs.commands.size(); ++i)
        {
            this->obs.commands[i] += std::clamp(
                target_commands[i] - this->obs.commands[i],
                -kCommandStep, kCommandStep);
        }
#if defined(USE_ROS2)
        geometry_msgs::msg::Twist model_twist_world;
        bool has_model_state = false;
        {
            std::lock_guard<std::mutex> state_lock(this->robot_state_msg_mutex_);
            model_twist_world = this->model_twist_world_;
            has_model_state = this->received_model_state_;
        }
        if (has_model_state)
        {
            const std::vector<float> linear_velocity_world = {
                static_cast<float>(model_twist_world.linear.x),
                static_cast<float>(model_twist_world.linear.y),
                static_cast<float>(model_twist_world.linear.z)
            };
            this->obs.lin_vel =
                QuatRotateInverse(this->obs.base_quat, linear_velocity_world);
        }
#endif
        this->obs.dof_pos = state_snapshot.motor_state.q;
        this->obs.dof_vel = state_snapshot.motor_state.dq;

        const std::vector<float> policy_actions = this->Forward();
        if (!this->params.Get<bool>("sync_applied_action_observation", false))
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
    for (std::size_t i = 0; i < clamped_obs.size(); ++i)
    {
        if (!std::isfinite(clamped_obs[i]))
        {
            std::cout << std::endl << LOGGER::ERROR
                      << "Non-finite policy observation at index " << i
                      << "; suppressing RL command" << std::endl;
            return std::vector<float>(this->params.Get<int>("num_of_dofs"), 0.0f);
        }
    }

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

    static bool logged_first_inference = false;
    if (!logged_first_inference)
    {
        const auto obs_minmax = std::minmax_element(clamped_obs.begin(), clamped_obs.end());
        const auto action_minmax = std::minmax_element(actions.begin(), actions.end());
        std::cout << std::endl << LOGGER::INFO
                  << "First policy inference: obs_dim=" << clamped_obs.size()
                  << " obs_range=[" << *obs_minmax.first << ", " << *obs_minmax.second << "]"
                  << " action_dim=" << actions.size()
                  << " action_range=[" << *action_minmax.first << ", " << *action_minmax.second << "]"
                  << std::endl << LOGGER::INFO << "First policy actions:";
        for (float action : actions)
        {
            std::cout << " " << action;
        }
        std::cout << std::endl;
        logged_first_inference = true;
    }

    for (std::size_t i = 0; i < actions.size(); ++i)
    {
        if (!std::isfinite(actions[i]))
        {
            std::cout << std::endl << LOGGER::ERROR
                      << "Non-finite policy action at index " << i
                      << "; suppressing RL command" << std::endl;
            return std::vector<float>(this->params.Get<int>("num_of_dofs"), 0.0f);
        }
    }

    // Match FilteredJointPositionAction used during Isaac Lab training. The
    // filtered value is returned to RunModel(), which stores it as the next
    // `actions` observation.
    const float action_filter_alpha =
        this->params.Get<float>("action_filter_alpha", 1.0f);
    if (action_filter_alpha < 1.0f &&
        this->obs.actions.size() == actions.size())
    {
        const float alpha =
            std::clamp(action_filter_alpha, 0.0f, 1.0f);
        for (std::size_t i = 0; i < actions.size(); ++i)
        {
            actions[i] =
                alpha * actions[i] +
                (1.0f - alpha) * this->obs.actions[i];
        }
    }

    // Constrain only Z1 target slew. The applied value is fed back as
    // `last_action`, avoiding a hidden deployment-only filter state.
    const float arm_target_rate_limit =
        this->params.Get<float>("arm_target_rate_limit", 0.0f);
    if (arm_target_rate_limit > 0.0f &&
        this->obs.actions.size() == actions.size())
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
                this->obs.actions[index] - max_action_step,
                this->obs.actions[index] + max_action_step);
        }
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
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
#if defined(USE_ROS1)
        this->plot_real_joint_pos[i].push_back(this->joint_positions[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]]);
        this->plot_target_joint_pos[i].push_back(this->joint_publishers_commands[i].q);
#elif defined(USE_ROS2)
        this->plot_real_joint_pos[i].push_back(this->robot_state_subscriber_msg.motor_state[i].q);
        this->plot_target_joint_pos[i].push_back(this->robot_command_publisher_msg.motor_command[i].q);
#endif
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}

#if defined(USE_ROS1)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Sim rl_sar(argc, argv);
    ros::spin();
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Sim>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
#endif
    return 0;
}
