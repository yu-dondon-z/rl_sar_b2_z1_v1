/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_b2_z1_no_gun.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree_arm_sdk/control/unitreeArm.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <csignal>
#include <limits>
#include <mutex>
#include <net/if.h>
#include <stdexcept>
#include <string>
#include <thread>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

namespace
{
constexpr char kLowCmdTopic[] = "rt/lowcmd";
constexpr char kLowStateTopic[] = "rt/lowstate";
constexpr char kJoystickTopic[] = "rt/wirelesscontroller";
constexpr double kPosStop = 2.146E+9;
constexpr double kVelStop = 16000.0;
constexpr float kRealLinearSpeedLimit = 0.5f;
constexpr float kRealLateralSpeedLimit = 0.4f;
constexpr float kRealYawSpeedLimit = 0.0f;
constexpr float kJoystickDeadzone = 0.05f;
constexpr bool kUseB2TargetVelocityFeedforward = false;
constexpr float kB2LegTargetVelocityLimit = 1.0f;
constexpr float kB2LegTargetVelocityAlpha = 0.15f;

std::atomic<bool> g_shutdown_requested{false};

union KeySwitch
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value = 0;
};

// Raw 40-byte radio packet embedded in B2 LowState. Unlike the separate
// wirelesscontroller topic, this remains available after releasing "ai".
struct RawRemotePacket
{
    uint8_t head[2];
    uint16_t keys;
    float lx;
    float rx;
    float ry;
    float l2;
    float ly;
    uint8_t idle[16];
};
static_assert(sizeof(RawRemotePacket) == 40, "Unexpected remote packet layout");

void SignalHandler(int)
{
    g_shutdown_requested.store(true);
}

float ScaleJoystickAxis(float axis, float limit)
{
    if (!std::isfinite(axis) || std::fabs(axis) <= kJoystickDeadzone)
    {
        return 0.0f;
    }
    const float magnitude =
        (std::min(std::fabs(axis), 1.0f) - kJoystickDeadzone) /
        (1.0f - kJoystickDeadzone);
    return std::copysign(magnitude * limit, axis);
}
} // namespace

class RLRealB2Z1NoGun : public RL
{
public:
    explicit RLRealB2Z1NoGun(const std::string &network_interface, bool enable_motion_switcher,
                          const std::string &z1_ip, uint32_t z1_to_port,
                          uint32_t z1_own_port, bool z1_skip_lowcmd,
                          bool preflight_only, bool disable_z1, float b2_kp_scale,
                          float b2_kd_scale)
    {
        this->ang_vel_axis = "body";
        this->robot_name = "b2_z1_no_gun";
        this->ReadYaml(this->robot_name, "base.yaml");

        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (!fsm_ptr)
        {
            throw std::runtime_error("Failed to create b2_z1_no_gun FSM");
        }
        this->fsm = *fsm_ptr;

        this->InitJointNum(this->params.Get<int>("num_of_dofs"));
        this->InitOutputs();
        this->InitControl();
        if (!std::isfinite(b2_kp_scale) || !std::isfinite(b2_kd_scale) ||
            b2_kp_scale < 0.5f || b2_kp_scale > 6.25f ||
            b2_kd_scale < 0.5f || b2_kd_scale > 3.0f)
        {
            throw std::runtime_error("Invalid B2 gain scale");
        }
        b2_kp_scale_ = b2_kp_scale;
        b2_kd_scale_ = b2_kd_scale;
        this->InitLowCmd();
        ValidateConfiguration();

        if (network_interface.empty() ||
            if_nametoindex(network_interface.c_str()) == 0)
        {
            throw std::runtime_error(
                "Network interface does not exist: " + network_interface);
        }
        ChannelFactory::Instance()->Init(0, network_interface);
        lowcmd_publisher_.reset(
            new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(kLowCmdTopic));
        lowcmd_publisher_->InitChannel();
        lowstate_subscriber_.reset(
            new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(kLowStateTopic));
        lowstate_subscriber_->InitChannel(
            std::bind(&RLRealB2Z1NoGun::LowStateHandler, this, std::placeholders::_1), 1);
        joystick_subscriber_.reset(
            new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(kJoystickTopic));
        joystick_subscriber_->InitChannel(
            std::bind(&RLRealB2Z1NoGun::JoystickHandler, this, std::placeholders::_1), 1);

        if (enable_motion_switcher)
        {
            motion_switcher_ = std::make_unique<MotionSwitcherClient>();
            motion_switcher_->SetTimeout(5.0f);
            motion_switcher_->Init();
        }

        z1_enabled_ = !disable_z1;
        if (z1_enabled_)
        {
            auto *z1_ctrl = new UNITREE_ARM::CtrlComponents();
            z1_ctrl->dt = 0.002;
            z1_ctrl->udp = new UNITREE_ARM::UDPPort(
                z1_ip, z1_to_port, z1_own_port,
                UNITREE_ARM::RECVSTATE_LENGTH, UNITREE_ARM::BlockYN::NO, 500000);
            z1_ctrl->armModel = new UNITREE_ARM::Z1Model();
            z1_ctrl->armModel->addLoad(0.03);
            z1_arm_ = std::make_unique<UNITREE_ARM::unitreeArm>(z1_ctrl);

            std::cout << LOGGER::INFO << "Using thirdparty z1_sdk UDP: "
                      << z1_ip << ":" << z1_to_port
                      << " <- local:" << z1_own_port << std::endl;
            // Do not call backToStart(): startup must never move the arm unexpectedly.
            std::cout << LOGGER::INFO << "Starting Z1 send/recv thread" << std::endl;
            z1_arm_->sendRecvThread->start();
            std::cout << LOGGER::INFO << "Switching Z1 to PASSIVE" << std::endl;
            const bool z1_passive_ok = z1_arm_->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
            std::cout << LOGGER::INFO << "Z1 PASSIVE result: " << z1_passive_ok << std::endl;
            if (!z1_passive_ok)
            {
                throw std::runtime_error("Failed to put Z1 into PASSIVE mode");
            }
        }
        else
        {
            std::cout << LOGGER::WARNING
                      << "Z1 disabled: no Z1 SDK init, no Z1 commands, B2-only test"
                      << std::endl;
        }

        std::cout << LOGGER::INFO
                  << (z1_enabled_ ? "Waiting for initial B2 and Z1 states"
                                  : "Waiting for initial B2 state")
                  << std::endl;
        WaitForInitialState();
        std::cout << LOGGER::INFO
                  << (z1_enabled_ ? "Initial B2 and Z1 states received"
                                  : "Initial B2 state received")
                  << std::endl;
        RobotState<float> initial_state;
        initial_state.motor_state.resize(
            this->params.Get<int>("num_of_dofs"));
        this->GetState(&initial_state);
        std::string initial_state_reason;
        if (!StateIsSafe(initial_state, &initial_state_reason))
        {
            throw std::runtime_error(
                "Initial robot state is unsafe: " + initial_state_reason);
        }
        std::cout << LOGGER::INFO
                  << "B2 real gain scales: kp=" << b2_kp_scale_
                  << " kd=" << b2_kd_scale_ << std::endl;

        if (preflight_only || z1_skip_lowcmd)
        {
            PrintB2Diagnostics();
            if (z1_enabled_)
            {
                PrintZ1Diagnostics();
            }
            diagnostic_only_ = true;
            std::cout << LOGGER::INFO
                      << "Preflight completed: B2 motion service was not released "
                      << "and no B2 low command was published"
                      << std::endl;
            return;
        }

        // Releasing the factory controller is intentionally the final
        // initialization step before starting the 500 Hz low-command writer.
        // A failed B2/Z1 state check therefore cannot leave B2 unmanaged.
        if (motion_switcher_)
        {
            ReleaseBuiltInMotionControl();
        }
        else
        {
            std::cout << LOGGER::WARNING
                      << "Skipping B2 MotionSwitcherClient; low-level takeover "
                      << "must already be exclusive"
                      << std::endl;
        }

        loop_keyboard_ = std::make_shared<LoopFunc>(
            "real_keyboard", 0.05,
            std::bind(&RLRealB2Z1NoGun::KeyboardInterface, this));
        loop_control_ = std::make_shared<LoopFunc>(
            "real_control", this->params.Get<float>("dt"),
            std::bind(&RLRealB2Z1NoGun::RobotControl, this));
        // B2's official low-level examples refresh rt/lowcmd every 2 ms.
        // Keep policy/FSM timing unchanged and retransmit the latest frame at
        // the hardware-required rate.
        loop_b2_writer_ = std::make_shared<LoopFunc>(
            "real_b2_writer", 0.002,
            std::bind(&RLRealB2Z1NoGun::WriteB2LowCmd, this));
        loop_rl_ = std::make_shared<LoopFunc>(
            "real_policy",
            this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
            std::bind(&RLRealB2Z1NoGun::RunModel, this));
        if (motion_switcher_)
        {
            loop_motion_audit_ = std::make_shared<LoopFunc>(
                "real_motion_audit", 1.0,
                std::bind(&RLRealB2Z1NoGun::AuditMotionControl, this));
        }

        loop_keyboard_->start();
        loop_b2_writer_->start();
        loop_control_->start();
        loop_rl_->start();
        if (loop_motion_audit_) loop_motion_audit_->start();

        std::cout << LOGGER::INFO
                  << "B2+Z1 real deployment ready in passive mode. "
                  << "Use a support frame; press 0 to stand."
                  << std::endl;
    }

    ~RLRealB2Z1NoGun()
    {
        if (loop_keyboard_) loop_keyboard_->shutdown();
        if (loop_control_) loop_control_->shutdown();
        if (loop_rl_) loop_rl_->shutdown();
        if (loop_motion_audit_) loop_motion_audit_->shutdown();

        SendPassiveCommands();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (loop_b2_writer_) loop_b2_writer_->shutdown();
        if (z1_arm_)
        {
            z1_arm_->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
            z1_arm_->sendRecvThread->shutdown();
        }
        std::cout << LOGGER::INFO << "B2+Z1 real deployment stopped" << std::endl;
    }

    bool DiagnosticOnly() const
    {
        return diagnostic_only_;
    }

private:
    void WaitForInitialState()
    {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(10);
        auto next_log = std::chrono::steady_clock::now();
        while ((!b2_state_ready_.load() || !Z1StateValid()) &&
               std::chrono::steady_clock::now() < deadline)
        {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_log)
            {
                std::cout << LOGGER::INFO
                          << "Waiting initial state: b2="
                          << b2_state_ready_.load()
                          << " z1=" << Z1StateValid()
                          << std::endl;
                next_log = now + std::chrono::seconds(1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!b2_state_ready_.load())
        {
            throw std::runtime_error("Timed out waiting for B2 low state");
        }
        if (!Z1StateValid())
        {
            throw std::runtime_error("Timed out waiting for Z1 low state");
        }
    }

    void ValidateConfiguration() const
    {
        const size_t num_dofs =
            static_cast<size_t>(this->params.Get<int>("num_of_dofs"));
        const auto b2_mapping = this->params.Get<std::vector<int>>("b2_dds_mapping");
        const auto z1_indices =
            this->params.Get<std::vector<int>>("z1_policy_indices");
        if (num_dofs != 18 || b2_mapping.size() != num_dofs ||
            z1_indices.size() != 6)
        {
            throw std::runtime_error(
                "Invalid B2+Z1 mapping configuration; expected 18 policy joints");
        }
        for (const int index : z1_indices)
        {
            if (index < 0 || index >= static_cast<int>(num_dofs) ||
                b2_mapping[index] != -1)
            {
                throw std::runtime_error("Invalid Z1 policy index mapping");
            }
        }
    }

    void PrintB2Diagnostics()
    {
        unitree_go::msg::dds_::LowState_ state;
        {
            std::lock_guard<std::mutex> lock(b2_state_mutex_);
            state = b2_low_state_;
        }
        std::cout << LOGGER::INFO
                  << "B2 preflight: tick=" << state.tick()
                  << " imu_quat=["
                  << state.imu_state().quaternion()[0] << ", "
                  << state.imu_state().quaternion()[1] << ", "
                  << state.imu_state().quaternion()[2] << ", "
                  << state.imu_state().quaternion()[3] << "]"
                  << std::endl;
        for (int dds_index = 0; dds_index < 12; ++dds_index)
        {
            const auto &motor = state.motor_state().at(dds_index);
            std::cout << LOGGER::INFO
                      << "  B2 motor[" << dds_index << "]"
                      << " mode=" << static_cast<int>(motor.mode())
                      << " q=" << motor.q()
                      << " dq=" << motor.dq()
                      << " lost=" << motor.lost()
                      << std::endl;
        }
    }

    void PrintZ1Diagnostics() const
    {
        std::cout << LOGGER::WARNING
                  << "Z1 LOWCMD switch skipped; printing receive state only"
                  << std::endl;
        for (int sample = 0; sample < 20; ++sample)
        {
            const auto q = z1_arm_->lowstate->getQ();
            const auto dq = z1_arm_->lowstate->getQd();
            std::cout << LOGGER::INFO
                      << "Z1 diag #" << sample
                      << " fsm=" << static_cast<int>(z1_arm_->_ctrlComp->recvState.state)
                      << " q=[" << q[0] << ", " << q[1] << ", " << q[2]
                      << ", " << q[3] << ", " << q[4] << ", " << q[5] << "]"
                      << " dq=[" << dq[0] << ", " << dq[1] << ", " << dq[2]
                      << ", " << dq[3] << ", " << dq[4] << ", " << dq[5] << "]"
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    bool Z1StateValid() const
    {
        if (!z1_enabled_)
        {
            return true;
        }
        const auto q = z1_arm_->lowstate->getQ();
        const auto dq = z1_arm_->lowstate->getQd();
        for (int i = 0; i < 6; ++i)
        {
            if (!std::isfinite(q[i]) || !std::isfinite(dq[i]))
            {
                return false;
            }
        }
        return true;
    }

    void ReleaseBuiltInMotionControl()
    {
        std::string form;
        std::string mode;
        if (motion_switcher_->CheckMode(form, mode) != 0)
        {
            throw std::runtime_error("Failed to query active B2 motion service");
        }
        if (!mode.empty())
        {
            std::cout << LOGGER::WARNING
                      << "Releasing active B2 motion service: " << mode << std::endl;
            if (motion_switcher_->ReleaseMode() != 0)
            {
                throw std::runtime_error("Failed to release B2 motion service");
            }
        }

        for (int attempt = 0; attempt < 20; ++attempt)
        {
            form.clear();
            mode.clear();
            if (motion_switcher_->CheckMode(form, mode) == 0 && mode.empty())
            {
                std::cout << LOGGER::INFO
                          << "B2 motion service release confirmed; "
                          << "low-level control is exclusive"
                          << std::endl;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        throw std::runtime_error(
            "B2 motion service is still active after release; refusing low-level control");
    }

    void InitLowCmd()
    {
        b2_low_command_.head()[0] = 0xFE;
        b2_low_command_.head()[1] = 0xEF;
        b2_low_command_.level_flag() = 0xFF;
        b2_low_command_.gpio() = 0;
        for (auto &motor : b2_low_command_.motor_cmd())
        {
            // B2 low-level PMSM servo mode. Unlike Go2, B2 expects 0x0A.
            motor.mode() = 0x0A;
            motor.q() = kPosStop;
            motor.kp() = 0.0f;
            motor.dq() = kVelStop;
            motor.kd() = 0.0f;
            motor.tau() = 0.0f;
        }
        b2_low_command_.crc() = Crc32Core(
            reinterpret_cast<uint32_t *>(&b2_low_command_),
            (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    }

    void GetState(RobotState<float> *state) override
    {
        unitree_go::msg::dds_::LowState_ b2_state;
        unitree_go::msg::dds_::WirelessController_ joystick;
        {
            std::lock_guard<std::mutex> lock(b2_state_mutex_);
            b2_state = b2_low_state_;
            joystick = joystick_;
        }

        RawRemotePacket raw_remote{};
        std::memcpy(
            &raw_remote, b2_state.wireless_remote().data(), sizeof(raw_remote));
        const bool raw_remote_valid =
            std::isfinite(raw_remote.lx) && std::isfinite(raw_remote.ly) &&
            std::isfinite(raw_remote.rx) &&
            std::fabs(raw_remote.lx) <= 1.5f &&
            std::fabs(raw_remote.ly) <= 1.5f &&
            std::fabs(raw_remote.rx) <= 1.5f;

        const float joystick_lx = raw_remote_valid ? raw_remote.lx : joystick.lx();
        const float joystick_ly = raw_remote_valid ? raw_remote.ly : joystick.ly();
        const float joystick_rx = raw_remote_valid ? raw_remote.rx : joystick.rx();
        key_switch_.value = raw_remote_valid ? raw_remote.keys : joystick.keys();
        if (key_switch_.components.A) this->control.SetGamepad(Input::Gamepad::A);
        if (key_switch_.components.B) this->control.SetGamepad(Input::Gamepad::B);
        if (key_switch_.components.X) this->control.SetGamepad(Input::Gamepad::X);
        if (key_switch_.components.L1 && key_switch_.components.X)
            this->control.SetGamepad(Input::Gamepad::LB_X);
        if (key_switch_.components.R1 && key_switch_.components.up)
            this->control.SetGamepad(Input::Gamepad::RB_DPadUp);

        const bool joystick_active =
            std::fabs(joystick_lx) > kJoystickDeadzone ||
            std::fabs(joystick_ly) > kJoystickDeadzone ||
            std::fabs(joystick_rx) > kJoystickDeadzone;
        if (joystick_active || joystick_was_active_)
        {
            this->control.x = ScaleJoystickAxis(
                joystick_ly, kRealLinearSpeedLimit);
            this->control.y = ScaleJoystickAxis(
                -joystick_lx, kRealLateralSpeedLimit);
            this->control.yaw = ScaleJoystickAxis(
                -joystick_rx, kRealYawSpeedLimit);
            joystick_was_active_ = joystick_active;
        }

        for (int i = 0; i < 4; ++i)
        {
            state->imu.quaternion[i] = b2_state.imu_state().quaternion()[i];
        }
        for (int i = 0; i < 3; ++i)
        {
            state->imu.gyroscope[i] = b2_state.imu_state().gyroscope()[i];
        }

        const auto b2_mapping = this->params.Get<std::vector<int>>("b2_dds_mapping");
        for (int policy_index = 0; policy_index < this->params.Get<int>("num_of_dofs");
             ++policy_index)
        {
            const int dds_index = b2_mapping.at(policy_index);
            if (dds_index >= 0)
            {
                const auto &motor = b2_state.motor_state().at(dds_index);
                state->motor_state.q[policy_index] = motor.q();
                state->motor_state.dq[policy_index] = motor.dq();
                state->motor_state.tau_est[policy_index] = motor.tau_est();
            }
        }

        const auto z1_indices =
            this->params.Get<std::vector<int>>("z1_policy_indices");
        if (!z1_enabled_)
        {
            const auto stand_pos =
                this->params.Get<std::vector<float>>("default_dof_pos") +
                this->params.Get<std::vector<float>>("sim_dof_pos_offset");
            for (const int policy_index : z1_indices)
            {
                state->motor_state.q[policy_index] = stand_pos[policy_index];
                state->motor_state.dq[policy_index] = 0.0f;
                state->motor_state.tau_est[policy_index] = 0.0f;
            }
            return;
        }

        const auto z1_q = z1_arm_->lowstate->getQ();
        const auto z1_dq = z1_arm_->lowstate->getQd();
        const auto z1_tau = z1_arm_->lowstate->getTau();
        for (int joint = 0; joint < 6; ++joint)
        {
            const int policy_index = z1_indices.at(joint);
            state->motor_state.q[policy_index] = z1_q[joint];
            state->motor_state.dq[policy_index] = z1_dq[joint];
            state->motor_state.tau_est[policy_index] = z1_tau[joint];
        }
    }

    void SetCommand(const RobotCommand<float> *command) override
    {
        unitree_go::msg::dds_::LowCmd_ b2_command;
        {
            std::lock_guard<std::mutex> lock(b2_command_mutex_);
            b2_command = b2_low_command_;
        }
        const auto b2_mapping = this->params.Get<std::vector<int>>("b2_dds_mapping");
        const int num_dofs = this->params.Get<int>("num_of_dofs");
        const float control_dt = std::max(this->params.Get<float>("dt"), 1.0e-4f);
        if (previous_b2_target_q_.size() != static_cast<size_t>(num_dofs))
        {
            previous_b2_target_q_.assign(num_dofs, 0.0f);
            previous_b2_target_dq_.assign(num_dofs, 0.0f);
            previous_b2_target_q_valid_ = false;
        }
        for (int policy_index = 0; policy_index < num_dofs; ++policy_index)
        {
            const int dds_index = b2_mapping.at(policy_index);
            if (dds_index < 0)
            {
                continue;
            }
            auto &motor = b2_command.motor_cmd().at(dds_index);
            const float target_q = command->motor_command.q[policy_index];
            float target_dq = command->motor_command.dq[policy_index];
            const float target_kp =
                command->motor_command.kp[policy_index] * b2_kp_scale_;
            const float target_kd =
                command->motor_command.kd[policy_index] * b2_kd_scale_;

            // Unitree low-level PD uses dq_des in the damping term. Sending a
            // moving q target with dq_des=0 makes the real motor controller
            // resist swing-leg motion, while Gazebo's explicit PD does not.
            // Use the target trajectory velocity only after policy handoff
            // gains are active; get-up/stand targets remain pure position PD.
            if (kUseB2TargetVelocityFeedforward && this->rl_init_done &&
                target_kp <= 180.0f && previous_b2_target_q_valid_)
            {
                const float raw_target_dq = std::clamp(
                    (target_q - previous_b2_target_q_[policy_index]) / control_dt,
                    -kB2LegTargetVelocityLimit,
                    kB2LegTargetVelocityLimit);
                target_dq = previous_b2_target_dq_[policy_index] +
                    kB2LegTargetVelocityAlpha *
                    (raw_target_dq - previous_b2_target_dq_[policy_index]);
            }
            else
            {
                target_dq = 0.0f;
            }
            motor.mode() = 0x0A;
            motor.q() = target_q;
            motor.dq() = target_dq;
            motor.kp() = target_kp;
            motor.kd() = target_kd;
            motor.tau() = command->motor_command.tau[policy_index];
            previous_b2_target_q_[policy_index] = target_q;
            previous_b2_target_dq_[policy_index] = target_dq;
        }
        previous_b2_target_q_valid_ = true;
        b2_command.crc() = Crc32Core(
            reinterpret_cast<uint32_t *>(&b2_command),
            (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        {
            std::lock_guard<std::mutex> lock(b2_command_mutex_);
            b2_low_command_ = b2_command;
        }

        if (!z1_enabled_)
        {
            return;
        }

        const auto z1_indices =
            this->params.Get<std::vector<int>>("z1_policy_indices");
        bool arm_active = false;
        for (const int policy_index : z1_indices)
        {
            arm_active =
                arm_active || command->motor_command.kp[policy_index] > 0.0f;
        }
        if (arm_active != z1_gains_active_)
        {
            if (arm_active)
            {
                z1_handoff_q_ = z1_arm_->lowstate->getQ();
                const Vec6 zero = Vec6::Zero();

                // Seed LOWCMD with the measured pose before enabling gains.
                // Otherwise the continuously running SDK thread can briefly
                // transmit its stale/default target during the FSM switch.
                z1_arm_->setArmCmd(z1_handoff_q_, zero, zero);
                auto z1_kp = z1_arm_->_ctrlComp->lowcmd->kp;
                auto z1_kd = z1_arm_->_ctrlComp->lowcmd->kd;
                for (double &gain : z1_kp)
                {
                    gain *= kZ1KpScale;
                }
                z1_arm_->_ctrlComp->lowcmd->setControlGain(z1_kp, z1_kd);
                std::cout << LOGGER::INFO << "Switching Z1 to LOWCMD" << std::endl;
                const bool z1_lowcmd_ok =
                    z1_arm_->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
                std::cout << LOGGER::INFO
                          << "Z1 LOWCMD result: " << z1_lowcmd_ok << std::endl;
                if (!z1_lowcmd_ok)
                {
                    throw std::runtime_error("Failed to put Z1 into LOWCMD mode");
                }
                z1_arm_->setArmCmd(z1_handoff_q_, zero, zero);
                z1_handoff_cycles_ = kZ1HandoffCycles;
                std::cout << LOGGER::INFO
                          << "Z1 LOWCMD seeded at measured pose; kp_scale="
                          << kZ1KpScale << ", starting 5 s handoff"
                          << std::endl;
            }
            else
            {
                z1_arm_->_ctrlComp->lowcmd->setPassive();
                z1_handoff_cycles_ = 0;
            }
            z1_gains_active_ = arm_active;
        }
        if (!arm_active)
        {
            return;
        }

        Vec6 q;
        Vec6 dq;
        Vec6 tau;
        for (int joint = 0; joint < 6; ++joint)
        {
            const int policy_index = z1_indices.at(joint);
            q[joint] = command->motor_command.q[policy_index];
            dq[joint] = command->motor_command.dq[policy_index];
            tau[joint] = command->motor_command.tau[policy_index];
        }
        if (z1_handoff_cycles_ > 0)
        {
            const double blend =
                1.0 - static_cast<double>(z1_handoff_cycles_) /
                          static_cast<double>(kZ1HandoffCycles);
            q = (1.0 - blend) * z1_handoff_q_ + blend * q;
            dq *= blend;
            tau *= blend;
            --z1_handoff_cycles_;
            if (z1_handoff_cycles_ == 0)
            {
                std::cout << LOGGER::INFO << "Z1 LOWCMD handoff completed" << std::endl;
            }
        }
        z1_arm_->setArmCmd(q, dq, tau);
    }

    bool StateIsSafe(const RobotState<float> &state, std::string *reason) const
    {
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            if (!std::isfinite(state.motor_state.q[i]) ||
                !std::isfinite(state.motor_state.dq[i]))
            {
                *reason = "non-finite joint state at policy index " + std::to_string(i);
                return false;
            }
            const float velocity_limit = IsArmPolicyIndex(i)
                ? this->params.Get<float>("arm_safety_velocity_limit", 2.5f)
                : 20.0f;
            if (std::fabs(state.motor_state.dq[i]) > velocity_limit)
            {
                *reason =
                    (IsArmPolicyIndex(i) ? "Z1 joint velocity exceeded at policy index "
                                         : "joint velocity exceeded at policy index ") +
                    std::to_string(i);
                return false;
            }
        }

        const auto euler = QuaternionToEuler(state.imu.quaternion);
        constexpr float kMaximumTilt = 0.785398f; // 45 degrees
        if (std::fabs(euler[0]) > kMaximumTilt ||
            std::fabs(euler[1]) > kMaximumTilt)
        {
            *reason = "body tilt exceeded 45 degrees";
            return false;
        }
        return true;
    }

    void RobotControl()
    {
        if (!b2_state_ready_.load() || !Z1StateValid())
        {
            return;
        }

        this->GetState(&this->robot_state);

        // The policy was trained for the complete B2+Z1 system. Substituting
        // the arm state while leaving its real mass uncontrolled is unsafe.
        if (!z1_enabled_ &&
            (this->control.current_keyboard == Input::Keyboard::Num1 ||
             this->control.current_gamepad == Input::Gamepad::RB_DPadUp))
        {
            std::cout << LOGGER::ERROR
                      << "Locomotion blocked: --disable-z1 is diagnostics-only "
                      << "for this full-body policy" << std::endl;
            this->control.current_keyboard = this->control.last_keyboard;
            this->control.current_gamepad = Input::Gamepad::None;
        }

        std::string unsafe_reason;
        if (!StateIsSafe(this->robot_state, &unsafe_reason))
        {
            if (!safety_stop_.exchange(true))
            {
                std::cout << LOGGER::ERROR
                          << "Real-robot safety stop: " << unsafe_reason << std::endl;
            }
            this->rl_init_done = false;
            this->control.SetKeyboard(Input::Keyboard::P);
        }

        this->StateController(&this->robot_state, &this->robot_command);
        this->control.x = std::clamp(
            this->control.x, -kRealLinearSpeedLimit, kRealLinearSpeedLimit);
        this->control.y = std::clamp(
            this->control.y, -kRealLateralSpeedLimit, kRealLateralSpeedLimit);
        this->control.yaw = std::clamp(
            this->control.yaw, -kRealYawSpeedLimit, kRealYawSpeedLimit);
        this->control.ClearInput();
        this->SetCommand(&this->robot_command);
    }

    void AuditMotionControl()
    {
        if (!motion_switcher_ || safety_stop_.load())
        {
            return;
        }
        std::string form;
        std::string mode;
        const int32_t result = motion_switcher_->CheckMode(form, mode);
        if (result != 0)
        {
            std::cout << LOGGER::WARNING
                      << "Runtime B2 motion-service audit failed: " << result
                      << std::endl;
            return;
        }
        if (!mode.empty())
        {
            std::cout << LOGGER::ERROR
                      << "B2 motion service reacquired control: " << mode
                      << "; requesting passive mode" << std::endl;
            safety_stop_.store(true);
            this->rl_init_done = false;
            this->control.SetKeyboard(Input::Keyboard::P);
        }

        unitree_go::msg::dds_::LowState_ state;
        unitree_go::msg::dds_::LowCmd_ command;
        {
            std::lock_guard<std::mutex> lock(b2_state_mutex_);
            state = b2_low_state_;
        }
        {
            std::lock_guard<std::mutex> lock(b2_command_mutex_);
            command = b2_low_command_;
        }
        ++transport_audit_count_;
        if (transport_audit_count_ % 5 == 0)
        {
            uint32_t max_lost = 0;
            float min_sent_kp = std::numeric_limits<float>::max();
            float max_sent_kp = 0.0f;
            float max_sent_kd = 0.0f;
            float max_sent_dq = 0.0f;
            for (int dds_index = 0; dds_index < 12; ++dds_index)
            {
                max_lost = std::max(max_lost,
                    state.motor_state()[dds_index].lost());
                min_sent_kp = std::min(min_sent_kp,
                    command.motor_cmd()[dds_index].kp());
                max_sent_kp = std::max(max_sent_kp,
                    command.motor_cmd()[dds_index].kp());
                max_sent_kd = std::max(max_sent_kd,
                    command.motor_cmd()[dds_index].kd());
                max_sent_dq = std::max(max_sent_dq,
                    std::fabs(command.motor_cmd()[dds_index].dq()));
            }
            const uint32_t tick_delta = state.tick() - last_audit_tick_;
            last_audit_tick_ = state.tick();
            std::cout << LOGGER::INFO
                      << "B2 transport diagnostic: tick_delta=" << tick_delta
                      << " modes=[";
            for (int dds_index = 0; dds_index < 12; ++dds_index)
            {
                if (dds_index > 0) std::cout << ",";
                std::cout << static_cast<int>(
                    state.motor_state()[dds_index].mode());
            }
            std::cout << "] max_lost=" << max_lost
                      << " sent_kp=[" << min_sent_kp << "," << max_sent_kp << "]"
                      << " sent_kd_max=" << max_sent_kd
                      << " sent_dq_max=" << max_sent_dq
                      << std::endl;
        }
    }

    void WriteB2LowCmd()
    {
        unitree_go::msg::dds_::LowCmd_ command;
        {
            std::lock_guard<std::mutex> lock(b2_command_mutex_);
            command = b2_low_command_;
        }
        if (lowcmd_publisher_)
        {
            lowcmd_publisher_->Write(command);
        }
    }

    void RunModel()
    {
        if (!this->rl_init_done || safety_stop_.load())
        {
            return;
        }

        RobotState<float> state_snapshot;
        state_snapshot.motor_state.resize(this->params.Get<int>("num_of_dofs"));
        this->GetState(&state_snapshot);

        this->episode_length_buf += 1;
        this->obs.ang_vel = state_snapshot.imu.gyroscope;
        std::vector<float> target_commands = {
            std::clamp(this->control.x, -kRealLinearSpeedLimit, kRealLinearSpeedLimit),
            std::clamp(this->control.y, -kRealLateralSpeedLimit, kRealLateralSpeedLimit),
            std::clamp(this->control.yaw, -kRealYawSpeedLimit, kRealYawSpeedLimit)
        };
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
        for (size_t i = 0; i < this->obs.commands.size(); ++i)
        {
            const float current = this->obs.commands[i];
            const float target = target_commands[i];
            const bool accelerating =
                current * target >= 0.0f &&
                std::fabs(target) > std::fabs(current);
            const float rate = accelerating
                ? command_accel_limits[i]
                : command_decel_limits[i];
            const float max_step = std::max(0.0f, rate) * policy_dt;
            this->obs.commands[i] += std::clamp(
                target - current, -max_step, max_step);
        }
        this->obs.base_quat = state_snapshot.imu.quaternion;
        this->obs.dof_pos = state_snapshot.motor_state.q;
        this->obs.dof_vel = state_snapshot.motor_state.dq;

        const std::vector<float> policy_actions = this->Forward();
        if (!this->params.Get<bool>("sync_applied_action_observation", false))
        {
            this->obs.actions = policy_actions;
        }
        this->ComputeOutput(
            policy_actions,
            this->output_dof_pos,
            this->output_dof_vel,
            this->output_dof_tau);
        output_dof_pos_queue.push(this->output_dof_pos);
        output_dof_vel_queue.push(this->output_dof_vel);
        output_dof_tau_queue.push(this->output_dof_tau);
    }

    std::vector<float> Forward() override
    {
        std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);
        if (!lock.owns_lock())
        {
            return this->obs.actions;
        }

        const auto observation = this->ComputeObservation();
        for (const float value : observation)
        {
            if (!std::isfinite(value))
            {
                std::cout << LOGGER::ERROR
                          << "Non-finite real-robot policy observation; "
                          << "requesting passive mode" << std::endl;
                safety_stop_.store(true);
                this->rl_init_done = false;
                this->control.SetKeyboard(Input::Keyboard::P);
                return this->obs.actions;
            }
        }

        std::vector<float> actions;
        if (!this->params.Get<std::vector<int>>("observations_history").empty())
        {
            this->history_obs_buf.insert(observation);
            this->history_obs = this->history_obs_buf.get_obs_vec(
                this->params.Get<std::vector<int>>("observations_history"));
            actions = this->model->forward({this->history_obs});
        }
        else
        {
            actions = this->model->forward({observation});
        }
        for (const float action : actions)
        {
            if (!std::isfinite(action))
            {
                std::cout << LOGGER::ERROR
                          << "Non-finite real-robot policy action; "
                          << "requesting passive mode" << std::endl;
                safety_stop_.store(true);
                this->rl_init_done = false;
                this->control.SetKeyboard(Input::Keyboard::P);
                return this->obs.actions;
            }
        }

        // Match FilteredJointPositionAction used during training. The returned
        // EMA output becomes mdp.last_action on the following policy step.
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

        // Constrain only Z1 target slew. The applied value becomes the next
        // `last_action` observation, keeping the policy loop self-consistent.
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
                const float scale =
                    std::max(std::fabs(action_scale[index]), 1.0e-6f);
                const float max_action_step =
                    arm_target_rate_limit * policy_period / scale;
                actions[index] = std::clamp(
                    actions[index],
                    this->obs.actions[index] - max_action_step,
                    this->obs.actions[index] + max_action_step);
            }
        }

        const auto lower = this->params.Get<std::vector<float>>("clip_actions_lower");
        const auto upper = this->params.Get<std::vector<float>>("clip_actions_upper");
        return (!lower.empty() && !upper.empty())
            ? clamp(actions, lower, upper)
            : actions;
    }

    void SendPassiveCommands()
    {
        RobotCommand<float> passive;
        passive.motor_command.resize(this->params.Get<int>("num_of_dofs"));
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            passive.motor_command.q[i] = this->robot_state.motor_state.q[i];
            passive.motor_command.dq[i] = 0.0f;
            passive.motor_command.kp[i] = 0.0f;
            passive.motor_command.kd[i] = 1.0f;
            passive.motor_command.tau[i] = 0.0f;
        }
        if (lowcmd_publisher_)
        {
            SetCommand(&passive);
        }
        if (z1_arm_)
        {
            z1_arm_->_ctrlComp->lowcmd->setPassive();
        }
        z1_gains_active_ = false;
        previous_b2_target_q_valid_ = false;
        std::fill(previous_b2_target_dq_.begin(), previous_b2_target_dq_.end(), 0.0f);
    }

    static bool IsArmPolicyIndex(int index)
    {
        return index == 8 || index >= 13;
    }

    static uint32_t Crc32Core(uint32_t *ptr, uint32_t length)
    {
        uint32_t crc = 0xFFFFFFFF;
        constexpr uint32_t polynomial = 0x04c11db7;
        for (uint32_t i = 0; i < length; ++i)
        {
            uint32_t data = ptr[i];
            uint32_t xbit = 1U << 31;
            for (int bit = 0; bit < 32; ++bit)
            {
                crc = (crc & 0x80000000U) ? (crc << 1) ^ polynomial : crc << 1;
                if (data & xbit) crc ^= polynomial;
                xbit >>= 1;
            }
        }
        return crc;
    }

    void LowStateHandler(const void *message)
    {
        std::lock_guard<std::mutex> lock(b2_state_mutex_);
        b2_low_state_ =
            *static_cast<const unitree_go::msg::dds_::LowState_ *>(message);
        b2_state_ready_.store(true);
    }

    void JoystickHandler(const void *message)
    {
        std::lock_guard<std::mutex> lock(b2_state_mutex_);
        joystick_ =
            *static_cast<const unitree_go::msg::dds_::WirelessController_ *>(message);
    }

    std::unique_ptr<UNITREE_ARM::unitreeArm> z1_arm_;
    std::unique_ptr<MotionSwitcherClient> motion_switcher_;
    unitree_go::msg::dds_::LowCmd_ b2_low_command_{};
    unitree_go::msg::dds_::LowState_ b2_low_state_{};
    unitree_go::msg::dds_::WirelessController_ joystick_{};
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_subscriber_;
    KeySwitch key_switch_;
    std::mutex b2_state_mutex_;
    std::mutex b2_command_mutex_;
    std::atomic<bool> b2_state_ready_{false};
    std::atomic<bool> safety_stop_{false};
    uint32_t last_audit_tick_ = 0;
    unsigned int transport_audit_count_ = 0;
    bool z1_gains_active_ = false;
    bool z1_enabled_ = true;
    bool diagnostic_only_ = false;
    bool joystick_was_active_ = false;
    bool previous_b2_target_q_valid_ = false;
    float b2_kp_scale_ = 1.0f;
    float b2_kd_scale_ = 1.0f;
    std::vector<float> previous_b2_target_q_;
    std::vector<float> previous_b2_target_dq_;
    static constexpr double kZ1KpScale = 0.35;
    static constexpr int kZ1HandoffCycles = 1000;
    int z1_handoff_cycles_ = 0;
    Vec6 z1_handoff_q_ = Vec6::Zero();
    std::shared_ptr<LoopFunc> loop_keyboard_;
    std::shared_ptr<LoopFunc> loop_control_;
    std::shared_ptr<LoopFunc> loop_b2_writer_;
    std::shared_ptr<LoopFunc> loop_rl_;
    std::shared_ptr<LoopFunc> loop_motion_audit_;
};

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0]
                  << " network_interface [--z1-ip IP] [--z1-to-port PORT]"
                  << " [--z1-own-port PORT] [--z1-skip-lowcmd]"
                  << " [--preflight] [--disable-z1] [--skip-motion-switcher]"
                  << " [--b2-kp-scale SCALE] [--b2-kd-scale SCALE]"
                  << std::endl;
        return 1;
    }

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);
    try
    {
        // Real low-level control must own rt/lowcmd exclusively. Releasing the
        // built-in service is therefore the safe default.
        bool enable_motion_switcher = true;
        std::string z1_ip = "127.0.0.1";
        uint32_t z1_to_port = 8071;
        uint32_t z1_own_port = 8072;
        bool z1_skip_lowcmd = false;
        bool preflight_only = false;
        bool disable_z1 = false;
        float b2_kp_scale = 1.0f;
        float b2_kd_scale = 1.0f;
        for (int i = 2; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "--enable-motion-switcher")
            {
                enable_motion_switcher = true;
            }
            else if (arg == "--skip-motion-switcher")
            {
                enable_motion_switcher = false;
            }
            else if (arg == "--disable-z1")
            {
                disable_z1 = true;
            }
            else if (arg == "--z1-ip" && i + 1 < argc)
            {
                z1_ip = argv[++i];
            }
            else if (arg == "--z1-to-port" && i + 1 < argc)
            {
                z1_to_port = static_cast<uint32_t>(std::stoul(argv[++i]));
            }
            else if (arg == "--z1-own-port" && i + 1 < argc)
            {
                z1_own_port = static_cast<uint32_t>(std::stoul(argv[++i]));
            }
            else if (arg == "--z1-skip-lowcmd")
            {
                z1_skip_lowcmd = true;
            }
            else if (arg == "--preflight")
            {
                preflight_only = true;
            }
            else if (arg == "--b2-kp-scale" && i + 1 < argc)
            {
                b2_kp_scale = std::stof(argv[++i]);
            }
            else if (arg == "--b2-kd-scale" && i + 1 < argc)
            {
                b2_kd_scale = std::stof(argv[++i]);
            }
            else
            {
                throw std::runtime_error("Unknown or incomplete argument: " + arg);
            }
        }
        RLRealB2Z1NoGun robot(
            argv[1], enable_motion_switcher, z1_ip, z1_to_port, z1_own_port,
            z1_skip_lowcmd, preflight_only, disable_z1,
            b2_kp_scale, b2_kd_scale);
        if (robot.DiagnosticOnly())
        {
            return 0;
        }
        while (!g_shutdown_requested.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (const std::exception &error)
    {
        std::cerr << LOGGER::ERROR << error.what() << std::endl;
        return 1;
    }
    return 0;
}
