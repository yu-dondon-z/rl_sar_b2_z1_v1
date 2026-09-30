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
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree_arm_sdk/control/unitreeArm.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <net/if.h>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

namespace
{
constexpr char kLowCmdTopic[] = "rt/lowcmd";
constexpr char kLowStateTopic[] = "rt/lowstate";
constexpr char kSportModeStateTopic[] = "rt/sportmodestate";
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

struct RealVisionPoint
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double s = 0.0;
};

struct TimedBasePose
{
    std::uint64_t monotonic_ns = 0;
    std::array<double, 3> position{{0.0, 0.0, 0.0}};
    std::array<double, 4> quaternion{{1.0, 0.0, 0.0, 0.0}};
};

std::uint64_t SteadyNowNs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool NormalizeQuaternion(std::array<double, 4> *quaternion)
{
    double norm_squared = 0.0;
    for (const double value : *quaternion)
    {
        if (!std::isfinite(value)) return false;
        norm_squared += value * value;
    }
    if (norm_squared < 1.0e-12) return false;
    const double inverse_norm = 1.0 / std::sqrt(norm_squared);
    for (double &value : *quaternion) value *= inverse_norm;
    return true;
}

std::array<double, 3> RotateByQuaternion(
    const std::array<double, 4> &q, const std::array<double, 3> &point)
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return {{
        (1.0 - 2.0 * (y * y + z * z)) * point[0] +
            2.0 * (x * y - z * w) * point[1] +
            2.0 * (x * z + y * w) * point[2],
        2.0 * (x * y + z * w) * point[0] +
            (1.0 - 2.0 * (x * x + z * z)) * point[1] +
            2.0 * (y * z - x * w) * point[2],
        2.0 * (x * z - y * w) * point[0] +
            2.0 * (y * z + x * w) * point[1] +
            (1.0 - 2.0 * (x * x + y * y)) * point[2]
    }};
}

std::array<double, 3> RotateByQuaternionInverse(
    const std::array<double, 4> &q, const std::array<double, 3> &point)
{
    return RotateByQuaternion({{q[0], -q[1], -q[2], -q[3]}}, point);
}

double WrapAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

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
                          float b2_kd_scale, bool enable_real_vision,
                          const std::string &vision_exchange_dir)
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
        sport_state_subscriber_.reset(
            new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
                kSportModeStateTopic));
        sport_state_subscriber_->InitChannel(
            std::bind(&RLRealB2Z1NoGun::SportStateHandler, this,
                      std::placeholders::_1), 1);
        joystick_subscriber_.reset(
            new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(kJoystickTopic));
        joystick_subscriber_->InitChannel(
            std::bind(&RLRealB2Z1NoGun::JoystickHandler, this, std::placeholders::_1), 1);

        real_vision_enabled_ = enable_real_vision;
        real_vision_exchange_dir_ = vision_exchange_dir;
        if (real_vision_enabled_)
        {
            std::error_code error;
            std::filesystem::create_directories(real_vision_exchange_dir_, error);
            if (error)
            {
                throw std::runtime_error(
                    "Cannot create real-vision exchange directory: " +
                    real_vision_exchange_dir_ + ": " + error.message());
            }
            std::cout << LOGGER::INFO
                      << "Real RGB-D trajectory input enabled: "
                      << real_vision_exchange_dir_ << std::endl;
        }

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
        if (real_vision_enabled_)
        {
            // rt/sportmodestate belongs to B2's factory motion service and
            // stops updating after low-level takeover.  Real visual tracking
            // therefore uses the low-state IMU orientation plus stance-foot
            // kinematics, both of which remain available under rt/lowcmd.
            std::cout << LOGGER::INFO
                      << "B2 visual odometry source: LowState IMU + stance-foot "
                         "kinematics"
                      << std::endl;
        }
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
                  << "Use a support frame; press 0 or remote A to stand."
                  << std::endl;
        std::cout << LOGGER::NOTE
                  << "Remote: A=get up, Y=locomotion, B=get down, "
                  << "R1+Y=visual trajectory, X=zero velocity, "
                  << "L1+X=emergency passive; "
                  << "left stick=forward/lateral."
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

    void UpdateRealSupportObservations(
        const unitree_go::msg::dds_::LowState_ &b2_state,
        const RobotState<float> &state)
    {
        // B2 publishes only the raw bridge signals on this robot;
        // foot_force_est remains zero.  The four bridges also have different
        // signs and non-zero offsets, so calibrate each DDS leg independently.
        // DDS leg order is FR, FL, RR, RL.  Policy/contact order is
        // FL, FR, RL, RR.
        const auto zero = this->params.Get<std::vector<float>>(
            "real_foot_force_zero_dds");
        const auto direction = this->params.Get<std::vector<float>>(
            "real_foot_force_direction_dds");
        const auto contact_on = this->params.Get<std::vector<float>>(
            "real_foot_force_contact_on_delta_dds");
        const auto contact_off = this->params.Get<std::vector<float>>(
            "real_foot_force_contact_off_delta_dds");
        if (zero.size() != 4 || direction.size() != 4 ||
            contact_on.size() != 4 || contact_off.size() != 4)
        {
            throw std::runtime_error(
                "Real B2 foot-force calibration must contain four values");
        }

        constexpr std::array<int, 4> dds_to_policy{{1, 0, 3, 2}};
        for (int dds_leg = 0; dds_leg < 4; ++dds_leg)
        {
            const float signal = direction[dds_leg] *
                (static_cast<float>(b2_state.foot_force()[dds_leg]) -
                 zero[dds_leg]);
            const int policy_leg = dds_to_policy[dds_leg];
            if (real_foot_contact_latched_[policy_leg])
            {
                real_foot_contact_latched_[policy_leg] =
                    signal >= contact_off[dds_leg];
            }
            else
            {
                real_foot_contact_latched_[policy_leg] =
                    signal >= contact_on[dds_leg];
            }
        }
        this->obs.foot_contacts.assign(4, 0);
        for (int policy_leg = 0; policy_leg < 4; ++policy_leg)
        {
            this->obs.foot_contacts[policy_leg] =
                real_foot_contact_latched_[policy_leg] ? 1 : 0;
        }

        // LowState contains no floating-base height.  Estimate the hip height
        // from B2's two 0.35 m leg links and the measured joint configuration.
        // During a support check, average only force-confirmed stance legs.
        // If no bridge is currently latched, use the median leg extension for
        // diagnostics without claiming foot contact.
        const float thigh_length = this->params.Get<float>(
            "real_b2_thigh_length", 0.35f);
        const float calf_length = this->params.Get<float>(
            "real_b2_calf_length", 0.35f);
        const float foot_radius = this->params.Get<float>(
            "real_b2_foot_radius", 0.032f);
        const float height_bias = this->params.Get<float>(
            "real_base_height_bias", 0.0f);
        const int thigh_indices[] = {4, 5, 6, 7};
        const int calf_indices[] = {9, 10, 11, 12};
        std::array<float, 4> leg_heights{};
        float contact_height_sum = 0.0f;
        int contact_count = 0;
        for (int leg = 0; leg < 4; ++leg)
        {
            const float hip = state.motor_state.q[leg];
            const float thigh = state.motor_state.q[thigh_indices[leg]];
            const float calf = state.motor_state.q[calf_indices[leg]];
            const float sagittal_extension =
                thigh_length * std::cos(thigh) +
                calf_length * std::cos(thigh + calf);
            leg_heights[leg] = std::max(
                0.0f,
                std::cos(hip) * sagittal_extension + foot_radius + height_bias);
            if (real_foot_contact_latched_[leg])
            {
                contact_height_sum += leg_heights[leg];
                ++contact_count;
            }
        }
        if (contact_count > 0)
        {
            this->obs.base_height = contact_height_sum /
                static_cast<float>(contact_count);
        }
        else
        {
            std::sort(leg_heights.begin(), leg_heights.end());
            this->obs.base_height =
                0.5f * (leg_heights[1] + leg_heights[2]);
        }
        // LowState has no body/knee collision channels.  Height, tilt, all
        // four calibrated foot contacts and joint speed form the real stop
        // guard; never fabricate a positive non-foot contact here.
        this->obs.undesired_ground_contact = false;
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

        KeySwitch previous_key_switch{};
        previous_key_switch.value = previous_remote_keys_;
        const bool passive_combo =
            key_switch_.components.L1 && key_switch_.components.X;
        const bool previous_passive_combo =
            previous_key_switch.components.L1 && previous_key_switch.components.X;
        const bool legacy_locomotion_combo =
            key_switch_.components.R1 && key_switch_.components.up;
        const bool previous_legacy_locomotion_combo =
            previous_key_switch.components.R1 && previous_key_switch.components.up;
        const bool vision_combo =
            key_switch_.components.R1 && key_switch_.components.Y;
        const bool previous_vision_combo =
            previous_key_switch.components.R1 && previous_key_switch.components.Y;
        const bool stop_button_pressed =
            key_switch_.components.X && !previous_key_switch.components.X &&
            !key_switch_.components.L1;

        // Use rising edges so a held radio button cannot repeatedly retrigger an
        // FSM transition.  Safety-related commands have the highest priority.
        if (passive_combo && !previous_passive_combo)
        {
            this->control.SetGamepad(Input::Gamepad::LB_X);
            std::cout << std::endl << LOGGER::WARNING
                      << "[Remote] L1+X -> emergency passive" << std::endl;
        }
        else if (key_switch_.components.B && !previous_key_switch.components.B)
        {
            this->control.SetGamepad(Input::Gamepad::B);
            std::cout << std::endl << LOGGER::INFO
                      << "[Remote] B -> get down" << std::endl;
        }
        else if (key_switch_.components.A && !previous_key_switch.components.A)
        {
            this->control.SetGamepad(Input::Gamepad::A);
            std::cout << std::endl << LOGGER::INFO
                      << "[Remote] A -> get up" << std::endl;
        }
        else if (vision_combo && !previous_vision_combo)
        {
            this->control.SetKeyboard(Input::Keyboard::T);
            std::cout << std::endl << LOGGER::INFO
                      << "[Remote] R1+Y -> toggle visual trajectory"
                      << std::endl;
        }
        else if ((key_switch_.components.Y && !previous_key_switch.components.Y &&
                  !key_switch_.components.R1) ||
                 (legacy_locomotion_combo &&
                  !previous_legacy_locomotion_combo))
        {
            this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
            std::cout << std::endl << LOGGER::INFO
                      << "[Remote] "
                      << (key_switch_.components.Y ? "Y" : "R1+DPadUp")
                      << " -> locomotion policy" << std::endl;
        }

        const bool joystick_active =
            std::fabs(joystick_lx) > kJoystickDeadzone ||
            std::fabs(joystick_ly) > kJoystickDeadzone ||
            std::fabs(joystick_rx) > kJoystickDeadzone;

        if (stop_button_pressed)
        {
            remote_velocity_stop_latched_ = true;
            this->control.SetKeyboard(Input::Keyboard::Space);
            std::cout << std::endl << LOGGER::INFO
                      << "[Remote] X -> zero velocity; centre sticks to re-arm"
                      << std::endl;
        }

        if (remote_velocity_stop_latched_)
        {
            this->control.x = 0.0f;
            this->control.y = 0.0f;
            this->control.yaw = 0.0f;
            joystick_was_active_ = joystick_active;
            if (!joystick_active)
            {
                remote_velocity_stop_latched_ = false;
            }
        }
        else if (joystick_active || joystick_was_active_)
        {
            this->control.x = ScaleJoystickAxis(
                joystick_ly, kRealLinearSpeedLimit);
            this->control.y = ScaleJoystickAxis(
                -joystick_lx, kRealLateralSpeedLimit);
            this->control.yaw = ScaleJoystickAxis(
                -joystick_rx, kRealYawSpeedLimit);
            joystick_was_active_ = joystick_active;
        }
        previous_remote_keys_ = key_switch_.value;

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

        // RobotControl is the sole 200 Hz writer of the support observations.
        // RunModel also obtains a private state snapshot, but must not advance
        // the contact hysteresis independently at 50 Hz.
        if (state == &this->robot_state)
        {
            UpdateRealSupportObservations(b2_state, *state);
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
                const bool z1_already_lowcmd =
                    z1_arm_->_ctrlComp->recvState.state ==
                    UNITREE_ARM::ArmFSMState::LOWCMD;
                std::cout << LOGGER::INFO
                          << (z1_already_lowcmd
                                  ? "Z1 already in LOWCMD; restoring active gains"
                                  : "Switching Z1 to LOWCMD")
                          << std::endl;
                const bool z1_lowcmd_ok = z1_already_lowcmd ||
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

    bool GetLatestBasePose(TimedBasePose *pose) const
    {
        std::lock_guard<std::mutex> lock(sport_state_mutex_);
        if (base_pose_history_.empty()) return false;
        *pose = base_pose_history_.back();
        const std::uint64_t now = SteadyNowNs();
        const std::uint64_t maximum_age_ns = static_cast<std::uint64_t>(
            1.0e9 * this->params.Get<double>(
                "real_vision_odometry_stale_timeout", 0.25));
        return now >= pose->monotonic_ns &&
               now - pose->monotonic_ns <= maximum_age_ns;
    }

    std::array<double, 3> FootPositionInBase(
        const RobotState<float> &state, int leg) const
    {
        constexpr std::array<double, 4> hip_x{{
            0.3285, 0.3285, -0.3285, -0.3285}};
        constexpr std::array<double, 4> side{{1.0, -1.0, 1.0, -1.0}};
        constexpr double hip_y = 0.072;
        constexpr double hip_link = 0.11973;
        const double thigh_length = this->params.Get<double>(
            "real_b2_thigh_length", 0.35);
        const double calf_length = this->params.Get<double>(
            "real_b2_calf_length", 0.35);
        const double hip = state.motor_state.q.at(leg);
        const double thigh = state.motor_state.q.at(4 + leg);
        const double calf = state.motor_state.q.at(9 + leg);

        // B2 leg chain from the deployed MJCF: hip rotates about +X, while
        // thigh and calf rotate about +Y.  The tiny sub-millimetre lateral
        // mesh offsets are intentionally omitted from odometry.
        const double sagittal_x =
            -thigh_length * std::sin(thigh) -
            calf_length * std::sin(thigh + calf);
        const double sagittal_z =
            -thigh_length * std::cos(thigh) -
            calf_length * std::cos(thigh + calf);
        const double lateral = side[leg] * hip_link;
        const double cos_hip = std::cos(hip);
        const double sin_hip = std::sin(hip);
        return {{
            hip_x[leg] + sagittal_x,
            side[leg] * hip_y + cos_hip * lateral - sin_hip * sagittal_z,
            sin_hip * lateral + cos_hip * sagittal_z}};
    }

    void UpdateProprioceptiveOdometry(const RobotState<float> &state)
    {
        TimedBasePose pose;
        pose.monotonic_ns = SteadyNowNs();
        for (int axis = 0; axis < 4; ++axis)
            pose.quaternion[axis] = state.imu.quaternion[axis];
        if (!NormalizeQuaternion(&pose.quaternion)) return;

        std::array<std::array<double, 3>, 4> feet_base{};
        for (int leg = 0; leg < 4; ++leg)
            feet_base[leg] = FootPositionInBase(state, leg);

        std::lock_guard<std::mutex> lock(sport_state_mutex_);
        if (!proprio_odometry_initialized_)
        {
            // Seed x/y from the final factory-odometry sample when available;
            // it is only an origin choice. All later motion comes from IMU and
            // stance-foot constraints rather than the stale factory topic.
            if (!sport_pose_history_.empty())
                pose.position = sport_pose_history_.back().position;
            pose.position[2] = std::max(
                0.0, static_cast<double>(this->obs.base_height));
            proprio_odometry_initialized_ = true;
        }
        else
        {
            pose.position = base_pose_history_.back().position;
            std::vector<double> candidate_x;
            std::vector<double> candidate_y;
            for (int leg = 0; leg < 4; ++leg)
            {
                if (!real_foot_contact_latched_[leg] ||
                    !foot_anchor_valid_[leg])
                    continue;
                const auto rotated = RotateByQuaternion(
                    pose.quaternion, feet_base[leg]);
                candidate_x.push_back(
                    foot_anchors_odom_[leg][0] - rotated[0]);
                candidate_y.push_back(
                    foot_anchors_odom_[leg][1] - rotated[1]);
            }
            if (!candidate_x.empty())
            {
                const double estimate_x = std::accumulate(
                    candidate_x.begin(), candidate_x.end(), 0.0) /
                    static_cast<double>(candidate_x.size());
                const double estimate_y = std::accumulate(
                    candidate_y.begin(), candidate_y.end(), 0.0) /
                    static_cast<double>(candidate_y.size());
                const auto &previous = base_pose_history_.back();
                const double dt = std::max(
                    1.0e-4,
                    static_cast<double>(pose.monotonic_ns -
                                        previous.monotonic_ns) * 1.0e-9);
                const double maximum_step =
                    this->params.Get<double>(
                        "real_proprio_odometry_max_planar_speed", 1.5) * dt;
                double dx = estimate_x - previous.position[0];
                double dy = estimate_y - previous.position[1];
                const double step = std::hypot(dx, dy);
                if (step > maximum_step && step > 1.0e-9)
                {
                    dx *= maximum_step / step;
                    dy *= maximum_step / step;
                }
                const double alpha = std::clamp(
                    this->params.Get<double>(
                        "real_proprio_odometry_position_filter_alpha", 0.35),
                    0.0, 1.0);
                pose.position[0] = previous.position[0] + alpha * dx;
                pose.position[1] = previous.position[1] + alpha * dy;
            }
            pose.position[2] = std::max(
                0.0, static_cast<double>(this->obs.base_height));
        }

        // Establish a world anchor on touchdown and retain it only while the
        // calibrated bridge remains in contact. This makes stance feet, not
        // integrated accelerometer noise, constrain planar translation.
        for (int leg = 0; leg < 4; ++leg)
        {
            if (!real_foot_contact_latched_[leg])
            {
                foot_anchor_valid_[leg] = false;
                continue;
            }
            if (!foot_anchor_valid_[leg])
            {
                const auto rotated = RotateByQuaternion(
                    pose.quaternion, feet_base[leg]);
                foot_anchors_odom_[leg] = {{
                    pose.position[0] + rotated[0],
                    pose.position[1] + rotated[1],
                    pose.position[2] + rotated[2]}};
                foot_anchor_valid_[leg] = true;
            }
        }

        base_pose_history_.push_back(pose);
        const std::uint64_t history_ns = static_cast<std::uint64_t>(
            1.0e9 * this->params.Get<double>(
                "real_vision_pose_history_duration", 3.0));
        while (base_pose_history_.size() > 2 &&
               pose.monotonic_ns - base_pose_history_.front().monotonic_ns >
                   history_ns)
            base_pose_history_.pop_front();
    }

    bool GetBasePoseAt(std::uint64_t timestamp_ns, TimedBasePose *pose) const
    {
        std::lock_guard<std::mutex> lock(sport_state_mutex_);
        if (base_pose_history_.empty()) return false;
        auto closest = base_pose_history_.begin();
        std::uint64_t closest_error = std::numeric_limits<std::uint64_t>::max();
        for (auto iterator = base_pose_history_.begin();
             iterator != base_pose_history_.end(); ++iterator)
        {
            const std::uint64_t error = iterator->monotonic_ns > timestamp_ns
                ? iterator->monotonic_ns - timestamp_ns
                : timestamp_ns - iterator->monotonic_ns;
            if (error < closest_error)
            {
                closest = iterator;
                closest_error = error;
            }
        }
        const std::uint64_t maximum_error_ns = static_cast<std::uint64_t>(
            1.0e9 * this->params.Get<double>(
                "real_vision_maximum_sync_error", 0.15));
        if (closest_error > maximum_error_ns) return false;
        *pose = *closest;
        return true;
    }

    static RealVisionPoint BaseToOdom(
        const TimedBasePose &pose, const RealVisionPoint &point)
    {
        const auto rotated = RotateByQuaternion(
            pose.quaternion, {{point.x, point.y, point.z}});
        return {pose.position[0] + rotated[0],
                pose.position[1] + rotated[1],
                pose.position[2] + rotated[2], point.s};
    }

    static RealVisionPoint OdomToBase(
        const TimedBasePose &pose, const RealVisionPoint &point)
    {
        const auto rotated = RotateByQuaternionInverse(
            pose.quaternion,
            {{point.x - pose.position[0], point.y - pose.position[1],
              point.z - pose.position[2]}});
        return {rotated[0], rotated[1], rotated[2], point.s};
    }

    static void RecomputeArcLength(std::vector<RealVisionPoint> *points)
    {
        if (points->empty()) return;
        (*points)[0].s = 0.0;
        for (std::size_t index = 1; index < points->size(); ++index)
        {
            const auto &previous = (*points)[index - 1];
            auto &point = (*points)[index];
            point.s = previous.s + std::hypot(
                point.x - previous.x, point.y - previous.y);
        }
    }

    void IngestRealVisionTrajectory()
    {
        if (!real_vision_enabled_) return;
        const std::filesystem::path path =
            std::filesystem::path(real_vision_exchange_dir_) /
            "latest_trajectory.json";
        std::error_code error;
        const auto modified = std::filesystem::last_write_time(path, error);
        if (!error && (!real_vision_mtime_valid_ ||
                       modified != real_vision_mtime_))
        {
            try
            {
                const YAML::Node root = YAML::LoadFile(path.string());
                if (root["frame_id"].as<std::string>("") != "b2_base")
                {
                    throw std::runtime_error(
                        "trajectory frame_id must be b2_base");
                }
                const std::uint64_t capture_ns =
                    root["capture_monotonic_ns"].as<std::uint64_t>(0);
                if (capture_ns == 0)
                {
                    throw std::runtime_error(
                        "trajectory has no capture_monotonic_ns");
                }
                TimedBasePose capture_pose;
                if (!GetBasePoseAt(capture_ns, &capture_pose))
                {
                    throw std::runtime_error(
                        "no synchronized proprioceptive B2 pose for camera frame");
                }
                const YAML::Node waypoints =
                    root["control_waypoints_robot_xyz"];
                if (!waypoints || !waypoints.IsSequence())
                {
                    throw std::runtime_error("trajectory contains no waypoints");
                }
                std::vector<RealVisionPoint> points_base;
                for (const auto &node : waypoints)
                {
                    if (!node.IsSequence() || node.size() < 3) continue;
                    RealVisionPoint point{node[0].as<double>(),
                                          node[1].as<double>(),
                                          node[2].as<double>(), 0.0};
                    if (std::isfinite(point.x) && std::isfinite(point.y) &&
                        std::isfinite(point.z) && point.x > -0.20 &&
                        point.x < 8.0 && std::fabs(point.y) < 3.0)
                    {
                        points_base.push_back(point);
                    }
                }
                if (points_base.size() < 3)
                    throw std::runtime_error("fewer than three valid waypoints");
                if (points_base.front().x > points_base.back().x)
                    std::reverse(points_base.begin(), points_base.end());

                std::vector<RealVisionPoint> points_odom;
                points_odom.reserve(points_base.size());
                for (const auto &point : points_base)
                    points_odom.push_back(BaseToOdom(capture_pose, point));
                RecomputeArcLength(&points_odom);
                if (points_odom.back().s < this->params.Get<double>(
                        "real_vision_minimum_path_length", 0.30))
                {
                    throw std::runtime_error("metric trajectory is too short");
                }

                live_trajectory_odom_ = points_odom;
                // Preserve already traversed crack points for the rear-mounted
                // tool. Join the refreshed front-camera path at its nearest
                // previously cached point instead of replacing history.
                if (trajectory_history_odom_.empty())
                {
                    trajectory_history_odom_ = points_odom;
                }
                else
                {
                    std::size_t join_index = 0;
                    double join_distance = std::numeric_limits<double>::infinity();
                    for (std::size_t index = 0;
                         index < trajectory_history_odom_.size(); ++index)
                    {
                        const double distance = std::hypot(
                            trajectory_history_odom_[index].x - points_odom.front().x,
                            trajectory_history_odom_[index].y - points_odom.front().y);
                        if (distance < join_distance)
                        {
                            join_distance = distance;
                            join_index = index;
                        }
                    }
                    if (join_distance > 0.60)
                    {
                        trajectory_history_odom_ = points_odom;
                    }
                    else
                    {
                        trajectory_history_odom_.resize(join_index + 1);
                        for (const auto &point : points_odom)
                        {
                            const auto &last = trajectory_history_odom_.back();
                            if (std::hypot(point.x - last.x,
                                           point.y - last.y) >= 0.025)
                                trajectory_history_odom_.push_back(point);
                        }
                    }
                }
                RecomputeArcLength(&trajectory_history_odom_);
                real_vision_last_received_ = std::chrono::steady_clock::now();
                real_vision_fresh_ = true;
                real_vision_mtime_ = modified;
                real_vision_mtime_valid_ = true;
            }
            catch (const std::exception &exception)
            {
                if (SteadyNowNs() - real_vision_last_parse_warning_ns_ >
                    1000000000ULL)
                {
                    std::cout << LOGGER::WARNING
                              << "[RealVision] rejected frame: "
                              << exception.what() << std::endl;
                    real_vision_last_parse_warning_ns_ = SteadyNowNs();
                }
            }
        }
        if (real_vision_fresh_)
        {
            const double age = std::chrono::duration<double>(
                std::chrono::steady_clock::now() -
                real_vision_last_received_).count();
            if (age > this->params.Get<double>(
                    "real_vision_trajectory_stale_timeout", 1.0))
                real_vision_fresh_ = false;
        }
    }

    RealVisionPoint SampleRealTrajectory(
        const std::vector<RealVisionPoint> &points, double arc_length) const
    {
        if (points.empty()) return {};
        arc_length = std::clamp(arc_length, 0.0, points.back().s);
        const auto upper = std::lower_bound(
            points.begin(), points.end(), arc_length,
            [](const RealVisionPoint &point, double value)
            { return point.s < value; });
        if (upper == points.begin()) return *upper;
        if (upper == points.end()) return points.back();
        const auto &end = *upper;
        const auto &start = *(upper - 1);
        const double denominator = std::max(end.s - start.s, 1.0e-9);
        const double blend = (arc_length - start.s) / denominator;
        return {start.x + blend * (end.x - start.x),
                start.y + blend * (end.y - start.y),
                start.z + blend * (end.z - start.z), arc_length};
    }

    void StopRealVision(const std::string &reason)
    {
        if (!real_vision_active_) return;
        real_vision_active_ = false;
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->ClearRuntimeTargetCommands();
        std::cout << LOGGER::INFO << "[RealVision] stopped: " << reason
                  << std::endl;
    }

    void ToggleRealVision()
    {
        if (!real_vision_enabled_) return;
        if (real_vision_active_)
        {
            StopRealVision("operator toggle");
            return;
        }
        TimedBasePose pose;
        const bool in_locomotion =
            this->fsm.current_state_ &&
            this->fsm.current_state_->GetStateName() ==
                "RLFSMStateRLLocomotion";
        if (!in_locomotion)
        {
            std::cout << LOGGER::WARNING
                      << "[RealVision] enter locomotion with Y (or key 1) "
                      << "before enabling trajectory mode"
                      << std::endl;
            return;
        }
        if (!this->locomotion_ready_for_motion)
        {
            std::cout << LOGGER::WARNING
                      << "[RealVision] locomotion policy is still warming up; "
                      << "wait for the ready message, then toggle again"
                      << std::endl;
            return;
        }
        if (!GetLatestBasePose(&pose))
        {
            std::cout << LOGGER::WARNING
                      << "[RealVision] no fresh IMU/stance-foot odometry; "
                      << "trajectory mode remains disabled" << std::endl;
            return;
        }
        if (!real_vision_fresh_ || live_trajectory_odom_.size() < 3)
        {
            std::cout << LOGGER::WARNING
                      << "[RealVision] no fresh synchronized RGB-D trajectory"
                      << std::endl;
            return;
        }
        real_vision_active_ = true;
        real_vision_start_time_ = std::chrono::steady_clock::now();
        real_target_initialized_ = false;
        std::cout << LOGGER::INFO
                  << "[RealVision] odometry-compensated crack following started"
                  << std::endl;
    }

    void UpdateRealVisionControl()
    {
        if (!real_vision_active_) return;
        TimedBasePose pose;
        if (!GetLatestBasePose(&pose))
        {
            StopRealVision("B2 proprioceptive odometry became stale");
            return;
        }
        if (!real_vision_fresh_)
        {
            const float retain = std::max(
                0.0f, real_vision_stale_command_retain_ - 0.005f /
                    std::max(0.5f, this->params.Get<float>(
                        "vision_loss_stop_duration", 2.5f)));
            real_vision_stale_command_retain_ = retain;
            this->control.x *= retain;
            this->control.y = 0.0f;
            this->control.yaw *= retain;
            if (retain <= 0.0f) StopRealVision("vision trajectory became stale");
            return;
        }
        real_vision_stale_command_retain_ = 1.0f;

        std::size_t nearest = 0;
        double nearest_distance = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < live_trajectory_odom_.size(); ++index)
        {
            const double distance = std::hypot(
                live_trajectory_odom_[index].x - pose.position[0],
                live_trajectory_odom_[index].y - pose.position[1]);
            if (distance < nearest_distance)
            {
                nearest_distance = distance;
                nearest = index;
            }
        }
        const double nearest_s = live_trajectory_odom_[nearest].s;
        const double lookahead = this->params.Get<double>(
            "trajectory_base_lookahead", 1.0);
        const double chord = this->params.Get<double>(
            "vision_heading_chord_length", 1.0);
        const auto aim = SampleRealTrajectory(
            live_trajectory_odom_, nearest_s + lookahead);
        const auto tangent_start = SampleRealTrajectory(
            live_trajectory_odom_, nearest_s + lookahead - 0.5 * chord);
        const auto tangent_end = SampleRealTrajectory(
            live_trajectory_odom_, nearest_s + lookahead + 0.5 * chord);
        const auto aim_base = OdomToBase(pose, aim);
        const auto tangent_start_base = OdomToBase(pose, tangent_start);
        const auto tangent_end_base = OdomToBase(pose, tangent_end);
        const double tangent_heading = std::atan2(
            tangent_end_base.y - tangent_start_base.y,
            tangent_end_base.x - tangent_start_base.x);
        const double cross_track_heading = std::atan2(
            this->params.Get<double>("trajectory_cross_track_gain", 0.8) *
                aim_base.y,
            std::max(aim_base.x, 0.10));
        const double heading_error = WrapAngle(
            tangent_heading + std::clamp(
                cross_track_heading,
                -this->params.Get<double>(
                    "trajectory_cross_track_heading_limit", 0.12),
                this->params.Get<double>(
                    "trajectory_cross_track_heading_limit", 0.12)));
        const double yaw_limit = this->params.Get<double>(
            "real_vision_yaw_command_limit", 0.15);
        const double requested_yaw = std::clamp(
            this->params.Get<double>("trajectory_heading_gain", 1.0) *
                heading_error, -yaw_limit, yaw_limit);
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - real_vision_start_time_).count();
        const double ramp_duration = std::max(
            0.5, this->params.Get<double>(
                "trajectory_start_speed_ramp_time", 2.5));
        double ramp = std::clamp(elapsed / ramp_duration, 0.0, 1.0);
        ramp = ramp * ramp * (3.0 - 2.0 * ramp);
        this->control.x = static_cast<float>(ramp * this->params.Get<double>(
            "trajectory_cruise_speed", 0.30));
        this->control.y = 0.0f;
        this->control.yaw = static_cast<float>(ramp * requested_yaw);

        // Re-express an already observed crack point in the current base
        // frame. This is the key front-camera-to-rear-tool handoff: the point
        // stays fixed in odom while B2 walks past it.
        RealVisionPoint best_target_odom;
        bool target_found = false;
        double best_target_error = std::numeric_limits<double>::infinity();
        const double nominal_x = -0.775;
        for (const auto &point_odom : trajectory_history_odom_)
        {
            auto point_base = OdomToBase(pose, point_odom);
            if (point_base.x < -1.00 || point_base.x > -0.55 ||
                std::fabs(point_base.y) > 0.40) continue;
            const double error = std::fabs(point_base.x - nominal_x);
            if (error < best_target_error)
            {
                best_target_error = error;
                best_target_odom = point_odom;
                target_found = true;
            }
        }
        if (target_found)
        {
            // Tool height is a world/odom vertical offset. Applying it after
            // conversion to base_link would tilt the offset with the body and
            // recreate the TCP-height error on pitched terrain.
            best_target_odom.z += this->params.Get<double>(
                "vision_tool_height_above_ground", 0.09);
            const auto tool_target_base = OdomToBase(pose, best_target_odom);
            const auto x_range = this->params.Get<std::vector<float>>(
                "vision_policy_target_x_range", {-1.0f, -0.55f});
            const auto z_range = this->params.Get<std::vector<float>>(
                "vision_policy_target_z_range", {-0.38f, -0.30f});
            std::array<double, 3> desired{{
                std::clamp(tool_target_base.x,
                           static_cast<double>(x_range[0]),
                           static_cast<double>(x_range[1])),
                std::clamp(tool_target_base.y,
                           -this->params.Get<double>(
                               "vision_policy_target_y_limit", 0.15),
                           this->params.Get<double>(
                               "vision_policy_target_y_limit", 0.15)),
                std::clamp(tool_target_base.z,
                           static_cast<double>(z_range[0]),
                           static_cast<double>(z_range[1]))}};
            if (!real_target_initialized_)
            {
                const auto nominal = this->params.Get<std::vector<float>>(
                    "target_commands");
                real_target_b_ = {{nominal[0], nominal[1], nominal[2]}};
                real_target_initialized_ = true;
            }
            const double alpha = std::clamp(
                0.005 / std::max(0.05, this->params.Get<double>(
                    "vision_policy_target_filter_time_constant", 0.25)),
                0.0, 1.0);
            const auto rate = this->params.Get<std::vector<float>>(
                "vision_policy_target_max_xyz_rate", {0.12f, 0.20f, 0.08f});
            for (int axis = 0; axis < 3; ++axis)
            {
                const double filtered = real_target_b_[axis] +
                    alpha * (desired[axis] - real_target_b_[axis]);
                const double maximum_step = 0.005 * rate[axis];
                real_target_b_[axis] += std::clamp(
                    filtered - real_target_b_[axis],
                    -maximum_step, maximum_step);
            }
            this->SetRuntimeTargetCommands({
                static_cast<float>(real_target_b_[0]),
                static_cast<float>(real_target_b_[1]),
                static_cast<float>(real_target_b_[2]),
                0.0f, -0.707107f, 0.0f, 0.707107f});
        }
    }

    void RobotControl()
    {
        if (!b2_state_ready_.load() || !Z1StateValid())
        {
            return;
        }

        this->GetState(&this->robot_state);
        if (real_vision_enabled_)
            UpdateProprioceptiveOdometry(this->robot_state);
        if (odometry_reset_detected_.exchange(false))
        {
            real_vision_fresh_ = false;
            StopRealVision("B2 odometry reset detected");
            live_trajectory_odom_.clear();
            trajectory_history_odom_.clear();
        }
        this->IngestRealVisionTrajectory();
        if (this->control.current_keyboard == Input::Keyboard::T)
            this->ToggleRealVision();
        if (real_vision_active_ &&
            this->control.current_keyboard == Input::Keyboard::Space)
            this->StopRealVision("operator emergency stop");
        if (real_vision_active_ &&
            this->control.current_keyboard == Input::Keyboard::P)
            this->StopRealVision("operator passive request");

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
        if (real_vision_active_ && !this->locomotion_ready_for_motion)
            this->StopRealVision("locomotion state is no longer ready");
        // Apply autonomous commands after the generic keyboard handler, just
        // as the MuJoCo follower does. Manual W/A/S/D cannot overwrite a live
        // visual path, while Space and P above remain immediate stop inputs.
        this->UpdateRealVisionControl();
        this->control.x = std::clamp(
            this->control.x, -kRealLinearSpeedLimit, kRealLinearSpeedLimit);
        this->control.y = std::clamp(
            this->control.y, -kRealLateralSpeedLimit, kRealLateralSpeedLimit);
        const float yaw_limit = real_vision_active_
            ? this->params.Get<float>("real_vision_yaw_command_limit", 0.15f)
            : kRealYawSpeedLimit;
        this->control.yaw = std::clamp(
            this->control.yaw, -yaw_limit, yaw_limit);
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
                      << " foot_force=["
                      << state.foot_force()[0] << ","
                      << state.foot_force()[1] << ","
                      << state.foot_force()[2] << ","
                      << state.foot_force()[3] << "]"
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
        this->obs.base_quat = state_snapshot.imu.quaternion;
        // Manual yaw remains disabled by kRealYawSpeedLimit in RobotControl,
        // but visual tracking has its own conservative yaw envelope.  Do not
        // clamp that already-validated visual command back to the manual
        // zero-yaw limit here: doing so made control.yaw reach 0.15 while the
        // policy observation (policy_yaw) stayed exactly zero.
        const float policy_yaw_limit = this->params.Get<float>(
            "real_vision_yaw_command_limit", 0.15f);
        std::vector<float> target_commands = this->ApplyMotionHeadingHold({
            std::clamp(this->control.x, -kRealLinearSpeedLimit, kRealLinearSpeedLimit),
            std::clamp(this->control.y, -kRealLateralSpeedLimit, kRealLateralSpeedLimit),
            std::clamp(this->control.yaw, -policy_yaw_limit, policy_yaw_limit)
        });
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

    void SportStateHandler(const void *message)
    {
        const auto state =
            *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
        TimedBasePose pose;
        pose.monotonic_ns = SteadyNowNs();
        for (int axis = 0; axis < 3; ++axis)
            pose.position[axis] = state.position()[axis];
        for (int axis = 0; axis < 4; ++axis)
            pose.quaternion[axis] = state.imu_state().quaternion()[axis];
        if (!NormalizeQuaternion(&pose.quaternion) ||
            !std::all_of(pose.position.begin(), pose.position.end(),
                         [](double value) { return std::isfinite(value); }))
            return;

        std::lock_guard<std::mutex> lock(sport_state_mutex_);
        if (!sport_pose_history_.empty())
        {
            const auto &previous = sport_pose_history_.back();
            const double dt = static_cast<double>(
                pose.monotonic_ns - previous.monotonic_ns) * 1.0e-9;
            const double jump = std::hypot(
                pose.position[0] - previous.position[0],
                pose.position[1] - previous.position[1]);
            // A localization reset would reinterpret every cached crack point.
            // Drop visual state rather than command a discontinuous turn.
            if (dt > 0.0 && dt < 0.25 && jump > 0.75)
            {
                sport_pose_history_.clear();
                return;
            }
        }
        sport_pose_history_.push_back(pose);
        const std::uint64_t history_ns = static_cast<std::uint64_t>(
            1.0e9 * this->params.Get<double>(
                "real_vision_pose_history_duration", 3.0));
        while (sport_pose_history_.size() > 2 &&
               pose.monotonic_ns - sport_pose_history_.front().monotonic_ns >
                   history_ns)
            sport_pose_history_.pop_front();
        sport_state_ready_.store(true);
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
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
        sport_state_subscriber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_subscriber_;
    KeySwitch key_switch_;
    std::mutex b2_state_mutex_;
    std::mutex b2_command_mutex_;
    mutable std::mutex sport_state_mutex_;
    std::deque<TimedBasePose> base_pose_history_;
    std::deque<TimedBasePose> sport_pose_history_;
    std::array<std::array<double, 3>, 4> foot_anchors_odom_{};
    std::array<bool, 4> foot_anchor_valid_{{false, false, false, false}};
    bool proprio_odometry_initialized_ = false;
    std::atomic<bool> b2_state_ready_{false};
    std::atomic<bool> sport_state_ready_{false};
    std::atomic<bool> odometry_reset_detected_{false};
    std::atomic<bool> safety_stop_{false};
    uint32_t last_audit_tick_ = 0;
    unsigned int transport_audit_count_ = 0;
    bool z1_gains_active_ = false;
    bool z1_enabled_ = true;
    bool diagnostic_only_ = false;
    bool joystick_was_active_ = false;
    bool remote_velocity_stop_latched_ = false;
    std::array<bool, 4> real_foot_contact_latched_{{false, false, false, false}};
    uint16_t previous_remote_keys_ = 0;
    bool previous_b2_target_q_valid_ = false;
    bool real_vision_enabled_ = false;
    bool real_vision_active_ = false;
    bool real_vision_fresh_ = false;
    bool real_vision_mtime_valid_ = false;
    bool real_target_initialized_ = false;
    float real_vision_stale_command_retain_ = 1.0f;
    std::string real_vision_exchange_dir_ = "/tmp/b2_z1_real_vision";
    std::filesystem::file_time_type real_vision_mtime_;
    std::chrono::steady_clock::time_point real_vision_last_received_;
    std::chrono::steady_clock::time_point real_vision_start_time_;
    std::uint64_t real_vision_last_parse_warning_ns_ = 0;
    std::vector<RealVisionPoint> live_trajectory_odom_;
    std::vector<RealVisionPoint> trajectory_history_odom_;
    std::array<double, 3> real_target_b_{{-0.775, 0.0, -0.35}};
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
                  << " [--vision] [--vision-exchange-dir DIR]"
                  << std::endl;
        return 1;
    }

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);
    // Ctrl+Z must shut down cleanly instead of suspending a process that keeps
    // DDS/UDP sockets and low-level robot control ownership indefinitely.
    std::signal(SIGTSTP, SignalHandler);
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
        bool enable_real_vision = false;
        std::string vision_exchange_dir = "/tmp/b2_z1_real_vision";
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
            else if (arg == "--vision")
            {
                enable_real_vision = true;
            }
            else if (arg == "--vision-exchange-dir" && i + 1 < argc)
            {
                vision_exchange_dir = argv[++i];
            }
            else
            {
                throw std::runtime_error("Unknown or incomplete argument: " + arg);
            }
        }
        RLRealB2Z1NoGun robot(
            argv[1], enable_motion_switcher, z1_ip, z1_to_port, z1_own_port,
            z1_skip_lowcmd, preflight_only, disable_z1,
            b2_kp_scale, b2_kd_scale, enable_real_vision,
            vision_exchange_dir);
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
