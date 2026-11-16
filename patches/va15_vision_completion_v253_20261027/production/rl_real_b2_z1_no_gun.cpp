/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_b2_z1_no_gun.hpp"
#include "va15_action_state.hpp"
#include "va15_remote_input.hpp"
#include "va15_kinematics_model_io.hpp"
#include "va15_joinable_loop.hpp"
#include "independent_model_io.hpp"
#include "trial_telemetry.hpp"
#include "trial_motor_contract.hpp"
#include "near_visual_io.hpp"
#include "visual_tcp.hpp"
#include "tcp_fk.hpp"
#include "visual_rear_completion.hpp" // V253: finite local endpoint; measured rear TCP owns arrival
#include "coherent_snapshot.hpp"
#include "priority_getdown.hpp"
#include "visual_review.hpp"
#include <cstdlib>
#include <sstream>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include "va15_z1_exit.hpp"

#include <algorithm>
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

#include "blind_fixed_guide.hpp"
class RLRealB2Z1NoGun : public RL
{
public:
    explicit RLRealB2Z1NoGun(const std::string &network_interface, bool enable_motion_switcher,
                          const std::string &z1_ip, uint32_t z1_to_port,
                          uint32_t z1_own_port, bool z1_skip_lowcmd,
                          bool preflight_only, bool disable_z1, float b2_kp_scale,
                          float b2_kd_scale, bool enable_real_vision,
                          const std::string &vision_exchange_dir, bool trial_motion,
                          const std::string &near_directory,const std::string &near_session,bool blind_fixed)
    {
        this->ang_vel_axis = "body";
        this->robot_name = "b2_z1_no_gun";
        this->ReadYaml(this->robot_name, "base.yaml");
        this->ReadYaml(this->robot_name + "/" + this->params.Get<std::string>(
            "deployment_policy_profile", "robot_lab"), "config.yaml");
        if(enable_real_vision)throw std::runtime_error("Isolated forward trial does not support vision");
        if(b2_kp_scale!=1.f||b2_kd_scale!=1.f||disable_z1||z1_skip_lowcmd||preflight_only)
            throw std::runtime_error("Near visual candidate requires the unchanged full-body gains and active Z1; use the separate read-only probe for diagnostics");
        blind_fixed_=blind_fixed;
        if(!blind_fixed_)near_inbox_=std::make_unique<va15_visual::Inbox>(near_directory,near_session);
        trial_motion_=trial_motion;
        if(!preflight_only && !z1_skip_lowcmd)
        {
            const char* directory=std::getenv("VA15_TRIAL_DIAGNOSTICS_DIR");
            if(!directory||!*directory)throw std::runtime_error("Use the diagnostic launcher: VA15_TRIAL_DIAGNOSTICS_DIR is required before takeover");
            telemetry_=std::make_unique<va15_telemetry::Recorder>(directory);
            std::cout<<LOGGER::INFO<<"[Diagnostics] Background CSV recorder: "<<directory
                <<"; no velocity/gain/threshold/budget change"<<std::endl;
        }
        this->params.config_node["isolated_forward_trial"]=true;
        this->params.config_node["isolated_forward_trial_motion"]=trial_motion_;
        this->params.config_node["near_visual_trial"]=!blind_fixed_;
        this->params.config_node["repeatable_visual_session"]=true;
        this->params.config_node["continuous_visual_session"]=true;
        this->params.config_node["independent_B_stop_candidate"]=true;
        auto trial_model=LoadIndependentModel((std::filesystem::path(__FILE__).parent_path()/"b2_independent_model.json").string());
        if(trial_model.source_sha256!="4bccdee2bfec530e0d8e22525ca9d9a5aea8ca7c1e1a7a4eadd979ee9ca2fb9f")
            throw std::runtime_error("Unexpected independent model provenance");
        trial_observer_=std::make_unique<va15_independent::Observer>(trial_model,true);
        std::cout<<LOGGER::WARNING<<"[Session V2.5.2] "<<(blind_fixed_?
            "BLIND FIXED TCP: no camera/visual yaw/path; x=0.15, y=yaw=0; R1+Y starts/stops":
            trial_motion_?"x=0.15 m/s, yaw<=0.06 rad/s; R1+Y starts/stops; cached flat-floor TCP enabled":"SHADOW ONLY: motion disabled")
            <<"; estimated support, not calibrated/certified feedback"<<std::endl;

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
        ConfigureStopFeedback();
        ConfigureKinematicsShadow();

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
            const auto odometry_deadline =
                std::chrono::steady_clock::now() + std::chrono::seconds(3);
            TimedBasePose pose;
            while (!GetLatestBasePose(&pose) &&
                   std::chrono::steady_clock::now() < odometry_deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (!GetLatestBasePose(&pose))
            {
                throw std::runtime_error(
                    "Real vision requires fresh rt/sportmodestate odometry; "
                    "the topic was not received within 3 seconds");
            }
            std::cout << LOGGER::INFO
                      << "B2 visual odometry ready: position=["
                      << pose.position[0] << ", " << pose.position[1]
                      << ", " << pose.position[2] << "]" << std::endl;
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

        loop_keyboard_ = std::make_shared<VA15JoinableLoop>(
            "real_keyboard", 0.05,
            std::bind(&RLRealB2Z1NoGun::KeyboardInterface, this));
        loop_control_ = std::make_shared<VA15JoinableLoop>(
            "real_control", this->params.Get<float>("dt"),
            std::bind(&RLRealB2Z1NoGun::RobotControl, this));
        // B2's official low-level examples refresh rt/lowcmd every 2 ms.
        // Keep policy/FSM timing unchanged and retransmit the latest frame at
        // the hardware-required rate.
        loop_b2_writer_ = std::make_shared<VA15JoinableLoop>(
            "real_b2_writer", 0.002,
            std::bind(&RLRealB2Z1NoGun::WriteB2LowCmd, this));
        loop_rl_ = std::make_shared<VA15JoinableLoop>(
            "real_policy",
            this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
            std::bind(&RLRealB2Z1NoGun::RunModel, this));
        if (motion_switcher_)
        {
            loop_motion_audit_ = std::make_shared<VA15JoinableLoop>(
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
        std::cout<<LOGGER::INFO<<"Remote: A=get up, Y=policy standby, R1+Y=visual trajectory, "
            <<"X=zero velocity, B=get down, L1+X=emergency passive. "
            <<"Keyboard: 0/1/T/Space/9/P. Centre sticks after X before re-arming."<<std::endl;
    }

    ~RLRealB2Z1NoGun()
    {
        callbacks_shutting_down_.store(true);
        if (loop_keyboard_) loop_keyboard_->shutdown();
        if (loop_control_) loop_control_->shutdown();
        if (loop_rl_) loop_rl_->shutdown();
        if (loop_motion_audit_) loop_motion_audit_->shutdown();

        // A diagnostic construction never starts the writer, and must not
        // prepare/send a low-level passive command as a teardown side effect.
        if (!diagnostic_only_)
        {
            SendPassiveCommands();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (loop_b2_writer_) loop_b2_writer_->shutdown();
        if (z1_arm_)
        {
            std::cout << LOGGER::INFO << "[Shutdown] Joining Z1 communication loop" << std::endl;
            z1_arm_->sendRecvThread->shutdown();
            if (!diagnostic_only_)
            {
                auto* component=z1_arm_->_ctrlComp;
                const auto result=VA15RequestZ1PassiveOnExit(*component->udp,
                    component->sendCmd,component->recvState);
                std::cout << (result.passive_reply ? LOGGER::INFO : LOGGER::WARNING)
                          << "[Shutdown] Z1 PASSIVE reply=" << result.passive_reply
                          << ", attempts=" << result.attempts
                          << "; bounded exit, not physical safety confirmation" << std::endl;
            }
        }
        // SDK CloseChannel resets DdsReader and joins its queued listener.
        // Do this while every mutex, history, and kinematics tracker is alive.
        if (joystick_subscriber_) joystick_subscriber_->CloseChannel();
        if (sport_state_subscriber_) sport_state_subscriber_->CloseChannel();
        if (lowstate_subscriber_) lowstate_subscriber_->CloseChannel();
        // Never delay the established robot/Z1 shutdown for diagnostic I/O.
        if(telemetry_)
            std::cout<<LOGGER::INFO<<"[Diagnostics] Recorder complete="<<telemetry_->Close()
                <<"; DDS write return is NOT a motor acknowledgement"<<std::endl;
        std::cout << LOGGER::INFO << "B2+Z1 real deployment stopped" << std::endl;
    }

    bool DiagnosticOnly() const
    {
        return diagnostic_only_;
    }

    bool RequiresRealStopFeedback() const override { return true; }

private:
    void ConfigureKinematicsShadow()
    {
        if (!this->params.Get<bool>("real_kinematics_shadow_only", false))
            throw std::runtime_error("Kinematic observer is diagnostics-only; no control override is supported");
        const auto file = this->params.Get<std::string>("real_kinematics_model_file", "");
        if (file.empty() || file.find('/') != std::string::npos || file.find('\\') != std::string::npos)
            throw std::runtime_error("Invalid profile-local shadow model filename");
        const auto profile = this->params.Get<std::string>("deployment_policy_profile", "VA15");
        kinematics_tracker_ = std::make_unique<va15_shadow::Tracker>(LoadVA15ShadowModel(
            std::string(POLICY_DIR) + "/" + this->robot_name + "/" + profile + "/" + file));
    }

    void PrintKinematicsShadow(const va15_shadow::Result& value) const
    {
        std::cout << LOGGER::INFO << "[Kinematics shadow] input_valid=" << value.input_valid
                  << " fresh=" << value.fresh << " sequence_valid=" << value.sequence_valid
                  << " h_hyp=" << value.hypothesis_height
                  << " speed_hyp=" << value.hypothesis_velocity_world.norm()
                  << " height_spread=" << value.height_spread
                  << " velocity_spread=" << value.velocity_spread
                  << " stance_evidence=" << (value.accepted_contacts >= 2 ? "conditional" : "UNKNOWN")
                  << " authority=0; " << value.reason << std::endl;
    }

    void ConfigureStopFeedback()
    {
        VA15StopFeedbackTracker::Calibration calibration;
        calibration.foot_validated = this->params.Get<bool>(
            "real_foot_contact_calibration_validated", false);
        calibration.sport_validated = this->params.Get<bool>(
            "real_stop_sport_feedback_validated", false);
        calibration.id = this->params.Get<std::string>("real_foot_contact_calibration_id", "");
        if (calibration.foot_validated)
        {
            const auto array = [&](const std::string& key) {
                auto value = this->params.Get<std::vector<float>>(key, {});
                if (value.size() != 4) throw std::runtime_error("Expected four DDS-order values: " + key);
                return std::array<float, 4>{{value[0], value[1], value[2], value[3]}};
            };
            calibration.zero = array("real_foot_contact_zero_dds");
            calibration.direction = array("real_foot_contact_direction_dds");
            calibration.on = array("real_foot_contact_on_dds");
            calibration.off = array("real_foot_contact_off_dds");
        }
        if (calibration.sport_validated && this->params.Get<std::string>(
                "real_stop_sport_validation_id", "").empty())
            throw std::runtime_error("Missing low-level sport height/velocity validation ID");
        stop_feedback_tracker_.Configure(calibration);
    }

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
        VA15StopFeedback feedback;
        va15_shadow::Result shadow;
        {
            std::lock_guard<std::mutex> lock(b2_state_mutex_);
            state = b2_low_state_;
            feedback = stop_feedback_tracker_.Snapshot(SteadyNowNs() * 1.e-9);
            shadow = kinematics_tracker_->Snapshot(SteadyNowNs() * 1.e-9);
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
        std::cout << LOGGER::INFO << "[Real stop feedback] raw_dds_FR_FL_RR_RL=["
                  << feedback.raw_dds[0] << ',' << feedback.raw_dds[1] << ','
                  << feedback.raw_dds[2] << ',' << feedback.raw_dds[3]
                  << "] height=" << feedback.base_height << " planar_speed=" << feedback.planar_speed
                  << " contacts_valid=" << feedback.contacts_valid
                  << " height_valid=" << feedback.height_valid << " velocity_valid=" << feedback.velocity_valid
                  << " low_fresh=" << feedback.lowstate_fresh << " sport_fresh=" << feedback.sport_fresh
                  << " unavailable=" << feedback.unavailable_reason
                  << "; unvalidated feedback blocks motion, not A/B static tests" << std::endl;
        PrintKinematicsShadow(shadow);
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

    // The control thread alone consumes radio buttons and joystick state.
    // Policy/preflight GetState calls only copy sensors.
    void ProcessRemoteInput()
    {
        unitree_go::msg::dds_::LowState_ b2_state;
        unitree_go::msg::dds_::WirelessController_ joystick;
        {
            std::lock_guard<std::mutex> lock(b2_state_mutex_);
            b2_state=b2_low_state_; joystick=joystick_;
        }
        RawRemotePacket raw{};
        std::memcpy(&raw,b2_state.wireless_remote().data(),sizeof(raw));
        const bool valid=std::isfinite(raw.lx) && std::isfinite(raw.ly) &&
            std::isfinite(raw.rx) && std::fabs(raw.lx)<=1.5f &&
            std::fabs(raw.ly)<=1.5f && std::fabs(raw.rx)<=1.5f;
        const float lx=valid ? raw.lx : joystick.lx();
        const float ly=valid ? raw.ly : joystick.ly();
        const float rx=valid ? raw.rx : joystick.rx();
        telemetry_raw_axes_={{lx,ly,rx}};
        telemetry_remote_keys_=valid?raw.keys:joystick.keys();
        telemetry_remote_valid_=valid;
        const bool centered=std::isfinite(lx) && std::isfinite(ly) && std::isfinite(rx) &&
            std::fabs(lx)<=kJoystickDeadzone && std::fabs(ly)<=kJoystickDeadzone &&
            std::fabs(rx)<=kJoystickDeadzone;
        const auto input=remote_input_.Update(valid ? raw.keys : joystick.keys(),centered);
        remote_motion_inhibited_=input.inhibit;
        using Event=VA15RemoteInput::Event;
        switch(input.event)
        {
        case Event::GetUp:
            this->control.SetGamepad(Input::Gamepad::A);
            std::cout<<LOGGER::INFO<<"[Remote] A -> get up"<<std::endl;
            break;
        case Event::Policy:
            this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
            std::cout<<LOGGER::INFO<<"[Remote] Y (or R1+Up) -> locomotion policy standby"<<std::endl;
            break;
        case Event::Vision:
            if(!input.inhibit) {
                this->control.SetKeyboard(Input::Keyboard::T);
                std::cout<<LOGGER::INFO<<"[Remote] R1+Y -> toggle "
                    <<(blind_fixed_?"blind fixed-TCP forward diagnostic":"body guide + cached flat-floor TCP")
                    <<" (V2.5.2)"<<std::endl;
                std::cout<<LOGGER::INFO<<"[Near visual] "<<(trial_motion_?"observed front guide; no unseen body extrapolation":"SHADOW ONLY")
                    <<"; cached flat-floor TCP enabled, no depth sensor"<<std::endl;
            }
            break;
        case Event::Stop:
            this->control.SetKeyboard(Input::Keyboard::Space);
            // Do not send Gamepad::X: the shared SDK maps it to navigation.
            StopRealVision("remote X");
            std::cout<<LOGGER::INFO<<"[Remote] X -> zero velocity; centre sticks to re-arm"<<std::endl;
            break;
        case Event::GetDown:
            this->control.SetGamepad(Input::Gamepad::B);
            StopRealVision("remote B");
            std::cout<<LOGGER::INFO<<"[Remote] B -> get down; manual motion inhibited until A"<<std::endl;
            break;
        case Event::Emergency:
            safety_stop_.store(true);
            this->rl_init_done=false;
            this->control.SetGamepad(Input::Gamepad::LB_X);
            this->control.SetKeyboard(Input::Keyboard::P);
            StopRealVision("remote L1+X");
            std::cout<<LOGGER::ERROR<<"[Remote] L1+X -> emergency passive; restart required"<<std::endl;
            break;
        case Event::None: break;
        }
        if(input.zero || !std::isfinite(lx) || !std::isfinite(ly) || !std::isfinite(rx)) {
            this->control.x=this->control.y=this->control.yaw=0.f;
            joystick_was_active_=false;
            return;
        }
        const bool active=!centered;
        if(active || joystick_was_active_) {
            this->control.x=ScaleJoystickAxis(ly,kRealLinearSpeedLimit);
            this->control.y=ScaleJoystickAxis(-lx,kRealLateralSpeedLimit);
            this->control.yaw=ScaleJoystickAxis(-rx,kRealYawSpeedLimit);
            joystick_was_active_=active;
        }
    }

    void GetState(RobotState<float> *state) override
    {GetStateTagged(state,nullptr,nullptr);}

    void GetStateTagged(RobotState<float>* state,va15_telemetry::Source* source,
        va15_telemetry::Command* measured=nullptr)
    {
        unitree_go::msg::dds_::LowState_ b2_state;
        va15_motor_contract::Status current_health;
        {
            std::lock_guard<std::mutex> lock(b2_state_mutex_);
            b2_state=b2_low_state_;
            if(source){source->tick=b2_state.tick();source->received_ns=b2_low_received_ns_;source->copy_ns=SteadyNowNs();}
            state->real_stop_feedback = stop_feedback_tracker_.Snapshot(SteadyNowNs() * 1.e-9);
            state->kinematics_shadow = kinematics_tracker_->Snapshot(SteadyNowNs() * 1.e-9);
            // Explicit alternative source in this private executable only.
            // Do not change any legacy calibration/validation YAML flag.
            const double health_now=SteadyNowNs()*1.e-9;
            current_health=motor_contract_.Snapshot(health_now);
            auto estimated=trial_observer_->Snapshot(health_now);
            if(!current_health.ready)estimated.input_valid=false;
            state->real_stop_feedback=va15_trial::Feedback(estimated);
            if(measured)for(int i=0;i<4;++i) {
                auto& c=measured->context;
                c.load_normal_dds[i]=estimated.normal_equivalent[i];
                c.load_horizontal_dds[i]=estimated.horizontal_equivalent[i];
                c.load_horizontal_limit_dds[i]=estimated.horizontal_limit[i];
                c.load_normal_lower_dds[i]=estimated.normal_lower[i];
                c.load_on_dds[i]=estimated.load_on_seconds[i];
                c.load_off_dds[i]=estimated.load_off_seconds[i];
                c.load_friction_credible_dds[i]=estimated.friction_credible[i];
            }
        }

        if(measured)
        {
            measured->bit_flag=b2_state.bit_flag();
            measured->contract_ready=current_health.ready;
            measured->contract_latched=current_health.latched;
            measured->contract_reason=unsigned(current_health.reason);
            measured->contract_bad_joint=current_health.joint;
            measured->contract_baseline_frozen=current_health.baseline_frozen;
            measured->contract_stable_seconds=current_health.stable_seconds;
            measured->contract_startup_changes=current_health.startup_changes;
            measured->lost_diagnostic_only=current_health.lost_diagnostic_only;
            measured->lost_change_events=current_health.lost_change_events;
            measured->contract_baseline=current_health.baseline;
            for(int i=0;i<12;++i)
            {
                const auto& m=b2_state.motor_state()[i];
                measured->q[i]=m.q();measured->dq[i]=m.dq();measured->tau[i]=m.tau_est();
                measured->mode[i]=m.mode();measured->lost[i]=m.lost();
                measured->motor_fault[i]=m.reserve()[0];measured->motor_frequency[i]=m.reserve()[1];
            }
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
            telemetry_command_context_.context.command_generated_ns=SteadyNowNs();
            telemetry_command_context_.sequence=++telemetry_command_sequence_;
            telemetry_command_context_.latest_actor_completion=telemetry_actor_completed_.load();
            telemetry_published_context_=telemetry_command_context_;
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
                // Diagnostic only: LOWCMD may replace the command-buffer
                // gains during the FSM transition. Do not infer physical
                // units or change the requested gains from this printout.
                const auto retained_kp = z1_arm_->_ctrlComp->lowcmd->kp;
                const auto retained_kd = z1_arm_->_ctrlComp->lowcmd->kd;
                std::ostringstream gain_diagnostic;
                gain_diagnostic << "Z1 LOWCMD buffer after FSM kp=[";
                for (std::size_t i = 0; i < retained_kp.size(); ++i)
                {
                    if (i) gain_diagnostic << ',';
                    gain_diagnostic << retained_kp[i];
                }
                gain_diagnostic << "] kd=[";
                for (std::size_t i = 0; i < retained_kd.size(); ++i)
                {
                    if (i) gain_diagnostic << ',';
                    gain_diagnostic << retained_kd[i];
                }
                gain_diagnostic << "] (SDK buffer snapshot, units unverified, not motor acknowledgment)";
                std::cout << LOGGER::INFO << gain_diagnostic.str() << std::endl;
                z1_arm_->setArmCmd(z1_handoff_q_, zero, zero);
                z1_handoff_cycles_ = kZ1HandoffCycles;
                std::cout << LOGGER::INFO
                          << "Z1 LOWCMD seeded at measured pose; requested_kp_scale="
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
                        "no synchronized B2 odometry pose for camera frame");
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

    bool PolicyStandbyReady() const
    {
        auto state=std::dynamic_pointer_cast<b2_z1_no_gun_fsm::RLFSMStateRLLocomotion>(
            this->fsm.current_state_);
        return VA15RemoteInput::VisionReady(bool(state),
            state && state->policy_started && this->model && state->handoff_hold_cycles<=0,
            state && state->getdown_requested,safety_stop_.load());
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
        const auto standby = std::dynamic_pointer_cast<b2_z1_no_gun_fsm::RLFSMStateRLLocomotion>(
            this->fsm.current_state_);
        // This stationary gate is an ENTRY requirement, not a condition that
        // would stop a healthy trajectory every time a foot swings.
        if (!PolicyStandbyReady() || !standby->RealStopSettled() ||
            standby->real_start_support_cycles < 100)
        {
            std::cout << LOGGER::WARNING
                      << "[RealVision] requires Y/1 standby, preparation, and 0.5 s validated stationary four-foot support; "
                      << "feedback=" << this->robot_state.real_stop_feedback.unavailable_reason
                      << std::endl;
            return;
        }
        if (!GetLatestBasePose(&pose))
        {
            std::cout << LOGGER::WARNING
                      << "[RealVision] no fresh rt/sportmodestate odometry; "
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
            StopRealVision("B2 odometry became stale");
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
        const auto control_begin=SteadyNowNs(),previous_begin=telemetry_control_previous_ns_;
        telemetry_control_previous_ns_=control_begin;
        if(previous_begin)telemetry_control_max_interval_=std::max(telemetry_control_max_interval_,control_begin-previous_begin);
        if (!b2_state_ready_.load() || !Z1StateValid())
        {
            return;
        }

        telemetry_command_context_={};
        this->GetStateTagged(&this->robot_state,&telemetry_command_context_.source,&telemetry_command_context_);
        ProcessRemoteInput();
        // Do not stand up or enter policy mode before valid current fault,
        // communication-frequency and command-stream evidence. B/P retain
        // their existing priority and are never blocked by this entry gate.
        const bool request_entry=this->control.current_keyboard==Input::Keyboard::Num0 ||
            this->control.current_keyboard==Input::Keyboard::Num1 ||
            this->control.current_gamepad==Input::Gamepad::A ||
            this->control.current_gamepad==Input::Gamepad::RB_DPadUp;
        if(request_entry&&!telemetry_command_context_.contract_ready)
        {
            this->control.current_keyboard=Input::Keyboard::None;
            this->control.current_gamepad=Input::Gamepad::None;
            this->control.x=this->control.y=this->control.yaw=0.f;
            std::cout<<LOGGER::WARNING<<"[Motor contract] A/Y refused: "
                <<va15_motor_contract::Text(static_cast<va15_motor_contract::Reason>(telemetry_command_context_.contract_reason))
                <<"; wait for READY then explicitly request again"<<std::endl;
        }
        // B/9 and its latched intent bypass camera inbox/cache/projection.
        // The FSM still performs a supported stop; this does NOT force down.
        if(!PrioritizeGetDown())UpdateNearGuide(control_begin,previous_begin);
        const std::array<float,3> requested{{this->control.x,this->control.y,this->control.yaw}};
        const auto wall_deadline=trial_wall_deadline_ns_.load();
        if(wall_deadline!=0 && SteadyNowNs()>=wall_deadline)
        {
            auto state=std::dynamic_pointer_cast<b2_z1_no_gun_fsm::RLFSMStateRLLocomotion>(
                this->fsm.states_.at("RLFSMStateRLLocomotion"));
            state->trial.Stop("monotonic wall-clock request deadline reached");
            this->control.x=this->control.y=this->control.yaw=0.f;
        }
        if (odometry_reset_detected_.exchange(false))
        {
            real_vision_fresh_ = false;
            StopRealVision("B2 odometry reset detected");
            live_trajectory_odom_.clear();
            trajectory_history_odom_.clear();
        }
        this->IngestRealVisionTrajectory();
        // T is consumed by UpdateNearGuide. The old metric/TCP follower is
        // intentionally disabled; camera bearings never enter that interface.
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
        if (safety_stop_.load() || !StateIsSafe(this->robot_state, &unsafe_reason))
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
        if(remote_motion_inhibited_)
            this->control.x=this->control.y=this->control.yaw=0.f;
        if (real_vision_active_ && !PolicyStandbyReady())
            this->StopRealVision("policy standby is no longer ready");
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
        auto trial_state=std::dynamic_pointer_cast<b2_z1_no_gun_fsm::RLFSMStateRLLocomotion>(
            this->fsm.states_.at("RLFSMStateRLLocomotion"));
        const bool forward=trial_motion_ && GuidanceRunning() &&
            trial_state->trial.Forward() &&
            this->fsm.current_state_->GetStateName()=="RLFSMStateRLLocomotion";
        if(forward && !kContinuousVisual && trial_wall_deadline_ns_.load()==0)
            trial_wall_deadline_ns_.store(SteadyNowNs()+va15_trial::Supervisor::RequestNanoseconds);
        trial_inference_forward_.store(forward);
        visual_control_heartbeat_ns_.store(control_begin);
        // StateController's shared keyboard handler runs AFTER FSM::Run.
        // Clamp again so W/A/S/D cannot bypass the supervisor.
        this->control.x=forward?std::clamp(this->control.x,0.f,float(va15_trial::Supervisor::SpeedLimit)):0.f;
        this->control.y=0.f;
        this->control.yaw=forward?float(near_output_.yaw):0.f;
        near_inference_yaw_.store(this->control.yaw);
        this->control.current_keyboard=this->control.last_keyboard=Input::Keyboard::None;
        this->control.current_gamepad=this->control.last_gamepad=Input::Gamepad::None;
        auto& context=telemetry_command_context_.context;
        context.control_sequence=++telemetry_control_sequence_;
        context.control_start_ns=control_begin;
        const auto state_name=this->fsm.current_state_->GetStateName();
        context.fsm=state_name=="RLFSMStatePassive"?0:state_name=="RLFSMStateGetUp"?1:
            state_name=="RLFSMStateRLLocomotion"?2:state_name=="RLFSMStateGetDown"?3:4;
        context.trial=int(trial_state->trial.stage);context.stop=int(trial_state->controlled_stop.stage);
        context.policy_running=trial_state->policy_running;context.inhibit=remote_motion_inhibited_;
        context.safety_fault=safety_stop_.load();context.raw_axes=telemetry_raw_axes_;
        context.raw_keys=telemetry_remote_keys_;context.remote_valid=telemetry_remote_valid_;
        context.requested=requested;context.limited={{this->control.x,this->control.y,this->control.yaw}};
        for(int i=0;i<3;++i)context.policy_command[i]=telemetry_policy_command_[i].load();
        context.request_seconds=trial_state->trial.requested;
        context.h_lower=this->robot_state.real_stop_feedback.base_height;
        context.v_upper=this->robot_state.real_stop_feedback.planar_speed;
        context.estimated_valid=va15_trial::EstimatedValid(this->robot_state.real_stop_feedback);
        context.stance_offset=trial_state->leg_offset_scale;
        context.front_handoff=trial_state->front_handoff_cycles;
        context.rear_handoff=trial_state->rear_handoff_cycles;
        context.arm_blend=trial_state->arm_blend_cycles;
        context.torque_handoff_active=trial_state->policy_handoff.Active();
        context.torque_handoff_pending=trial_state->policy_handoff.pending;
        context.torque_handoff_failed=trial_state->policy_handoff.failed;
        context.torque_handoff_elapsed=trial_state->policy_handoff.elapsed;
        context.blind_fixed_target=blind_fixed_;
        const auto& gate=trial_state->controlled_stop.diagnostic;
        context.stop_gate_mask=gate.mask;context.stop_contacts=gate.contacts;
        context.stop_contacts_valid=gate.contacts_valid;context.stop_source_valid=gate.source_valid;
        context.stop_quiet_s=trial_state->controlled_stop.quiet;
        context.stop_tilt=gate.tilt;context.stop_gyro=gate.gyro;
        context.stop_joint_speed=gate.joint_speed;context.stop_joint_drift=gate.drift;
        for(int i=0;i<3;++i)context.body_gyro[i]=this->robot_state.imu.gyroscope[i];
        for(int i=0;i<4;++i)context.body_quaternion[i]=this->robot_state.imu.quaternion[i];
        context.visual_sequence=near_guide_.latest.sequence;
        context.visual_capture_ns=near_guide_.latest.capture_ns;
        context.visual_publish_ns=near_guide_.latest.publish_ns;
        const bool reviewing=near_review_.guide.stage==va15_visual::Guide::Stage::Running;
        context.visual_stage=int(reviewing?near_review_.guide.stage:near_guide_.stage);
        context.visual_qualified=near_guide_.qualified;
        context.visual_ready=near_guide_.Ready(control_begin);context.visual_shadow=!near_guide_.spent;
        const double bearing=va15_visual::Bearing(near_guide_.latest.u,near_guide_.latest.v);
        context.visual_bearing=std::isfinite(bearing)?bearing:0.;
        context.visual_would_x=reviewing?near_review_.would.x:near_output_.x;
        context.visual_would_yaw=reviewing?near_review_.would.yaw:near_output_.yaw;
        const auto look_bearing=va15_visual::Bearing(near_guide_.latest.look_u,near_guide_.latest.look_v);
        context.visual_look_bearing=std::isfinite(look_bearing)?look_bearing:0.;
        context.visual_path_count=near_guide_.latest.path_count;context.visual_path_span=near_guide_.latest.path_span;
        context.visual_review_approved=near_review_.approved;
        context.visual_session_count=near_guide_.sessions;
        context.visual_end_stage=int(front_end_stop_.stage);context.visual_end_frames=front_end_stop_.frames;
        context.visual_end_requested=front_end_stop_.Requested();context.visual_end_stood=front_end_stop_.Stood();
        context.terminal_confirmed=front_end_stop_.Confirmed();
        context.terminal_remaining=front_end_stop_.remaining;context.terminal_lateral=front_end_stop_.lateral;
        context.terminal_xy_error=front_end_stop_.distance;context.terminal_z_error=front_end_stop_.height_error;
        for(int j=0;j<3;++j)context.terminal_endpoint[j]=front_end_stop_.endpoint[j];
        context.body_cross_track=near_guide_.body_lateral;context.body_path_heading=near_guide_.body_heading;
        context.yaw_reference=near_guide_.yaw_reference;context.course_slip_diagnostic=near_guide_.course_slip;
        context.body_reference_valid=near_guide_.body_reference_valid;context.body_reference_history=near_guide_.body_history_used;
        context.body_control_weight=near_guide_.body_control_weight;context.visual_integral=near_guide_.visual_integral;
        context.visual_preview_valid=near_guide_.metric_bearing_valid;
        context.visual_preview_history=tcp_forward_reference_.historical;
        context.visual_preview_x=tcp_forward_reference_.point_body.x();
        context.visual_preview_y=tcp_forward_reference_.point_body.y();
        context.visual_preview_age=tcp_forward_reference_age_;
        context.stop_recovery_quiet=trial_state->controlled_stop.recovery_quiet;
        context.getdown_pending=trial_state->getdown_requested;
        context.getdown_visual_preempted=getdown_visual_preempted_;
        context.stop_planted_hip_reference=trial_state->controlled_stop.planted_hip_reference_valid;
        if(trial_state->controlled_stop.recovery_reference_q.size()==18)
            for(int i=0;i<4;++i)context.stop_hip_reference[i]=trial_state->controlled_stop.recovery_reference_q[i];
        context.tcp_active=tcp_tracker_.active;context.tcp_ever_active=tcp_tracker_.ever_tracked;
        context.tcp_cache_curves=tcp_tracker_.curves.size();context.tcp_cache_age=tcp_tracker_.cache_age;
        context.tcp_rear_crossings=tcp_tracker_.rear_crossings;context.tcp_workspace_rejections=tcp_tracker_.workspace_rejections;
        context.tcp_rear_gap=tcp_tracker_.nearest_rear_gap;context.tcp_pose_valid=tcp_local_pose_.valid;
        va15_tcp::V3 measured_tcp;
        context.tcp_measured_valid=va15_tcp::MeasuredTCPBase(this->robot_state.motor_state.q,measured_tcp);
        context.tcp_error_norm=context.tcp_measured_valid?(measured_tcp-tcp_tracker_.target).norm():0.;
        for(int j=0;j<3;++j) {
            context.tcp_desired[j]=tcp_tracker_.desired[j];context.tcp_target[j]=tcp_tracker_.target[j];
            context.tcp_measured[j]=context.tcp_measured_valid?measured_tcp[j]:0.;context.local_pose[j]=tcp_local_pose_.p[j];
        }
        this->SetCommand(&this->robot_command);
        if(telemetry_ && control_begin-telemetry_control_record_ns_>=20000000ull)
        {
            va15_telemetry::Control record;
            record.source=telemetry_command_context_.source;record.context=context;
            record.previous_start_ns=previous_begin;record.max_interval_ns=telemetry_control_max_interval_;
            record.latest_actor_completion=telemetry_actor_completed_.load();
            for(int i=0;i<18;++i)
            {
                record.q[i]=this->robot_state.motor_state.q[i];record.dq[i]=this->robot_state.motor_state.dq[i];
                record.tau[i]=this->robot_state.motor_state.tau_est[i];record.fsm_q[i]=this->robot_command.motor_command.q[i];
            }
            record.end_ns=SteadyNowNs();telemetry_->Push(record);
            telemetry_control_record_ns_=control_begin;telemetry_control_max_interval_=0;
        }
        if (++kinematics_log_cycles_ % 200 == 0)
        {
            if(trial_state->controlled_stop.Active()) {
                const auto& g=trial_state->controlled_stop.diagnostic;
                std::cout<<LOGGER::WARNING<<"[Stop gate V2.5.2] stage="<<int(trial_state->controlled_stop.stage)
                    <<" fail_mask="<<g.mask<<" contacts_FL_FR_RL_RR=";
                for(unsigned i=0;i<4;++i)std::cout<<((g.contacts&(1u<<i))?'1':'0');
                std::cout<<" contacts_valid="<<g.contacts_valid<<" source_valid="<<g.source_valid
                    <<" quiet_s="<<trial_state->controlled_stop.quiet<<" h="<<g.height<<" v_upper="<<g.speed
                    <<" tilt="<<g.tilt<<" gyro="<<g.gyro<<" dq="<<g.joint_speed<<" drift="<<g.drift
                    <<"; diagnostics only, no forced lowering"<<std::endl;
                std::cout<<LOGGER::INFO<<"[GetDown] pending="<<context.getdown_pending
                    <<" visual_preempted="<<context.getdown_visual_preempted
                    <<" fixed_hip_reference="<<context.stop_planted_hip_reference
                    <<"; SDK/DDS order FR,FL,RR,RL model force diagnostics:\n";
                for(unsigned i=0;i<4;++i)
                    std::cout<<"  foot="<<i<<" fz="<<context.load_normal_dds[i]
                        <<" horizontal="<<context.load_horizontal_dds[i]
                        <<" limit="<<context.load_horizontal_limit_dds[i]
                        <<" fz_lower="<<context.load_normal_lower_dds[i]
                        <<" credible="<<context.load_friction_credible_dds[i]
                        <<" on_s="<<context.load_on_dds[i]<<" off_s="<<context.load_off_dds[i]<<'\n';
            }
            PrintKinematicsShadow(this->robot_state.kinematics_shadow);
            const auto& evidence=this->robot_state.real_stop_feedback;
            const auto& health=telemetry_command_context_;
            std::cout<<LOGGER::INFO<<"[Motor contract] ready="<<health.contract_ready
                <<" latched="<<health.contract_latched<<" bit_flag="<<health.bit_flag
                <<" baseline_frozen="<<health.contract_baseline_frozen<<" stable_s="<<health.contract_stable_seconds
                <<" startup_changes="<<health.contract_startup_changes
                <<" lost_DIAGNOSTIC_ONLY="<<health.lost_diagnostic_only
                <<" lost_change_events="<<health.lost_change_events
                <<" bad_joint="<<health.contract_bad_joint<<" reason="
                <<va15_motor_contract::Text(static_cast<va15_motor_contract::Reason>(health.contract_reason))
                <<"; hardware_qualified=0"<<std::endl;
            std::cout<<LOGGER::INFO<<"[Motor contract counts] raw_dds=[";
            for(int i=0;i<12;++i)std::cout<<(i?",":"")<<health.lost[i];
            std::cout<<"] baseline_dds=[";
            for(int i=0;i<12;++i)std::cout<<(i?",":"")<<health.contract_baseline[i];
            std::cout<<"]"<<std::endl;
            std::cout<<LOGGER::WARNING<<"[Restricted trial estimate] source="<<evidence.unavailable_reason
                <<" sport_fresh="<<evidence.sport_fresh<<" support_estimate_valid="<<va15_trial::EstimatedValid(evidence)
                <<" h_lower="<<evidence.base_height<<" v_upper="<<evidence.planar_speed
                <<" motion_enabled="<<trial_motion_<<" hardware_qualified=0"<<std::endl;
        }
    }

    bool PrioritizeGetDown()
    {
        const bool pressed=this->control.current_keyboard==Input::Keyboard::Num9||
            this->control.current_gamepad==Input::Gamepad::B;
        auto loco=std::dynamic_pointer_cast<b2_z1_no_gun_fsm::RLFSMStateRLLocomotion>(
            this->fsm.states_.at("RLFSMStateRLLocomotion"));
        const bool in_policy=this->fsm.current_state_->GetStateName()=="RLFSMStateRLLocomotion";
        getdown_visual_preempted_=va15_priority::PreemptVisualForGetDown(pressed,
            in_policy&&loco->getdown_requested,near_guide_,near_output_);
        if(!getdown_visual_preempted_)return false;
        if(in_policy)loco->getdown_requested=true;
        blind_guide_.Stop("operator get-down priority");
        front_end_stop_.Interrupt("operator get-down priority; not visual-end completion");
        if(in_policy)loco->trial.Stop("operator get-down priority; request remains pending");
        this->control.x=this->control.y=this->control.yaw=0.f;
        if(this->control.current_keyboard==Input::Keyboard::T)
            this->control.current_keyboard=Input::Keyboard::None;
        near_inference_yaw_.store(0.f);trial_inference_forward_.store(false);
        tcp_tracker_.active=false;tcp_tracker_.reason="operator get-down priority; holding last TCP target";
        tcp_local_pose_.valid=false;tcp_forward_reference_={};tcp_forward_reference_age_=0.;
        if(near_inbox_)near_inbox_->SetPhase(3u);
        return true;
    }

    void UpdateNearGuide(std::uint64_t now,std::uint64_t previous)
    {
        // Keep controller dt tied to consecutive control STARTS. Source age
        // instead uses a clock sampled AFTER the receive mutex is acquired.
        // A callback can arrive between control_begin and this snapshot.
        const double control_dt=previous?double(now-previous)*1.e-9:.005;
        if(blind_fixed_) { UpdateBlindGuide(control_dt);return; }
        const auto item=near_inbox_->Snapshot();
        va15_tcp::Pose local_pose,capture_pose;
        va15_tcp::V3 local_body_velocity=va15_tcp::V3::Zero();
        unsigned pose_generation=0;
        bool local_valid=false,capture_valid=false;
        now=VA15ReadCoherentSnapshot(b2_state_mutex_,[]{return SteadyNowNs();},[&](std::uint64_t snapshot_now){
            pose_generation=tcp_pose_history_.generation;
            const auto estimate=trial_observer_->Snapshot(snapshot_now*1.e-9);
            if(va15_trial::Healthy(estimate)&&tcp_last_supported_pose_ns_&&snapshot_now>=tcp_last_supported_pose_ns_&&
               snapshot_now-tcp_last_supported_pose_ns_<=250000000ULL&&
               !tcp_pose_history_.poses.empty()) {
                local_pose=tcp_pose_history_.poses.back();
                local_body_velocity=local_pose.q.conjugate()*estimate.velocity;
                local_valid=snapshot_now>=local_pose.ns&&snapshot_now-local_pose.ns<=100000000ULL;
                capture_valid=tcp_pose_history_.At(item->capture_ns,capture_pose);
            }
        });
        near_guide_.Observe(*item,now);
        // Tolerant single-frame reuse can retain the prior accepted capture.
        // Interpolate that exact capture, not the rejected frame's timestamp.
        if(capture_valid&&capture_pose.ns!=near_guide_.latest.capture_ns)capture_valid=false;
        if(!capture_valid&&local_valid&&near_guide_.latest.capture_ns!=item->capture_ns) {
            VA15ReadCoherentSnapshot(b2_state_mutex_,[]{return SteadyNowNs();},[&](std::uint64_t){
                if(pose_generation==tcp_pose_history_.generation)
                    capture_valid=tcp_pose_history_.At(near_guide_.latest.capture_ns,capture_pose);
            });
        }
        if(local_valid)near_guide_.ObserveBodyVelocity(local_body_velocity.x(),local_body_velocity.y(),
            control_dt);
        if(pose_generation!=tcp_tracker_.generation) {
            if(near_guide_.stage==va15_visual::Guide::Stage::Running)
                near_guide_.Stop("local pose sequence reset; no automatic resume");
            front_end_stop_.Interrupt("local pose generation reset; stored endpoint invalidated");
            tcp_tracker_.ClearObservations();tcp_tracker_.generation=pose_generation;
        }
        if(capture_valid&&va15_visual::Valid(near_guide_.latest,now))
            tcp_tracker_.Observe(near_guide_.latest,capture_pose);
        tcp_forward_reference_={};tcp_forward_reference_age_=0.;
        near_guide_.metric_bearing_valid=local_valid&&tcp_tracker_.Guidance(local_pose,
            near_guide_.metric_bearing,&tcp_forward_reference_);
        if(near_guide_.metric_bearing_valid&&local_pose.ns>=tcp_forward_reference_.capture_ns)
            tcp_forward_reference_age_=double(local_pose.ns-tcp_forward_reference_.capture_ns)*1.e-9;
        near_guide_.body_reference_valid=local_valid&&tcp_tracker_.BodyGuidance(local_pose,
            near_guide_.body_lateral,near_guide_.body_heading,near_guide_.body_history_used);
        tcp_local_pose_=local_pose;tcp_local_pose_.valid=local_valid;
        auto loco=std::dynamic_pointer_cast<b2_z1_no_gun_fsm::RLFSMStateRLLocomotion>(
            this->fsm.states_.at("RLFSMStateRLLocomotion"));
        const bool centered=telemetry_remote_valid_&&std::all_of(telemetry_raw_axes_.begin(),
            telemetry_raw_axes_.end(),[](float v){return std::isfinite(v)&&std::fabs(v)<=kJoystickDeadzone;});
        const bool healthy=!safety_stop_.load()&&telemetry_command_context_.contract_ready&&
            va15_trial::HealthyFeedback(this->robot_state.real_stop_feedback);
        const auto key=this->control.current_keyboard;const auto pad=this->control.current_gamepad;
        const bool stop=key==Input::Keyboard::Space||key==Input::Keyboard::Num9||key==Input::Keyboard::P||
            pad==Input::Gamepad::B||pad==Input::Gamepad::LB_X||remote_motion_inhibited_;
        const bool quiet=PolicyStandbyReady()&&loco->RealStopSettled()&&
            loco->real_start_support_cycles>=100&&!stop;
        if(key==Input::Keyboard::T)
        {
            std::cout<<LOGGER::INFO<<"[Visual gate] near_deg="<<va15_visual::Bearing(near_guide_.latest.u,near_guide_.latest.v)*57.2957795
                <<" / display "<<va15_visual::DisplayNearBearingLimit*57.2957795
                <<" look_deg="<<va15_visual::Bearing(near_guide_.latest.look_u,near_guide_.latest.look_v)*57.2957795
                <<" / display "<<va15_visual::DisplayLookBearingLimit*57.2957795
                <<" curve_ready="<<near_guide_.Ready(now)<<" quiet="<<quiet
                <<" neutral="<<centered<<" health="<<healthy<<" local_pose_valid="<<local_valid
                <<" trial_stage="<<int(loco->trial.stage)<<" stop_stage="<<int(loco->controlled_stop.stage)<<std::endl;
            if(near_guide_.stage==va15_visual::Guide::Stage::Running) {
                front_end_stop_.Interrupt("operator R1+Y/T stop");
                near_guide_.Stop("operator R1+Y/T stop");
            }
            else if(trial_motion_&&!stop&&local_valid) {
                bool session_ready=loco->trial.stage==va15_trial::Supervisor::Stage::Armed;
                if(!session_ready&&quiet&&centered&&healthy&&near_guide_.Ready(now))
                    session_ready=loco->RearmVisualSession(centered,healthy);
                if(near_guide_.Start(now,quiet&&session_ready,centered,healthy)) {
                    tcp_tracker_.Begin(local_pose,tcp_tracker_.target);
                    front_end_stop_.Begin(pose_generation);
                }
                std::cout<<LOGGER::WARNING<<"[Visual session] "<<near_guide_.reason
                    <<"; direct toggle, no four-second review, no automatic resume"<<std::endl;
            } else {
                std::cout<<LOGGER::WARNING<<"[Visual session] "<<(!local_valid?"local pose unavailable; no start":
                    "shadow/stop has priority; walking disabled")<<std::endl;
            }
            this->control.current_keyboard=Input::Keyboard::None;
        }
        const auto old=near_guide_.stage;
        va15_tcp::V3 measured_tcp_world=va15_tcp::V3::Zero();
        const bool measured_tcp_valid=local_valid&&z1_enabled_&&
            va15_tcp::MeasuredTCPWorld(local_pose,this->robot_state.motor_state.q,measured_tcp_world);
        if(stop||!centered||!healthy||!local_valid||!PolicyStandbyReady()||
           !std::isfinite(control_dt)||control_dt<=0.||control_dt>.02)
            front_end_stop_.Interrupt("operator/state/time stop; not normal visual-end completion");
        else if(near_guide_.stage==va15_visual::Guide::Stage::Running) {
            front_end_stop_.Observe(*item,capture_pose,now,capture_valid&&near_guide_.qualified>=6&&
                near_guide_.latest.sequence==item->sequence&&va15_visual::Valid(near_guide_.latest,now),pose_generation);
        }
        if(near_guide_.stage==va15_visual::Guide::Stage::Running&&
           front_end_stop_.stage==va15_visual_end::RearCompletion::Stage::Interrupted) {
            near_guide_.Stop(front_end_stop_.reason);near_output_={0.,0.,true};
        } else if(near_guide_.stage==va15_visual::Guide::Stage::Running&&front_end_stop_.Confirmed()) {
            // Explicit finite-goal authority, NOT renewed live-camera freshness.
            // No packet/capture clock is fabricated after the line leaves view.
            near_output_=front_end_stop_.Tick(now,control_dt,local_pose,measured_tcp_world,
                measured_tcp_valid,pose_generation,healthy,stop,centered,near_guide_.yaw,near_guide_.visual_integral);
            near_guide_.yaw=near_output_.yaw;
            near_guide_.visual_integral=front_end_stop_.steering_integral;
            if(near_output_.stop)near_guide_.Stop(front_end_stop_.reason);
        } else near_output_=near_guide_.Tick(now,control_dt,healthy,stop,centered);
        if(near_guide_.stage==va15_visual::Guide::Stage::Running&&!local_valid) {
            near_guide_.Stop("local pose stale/unreliable; TCP target frozen");near_output_={0.,0.,true};
        }
        const double tcp_dt=control_dt;
        if(local_valid) {
            const bool endpoint_target=front_end_stop_.Confirmed()&&
                tcp_tracker_.StepEndpoint(local_pose,tcp_dt,front_end_stop_.endpoint);
            if(!endpoint_target)tcp_tracker_.Step(local_pose,tcp_dt,
                trial_motion_&&near_guide_.stage==va15_visual::Guide::Stage::Running);
        }
        if(this->fsm.current_state_->GetStateName()=="RLFSMStateRLLocomotion") {
            // Target commands enter the ORIGINAL 70D actor observation.
            // No IK overlay, joint offset, gain change or arm-only controller.
            this->SetRuntimeTargetCommands({float(tcp_tracker_.target.x()),float(tcp_tracker_.target.y()),
                float(tcp_tracker_.target.z()),0.f,-.707107f,0.f,.707107f});
        } else {
            this->ClearRuntimeTargetCommands();tcp_tracker_.target={-.775,0.,-.35};
            tcp_tracker_.active=tcp_tracker_.ever_tracked=false;
        }
        if(near_guide_.stage==va15_visual::Guide::Stage::Running&&
           this->fsm.current_state_->GetStateName()!="RLFSMStateRLLocomotion") {
            near_guide_.Stop("policy state exited");near_output_={0.,0.,true};
        }
        if(near_output_.stop&&trial_motion_)loco->trial.Stop(near_guide_.reason);
        if(near_output_.stop&&!front_end_stop_.Requested())
            front_end_stop_.Interrupt("ordinary protective/manual stop; no confirmed visual end");
        const bool already_stood=front_end_stop_.Stood();
        front_end_stop_.Park(control_dt,healthy&&local_valid,
            loco->controlled_stop.Ready()&&loco->RealStopSettled(),measured_tcp_world,measured_tcp_valid);
        if(front_end_stop_.Stood()&&!already_stood)
            std::cout<<LOGGER::INFO<<"[Visual end] "<<front_end_stop_.reason<<std::endl;
        near_inbox_->SetPhase(!PolicyStandbyReady()?0u:
            near_guide_.stage==va15_visual::Guide::Stage::Running?2u:
            quiet&&(!loco->controlled_stop.Active()||loco->controlled_stop.Ready())?1u:3u);
        if(old!=near_guide_.stage)
            std::cout<<LOGGER::WARNING<<"[Near visual] "<<near_guide_.reason<<"; no automatic resume"<<std::endl;
        this->control.x=trial_motion_?float(near_output_.x):0.f;
        this->control.y=0.f;this->control.yaw=trial_motion_?float(near_output_.yaw):0.f;
        if(++near_report_cycles_%200==0)
            std::cout<<LOGGER::INFO<<"[Near visual] stage="<<int(near_guide_.stage)
                <<" ready="<<near_guide_.Ready(now)<<" frames="<<near_guide_.qualified
                <<" sequence="<<near_guide_.latest.sequence<<" session_count="<<near_guide_.sessions
                <<" path_points="<<near_guide_.latest.path_count<<" path_span_px="<<near_guide_.latest.path_span
                <<" near_deg="<<va15_visual::Bearing(near_guide_.latest.u,near_guide_.latest.v)*57.2957795
                <<" look_deg="<<va15_visual::Bearing(near_guide_.latest.look_u,near_guide_.latest.look_v)*57.2957795
                <<" direct_toggle=1 continuous=1 restart_ready="
                <<(quiet&&healthy&&centered&&(!loco->trial.Stopping()||loco->controlled_stop.Ready()))
                <<" would_x="<<near_output_.x<<" would_yaw="<<near_output_.yaw
                <<" TCP_flat_floor=1 active="<<tcp_tracker_.active
                <<" cache_frames="<<tcp_tracker_.curves.size()<<" target_b=["<<tcp_tracker_.target.transpose()
                <<"] clearance="<<tcp_tracker_.clearance<<" local_pose_valid="<<local_valid
                <<" course_slip_rad="<<near_guide_.course_slip
                <<" slip_command_enabled=0 body_cross_track_m="<<near_guide_.body_lateral
                <<" body_heading_rad="<<near_guide_.body_heading
                <<" body_reference_valid="<<near_guide_.body_reference_valid
                <<" body_reference_history="<<near_guide_.body_history_used
                <<" body_control_weight="<<near_guide_.body_control_weight
                <<" visual_integral="<<near_guide_.visual_integral
                <<" yaw_reference="<<near_guide_.yaw_reference
                <<" metric_guide_rad="<<near_guide_.metric_bearing<<" metric_valid="<<near_guide_.metric_bearing_valid
                <<" preview_x_m="<<tcp_forward_reference_.point_body.x()
                <<" preview_y_m="<<tcp_forward_reference_.point_body.y()
                <<" preview_history="<<tcp_forward_reference_.historical
                <<" preview_age_s="<<tcp_forward_reference_age_
                <<" cache_oldest_s="<<tcp_tracker_.cache_age
                <<" rear_crossings="<<tcp_tracker_.rear_crossings
                <<" workspace_rejections="<<tcp_tracker_.workspace_rejections
                <<" nearest_rear_gap_m="<<tcp_tracker_.nearest_rear_gap
                <<" tcp_reason="<<tcp_tracker_.reason<<"; monocular flat-floor estimate, NOT depth"<<std::endl;
        if(near_report_cycles_%200==0)
            std::cout<<LOGGER::INFO<<"[Visual end] stage="<<int(front_end_stop_.stage)
                <<" cap_frames="<<front_end_stop_.frames<<" end_seen="<<front_end_stop_.Confirmed()
                <<" measured_tcp_valid="<<measured_tcp_valid
                <<" remaining_m="<<front_end_stop_.remaining<<" lateral_m="<<front_end_stop_.lateral
                <<" endpoint_xy_error_m="<<front_end_stop_.distance<<" height_error_m="<<front_end_stop_.height_error
                <<" stood="<<front_end_stop_.Stood()<<" reason="<<front_end_stop_.reason<<std::endl;
    }

    bool GuidanceRunning() const {
        return blind_fixed_?blind_guide_.running:near_guide_.stage==va15_visual::Guide::Stage::Running;
    }

    void UpdateBlindGuide(double dt) {
        auto loco=std::dynamic_pointer_cast<b2_z1_no_gun_fsm::RLFSMStateRLLocomotion>(
            this->fsm.states_.at("RLFSMStateRLLocomotion"));
        const bool centered=telemetry_remote_valid_&&std::all_of(telemetry_raw_axes_.begin(),telemetry_raw_axes_.end(),
            [](float v){return std::isfinite(v)&&std::fabs(v)<=kJoystickDeadzone;});
        const bool healthy=!safety_stop_.load()&&telemetry_command_context_.contract_ready&&
            va15_trial::HealthyFeedback(this->robot_state.real_stop_feedback);
        const auto key=this->control.current_keyboard;const auto pad=this->control.current_gamepad;
        const bool stop=key==Input::Keyboard::Space||key==Input::Keyboard::Num9||key==Input::Keyboard::P||
            pad==Input::Gamepad::B||pad==Input::Gamepad::LB_X||remote_motion_inhibited_||!PolicyStandbyReady();
        const bool quiet=PolicyStandbyReady()&&loco->RealStopSettled()&&loco->real_start_support_cycles>=100&&!stop;
        if(key==Input::Keyboard::T) {
            if(blind_guide_.running) {blind_guide_.Stop("explicit R1+Y/T stop");loco->trial.Stop(blind_guide_.reason);}
            else if(trial_motion_&&!stop) {
                bool ready=loco->trial.stage==va15_trial::Supervisor::Stage::Armed;
                if(!ready&&quiet&&healthy&&centered)ready=loco->RearmVisualSession(centered,healthy);
                blind_guide_.Start(quiet&&ready,centered,healthy);
            }
            std::cout<<LOGGER::WARNING<<"[Blind fixed target] "<<blind_guide_.reason<<std::endl;
            this->control.current_keyboard=Input::Keyboard::None;
        }
        if(blind_guide_.Tick(dt,healthy,stop,centered))loco->trial.Stop(blind_guide_.reason);
        // Never override the original seven target observations in blind
        // mode; this is not a substitute for successful vision acceptance.
        this->ClearRuntimeTargetCommands();
        near_output_={blind_guide_.running?.15:0.,0.,false};
        this->control.x=float(near_output_.x);this->control.y=this->control.yaw=0.f;
        if(++near_report_cycles_%200==0)
            std::cout<<LOGGER::INFO<<"[Blind fixed target] running="<<blind_guide_.running
                <<" quiet="<<quiet<<" healthy="<<healthy<<" original_TCP=1 no_camera=1 reason="<<blind_guide_.reason<<std::endl;
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
        const auto begin=SteadyNowNs(),previous=telemetry_writer_previous_ns_;
        telemetry_writer_previous_ns_=begin;
        if(previous)telemetry_writer_max_interval_=std::max(telemetry_writer_max_interval_,begin-previous);
        const bool record_this=telemetry_ && begin-telemetry_writer_record_ns_>=20000000ull;
        va15_telemetry::Command record;
        unitree_go::msg::dds_::LowCmd_ command;
        std::uint64_t audit_source_ns=0;
        {
            std::lock_guard<std::mutex> lock(b2_command_mutex_);
            command = b2_low_command_;
            audit_source_ns=telemetry_published_context_.source.received_ns;
            if(record_this)record=telemetry_published_context_;
        }
        if (lowcmd_publisher_)
        {
            const bool write_ok=lowcmd_publisher_->Write(command);
            near_review_.audit.Notify(SteadyNowNs(),previous?begin-previous:0,audit_source_ns,write_ok);
            // Only begin the health warmup after OUR writer starts. Factory
            // traffic observed during Z1 initialization cannot pre-arm this
            // candidate before the takeover gap. A DDS return is not an ACK;
            // raw bit_flag must independently confirm timeout cleared.
            if(write_ok)motor_contract_stream_started_.store(true);
            if(record_this)
            {
                record.begin_ns=begin;record.end_ns=SteadyNowNs();record.previous_begin_ns=previous;
                record.max_interval_ns=telemetry_writer_max_interval_;record.write_ok=write_ok;
                for(int i=0;i<12;++i)
                {
                    const auto& m=command.motor_cmd()[i];record.cmd_q[i]=m.q();record.cmd_dq[i]=m.dq();
                    record.kp[i]=m.kp();record.kd[i]=m.kd();record.ff[i]=m.tau();
                }
                telemetry_->Push(record);telemetry_writer_record_ns_=begin;telemetry_writer_max_interval_=0;
            }
        }
    }

    void RunModel()
    {
        if (!this->rl_init_done || safety_stop_.load() || !trial_motion_)
        {
            return;
        }

        RobotState<float> state_snapshot;
        va15_telemetry::Actor actor_record;
        actor_record.begin_ns=SteadyNowNs();actor_record.previous_begin_ns=telemetry_actor_previous_ns_;
        telemetry_actor_previous_ns_=actor_record.begin_ns;
        state_snapshot.motor_state.resize(this->params.Get<int>("num_of_dofs"));
        this->GetStateTagged(&state_snapshot,&actor_record.source);

        this->episode_length_buf += 1;
        this->obs.ang_vel = state_snapshot.imu.gyroscope;
        this->obs.base_quat = state_snapshot.imu.quaternion;
        std::vector<float> target_commands = this->ApplyMotionHeadingHold({
            std::clamp(this->control.x, -kRealLinearSpeedLimit, kRealLinearSpeedLimit),
            std::clamp(this->control.y, -kRealLateralSpeedLimit, kRealLateralSpeedLimit),
            std::clamp(near_inference_yaw_.load(),-float(va15_visual::YawLimit),float(va15_visual::YawLimit))
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
        // Also clamp AFTER heading hold/minimum-speed/policy-limit shaping.
        const auto wall_deadline=trial_wall_deadline_ns_.load();
        const auto inference_now=SteadyNowNs(),heartbeat=visual_control_heartbeat_ns_.load();
        const bool allowed=trial_inference_forward_.load()&&heartbeat&&inference_now>=heartbeat&&
            inference_now-heartbeat<=50000000ULL&&(kContinuousVisual||(wall_deadline!=0&&inference_now<wall_deadline));
        target_commands[0]=allowed?std::clamp(target_commands[0],0.f,float(va15_trial::Supervisor::SpeedLimit)):0.f;
        target_commands[1]=0.f;
        target_commands[2]=allowed?std::clamp(near_inference_yaw_.load(),
            -float(va15_visual::YawLimit),float(va15_visual::YawLimit)):0.f;
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
            telemetry_policy_command_[i].store(this->obs.commands[i]);
            actor_record.command[i]=this->obs.commands[i];
        }
        this->obs.dof_pos = state_snapshot.motor_state.q;
        this->obs.dof_vel = state_snapshot.motor_state.dq;

        const std::vector<float> policy_actions = this->Forward();
        if (!this->params.Get<bool>("sync_applied_action_observation", false) &&
            !this->params.Get<bool>("use_raw_policy_action_observation", false))
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
        actor_record.end_ns=SteadyNowNs();actor_record.queued_frame=++telemetry_queued_frames_;
        actor_record.model_completion=telemetry_actor_completed_.load();
        actor_record.model_executed=telemetry_model_executed_;
        actor_record.observation=telemetry_observation_;actor_record.raw_action=telemetry_raw_action_;
        for(int i=0;i<18;++i){actor_record.filtered_action[i]=policy_actions[i];actor_record.target_q[i]=output_dof_pos[i];}
        if(telemetry_)telemetry_->Push(actor_record);
    }

    std::vector<float> Forward() override
    {
        telemetry_model_executed_=false;
        telemetry_observation_.fill(std::numeric_limits<float>::quiet_NaN());
        telemetry_raw_action_.fill(std::numeric_limits<float>::quiet_NaN());
        std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);
        if (!lock.owns_lock())
        {
            return this->params.Get<bool>("use_raw_policy_action_observation",false)
                ? va15_action_state_.Applied() : this->obs.actions;
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

        for(int i=0;i<70;++i)telemetry_observation_[i]=observation[i];
        for(int i=0;i<18;++i)telemetry_raw_action_[i]=actions[i];
        telemetry_model_executed_=true;telemetry_actor_completed_.fetch_add(1);
        if(this->params.Get<bool>("use_raw_policy_action_observation",false))
        {
            auto applied=va15_action_state_.Update(actions,this->obs.actions,
                this->params.Get<float>("action_filter_alpha",.2f),
                this->params.Get<float>("policy_input_clip",5.f),
                this->reset_policy_action_state || this->episode_length_buf==1);
            this->reset_policy_action_state=false;
            // Preserve the legacy output safety envelope, independently of
            // the training-side bounded raw history and EMA state.
            return clamp(applied,
                this->params.Get<std::vector<float>>("clip_actions_lower"),
                this->params.Get<std::vector<float>>("clip_actions_upper"));
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
        if (callbacks_shutting_down_.load()) return;
        std::lock_guard<std::mutex> lock(b2_state_mutex_);
        b2_low_state_ =
            *static_cast<const unitree_go::msg::dds_::LowState_ *>(message);
        std::array<float, 4> raw;
        for (int i = 0; i < 4; ++i) raw[i] = b2_low_state_.foot_force()[i];
        const double now = SteadyNowNs() * 1.e-9;
        b2_low_received_ns_=SteadyNowNs();
        va15_motor_contract::Raw current;
        current.tick=b2_low_state_.tick();current.bit_flag=b2_low_state_.bit_flag();
        for(int i=0;i<12;++i)
        {
            const auto& m=b2_low_state_.motor_state()[i];
            current.mode[i]=m.mode();current.lost[i]=m.lost();
            current.fault[i]=m.reserve()[0];current.frequency[i]=m.reserve()[1];
        }
        if(motor_contract_stream_started_.load())motor_contract_.Update(current,now);
        stop_feedback_tracker_.UpdateLow(raw, b2_low_state_.tick(), now);
        va15_shadow::Sample sample;
        for (int i = 0; i < 12; ++i)
        {
            sample.q[i] = b2_low_state_.motor_state()[i].q();
            sample.dq[i] = b2_low_state_.motor_state()[i].dq();
        }
        for (int i = 0; i < 4; ++i) sample.quaternion[i] = b2_low_state_.imu_state().quaternion()[i];
        for (int i = 0; i < 3; ++i) sample.gyro[i] = b2_low_state_.imu_state().gyroscope()[i];
        const auto evidence = stop_feedback_tracker_.Snapshot(now);
        sample.contact_evidence_valid = evidence.contacts_valid;
        sample.contact_dds = {{evidence.contacts[1],evidence.contacts[0],evidence.contacts[3],evidence.contacts[2]}};
        kinematics_tracker_->Update(sample, b2_low_state_.tick(), now);
        if(now-trial_observer_received_>=.009 || now<trial_observer_received_)
        {
            va15_independent::Sample measured;
            measured.documented_motor_health=motor_contract_.Snapshot(now).ready;
            for(int i=0;i<12;++i)
            {
                const auto& m=b2_low_state_.motor_state()[i];
                measured.q[i]=m.q();measured.dq[i]=m.dq();measured.tau[i]=m.tau_est();
                measured.mode[i]=m.mode();measured.lost[i]=m.lost();
            }
            const auto& imu=b2_low_state_.imu_state();
            measured.quaternion=Eigen::Quaterniond(imu.quaternion()[0],imu.quaternion()[1],imu.quaternion()[2],imu.quaternion()[3]);
            for(int i=0;i<3;++i)
            {measured.gyro[i]=imu.gyroscope()[i];measured.specific_force[i]=imu.accelerometer()[i];}
            trial_observer_->Update(measured,b2_low_state_.tick(),now);
            trial_observer_received_=now;
            const auto pose=trial_observer_->Snapshot(now);
            const auto pose_ns=std::uint64_t(now*1.e9);
            if(va15_trial::Healthy(pose)&&pose.estimate_valid_for_review)tcp_last_supported_pose_ns_=pose_ns;
            // Trot can briefly have fewer accepted foot updates. Use the
            // filter's bounded inertial prediction for <=250 ms, not a fake
            // contact or an extra four-feet requirement on every gait frame.
            if(va15_trial::Healthy(pose)&&tcp_last_supported_pose_ns_&&pose_ns>=tcp_last_supported_pose_ns_&&
               pose_ns-tcp_last_supported_pose_ns_<=250000000ULL)
                tcp_pose_history_.Add({pose_ns,pose.position,measured.quaternion.normalized(),true});
            else if(!pose.fresh||!pose.input_valid||!pose.sequence_valid||!pose.initialized)
                tcp_pose_history_.Clear();
        }
        b2_state_ready_.store(true);
    }

    void SportStateHandler(const void *message)
    {
        if (callbacks_shutting_down_.load()) return;
        const auto state =
            *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
        TimedBasePose pose;
        pose.monotonic_ns = SteadyNowNs();
        {
            const auto& stamp = state.stamp();
            const bool stamp_ok = stamp.sec() >= 0 && stamp.nanosec() < 1000000000u;
            const auto stamp_ns = stamp_ok ? static_cast<std::uint64_t>(stamp.sec()) * 1000000000ull +
                stamp.nanosec() : 0ull;
            std::lock_guard<std::mutex> lock(b2_state_mutex_);
            stop_feedback_tracker_.UpdateSport(state.body_height(), state.velocity(), stamp_ns,
                pose.monotonic_ns * 1.e-9, state.error_code() == 0);
        }
        for (int axis = 0; axis < 3; ++axis)
            pose.position[axis] = state.position()[axis];
        for (int axis = 0; axis < 4; ++axis)
            pose.quaternion[axis] = state.imu_state().quaternion()[axis];
        if (!NormalizeQuaternion(&pose.quaternion) ||
            !std::all_of(pose.position.begin(), pose.position.end(),
                         [](double value) { return std::isfinite(value); }))
            return;

        std::lock_guard<std::mutex> lock(sport_state_mutex_);
        if (!base_pose_history_.empty())
        {
            const auto &previous = base_pose_history_.back();
            const double dt = static_cast<double>(
                pose.monotonic_ns - previous.monotonic_ns) * 1.0e-9;
            const double jump = std::hypot(
                pose.position[0] - previous.position[0],
                pose.position[1] - previous.position[1]);
            // A localization reset would reinterpret every cached crack point.
            // Drop visual state rather than command a discontinuous turn.
            if (dt > 0.0 && dt < 0.25 && jump > 0.75)
            {
                base_pose_history_.clear();
                odometry_reset_detected_.store(true);
                return;
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
        sport_state_ready_.store(true);
    }

    void JoystickHandler(const void *message)
    {
        if (callbacks_shutting_down_.load()) return;
        std::lock_guard<std::mutex> lock(b2_state_mutex_);
        joystick_ =
            *static_cast<const unitree_go::msg::dds_::WirelessController_ *>(message);
    }

    std::unique_ptr<va15_telemetry::Recorder> telemetry_;
    va15_telemetry::Command telemetry_command_context_,telemetry_published_context_;
    std::uint64_t b2_low_received_ns_=0; // existing b2_state_mutex_
    std::uint64_t telemetry_command_sequence_=0; // existing b2_command_mutex_
    std::uint64_t telemetry_control_sequence_=0,telemetry_control_previous_ns_=0,telemetry_control_record_ns_=0,telemetry_control_max_interval_=0;
    std::uint64_t telemetry_writer_previous_ns_=0,telemetry_writer_record_ns_=0,telemetry_writer_max_interval_=0;
    std::uint64_t telemetry_actor_previous_ns_=0,telemetry_queued_frames_=0;
    std::atomic<std::uint64_t> telemetry_actor_completed_{0};
    std::array<std::atomic<float>,3> telemetry_policy_command_{{0.f,0.f,0.f}};
    std::array<float,3> telemetry_raw_axes_{};
    unsigned telemetry_remote_keys_=0,telemetry_remote_valid_=0;
    bool telemetry_model_executed_=false;
    std::array<float,70> telemetry_observation_{};
    std::array<float,18> telemetry_raw_action_{};
    std::unique_ptr<UNITREE_ARM::unitreeArm> z1_arm_;
    std::unique_ptr<MotionSwitcherClient> motion_switcher_;
    unitree_go::msg::dds_::LowCmd_ b2_low_command_{};
    unitree_go::msg::dds_::LowState_ b2_low_state_{};
    unitree_go::msg::dds_::WirelessController_ joystick_{};
    VA15RemoteInput remote_input_;
    VA15StopFeedbackTracker stop_feedback_tracker_; // protected by b2_state_mutex_
    std::unique_ptr<va15_shadow::Tracker> kinematics_tracker_; // same mutex
    std::unique_ptr<va15_independent::Observer> trial_observer_; // same mutex; never an actor observation
    va15_motor_contract::Guard motor_contract_{va15_motor_contract::LostPolicy::DiagnosticOnly}; // b2_state_mutex_
    std::atomic<bool> motor_contract_stream_started_{false};
    double trial_observer_received_=0.;
    bool trial_motion_=false;
    bool blind_fixed_=false;
    va15_blind::Guide blind_guide_;
    std::unique_ptr<va15_visual::Inbox> near_inbox_;
    static constexpr bool kContinuousVisual=true;
    va15_visual::Guide near_guide_{true,kContinuousVisual,true};
    va15_tcp::History tcp_pose_history_; // b2_state_mutex_; no-slip local estimate, NOT sensor truth
    std::uint64_t tcp_last_supported_pose_ns_=0; // same mutex; bounded inertial prediction only
    va15_tcp::Tracker tcp_tracker_; // control thread only
    bool getdown_visual_preempted_=false;
    va15_tcp::Pose tcp_local_pose_; // control-thread diagnostic snapshot, NOT pose authority
    va15_tcp::Tracker::ForwardReference tcp_forward_reference_;
    double tcp_forward_reference_age_=0.;
    va15_visual_end::RearCompletion front_end_stop_; // historical member name; rear FK owns completion
    va15_visual::Review near_review_;
    va15_visual::Output near_output_;
    std::atomic<float> near_inference_yaw_{0.f};
    unsigned near_report_cycles_=0;
    std::atomic<bool> trial_inference_forward_{false};
    std::atomic<std::uint64_t> trial_wall_deadline_ns_{0};
    std::atomic<std::uint64_t> visual_control_heartbeat_ns_{0};
    unsigned int kinematics_log_cycles_ = 0;
    bool remote_motion_inhibited_=true;
    std::mutex b2_state_mutex_;
    std::mutex b2_command_mutex_;
    mutable std::mutex sport_state_mutex_;
    std::deque<TimedBasePose> base_pose_history_;
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
    std::shared_ptr<VA15JoinableLoop> loop_keyboard_;
    std::shared_ptr<VA15JoinableLoop> loop_control_;
    std::shared_ptr<VA15JoinableLoop> loop_b2_writer_;
    VA15ActionState va15_action_state_;
    std::shared_ptr<VA15JoinableLoop> loop_rl_;
    std::shared_ptr<VA15JoinableLoop> loop_motion_audit_;
    std::atomic<bool> callbacks_shutting_down_{false};
    // Declared LAST: on constructor failure, these readers still close/join
    // before the state/mutexes captured by their callbacks can be destroyed.
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_state_subscriber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_subscriber_;
};

// Offline checker deliberately constructs no robot or network SDK objects.
class VA15OfflineContract final : public RL {
public:
    std::vector<float> Forward() override {return {};}
    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
};

static void CheckStandTransitionsOffline(VA15OfflineContract& rl) {
    // Pure command-buffer regression: no physics or SDK calls.
    struct Quiet {
        std::ostringstream text;
        std::streambuf* previous;
        std::ios::fmtflags flags;
        std::streamsize precision;
        Quiet():previous(std::cout.rdbuf(text.rdbuf())),
            flags(std::cout.flags()),precision(std::cout.precision()){}
        ~Quiet(){std::cout.rdbuf(previous);std::cout.flags(flags);std::cout.precision(precision);}
    } quiet;
    const auto require=[](bool ok){if(!ok)throw std::runtime_error("Stand transition regression");};
    using namespace b2_z1_no_gun_fsm;
    const auto stand=DeploymentStandPosition(rl);
    const auto stand_kp=DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
    const auto fixed_kp=rl.params.Get<std::vector<float>>("fixed_kp");
    rl.robot_state.motor_state.q=stand;
    rl.robot_state.motor_state.dq.assign(18,0.f);
    rl.robot_state.motor_state.tau_est.assign(18,0.f);
    RLFSMStateGetUp up(&rl);
    up.fsm_state=&rl.robot_state;up.fsm_command=&rl.robot_command;
    up.percent_pre_getup=up.percent_getup=up.percent_arm_deploy=1.f;
    up.stand_from_passive=false;up.work_pose_reported=true;
    for(int i=0;i<500;++i) up.Run();
    require(up.post_getup_hold_reported);
    for(int i:VA15LegIndices) require(std::fabs(rl.robot_command.motor_command.kp[i]-stand_kp[i])<1.e-4);
    up.locomotion_requested=true;
    up.percent_getup=.5f;
    rl.control.current_gamepad=Input::Gamepad::B;
    require(up.CheckChange()=="RLFSMStateGetUp" && !up.locomotion_requested);
    rl.control.current_gamepad=Input::Gamepad::None;
    up.percent_getup=1.f;
    require(up.CheckChange()=="RLFSMStateGetDown");
    // Y/1 preparation and idle must not drop back to the weak fixed gains.
    RLFSMStateRLLocomotion standby(&rl);
    standby.fsm_state=&rl.robot_state;standby.fsm_command=&rl.robot_command;
    standby.Enter();
    for(int j=0;j<401;++j) standby.Run();
    require(!rl.rl_init_done);
    for(int i:VA15LegIndices) require(std::fabs(rl.robot_command.motor_command.kp[i]-stand_kp[i])<1.e-4);
    const auto held_q=rl.robot_command.motor_command.q;
    const auto held_kp=rl.robot_command.motor_command.kp;
    const auto held_kd=rl.robot_command.motor_command.kd;
    rl.start_state=rl.robot_state;
    for(int i=9;i<=12;++i) {
        rl.robot_state.motor_state.q[i]-=.1f;
        rl.start_state.motor_state.q[i]=-2.8f;
    }
    RLFSMStateGetDown down(&rl);
    down.fsm_state=&rl.robot_state;down.fsm_command=&rl.robot_command;
    down.Enter();down.Run();
    for(int i:VA15LegIndices) {
        require(rl.robot_command.motor_command.q[i]==held_q[i]);
        require(rl.robot_command.motor_command.kp[i]==held_kp[i]);
        require(rl.robot_command.motor_command.kd[i]==held_kd[i]);
    }
    down.Run();
    for(int i:VA15LegIndices) {
        require(std::fabs(rl.robot_command.motor_command.q[i]-held_q[i])<.01f);
        require(std::fabs(rl.robot_command.motor_command.kp[i]-held_kp[i])<.1f);
    }
    for(int j=0;j<700;++j) {
        // Memory-only measured feedback follows the commanded pose. Do not
        // force completion fields: exercise actual simultaneous progress.
        rl.robot_state.motor_state.q=rl.robot_command.motor_command.q;
        rl.robot_state.motor_state.dq.assign(18,0.f);
        down.Run();
        if(j==200) require(down.body_progress>0. && down.arm_progress<1.);
    }
    require(down.complete && down.CheckChange()=="RLFSMStatePassive");
    for(int i:VA15LegIndices) require(std::fabs(rl.robot_command.motor_command.kp[i]-fixed_kp[i])<1.e-4);
}

static int CheckVA15Offline() {
    VA15OfflineContract rl;
    rl.robot_name="b2_z1_no_gun";
    rl.ReadYaml(rl.robot_name,"base.yaml");
    const auto profile=rl.params.Get<std::string>("deployment_policy_profile","robot_lab");
    if(profile!="VA15") throw std::runtime_error("Expected explicit VA15 profile");
    rl.InitRL(rl.robot_name+"/"+profile);
    auto golden=YAML::LoadFile(std::string(POLICY_DIR)+"/b2_z1_no_gun/VA15/golden.json");
    double max_error=0.;
    bool close=true;
    for(std::size_t j=0;j<golden["observations"].size();++j) {
        auto input=golden["observations"][j].as<std::vector<float>>();
        auto expected=golden["actions"][j].as<std::vector<float>>();
        auto actual=rl.model->forward({input});
        if(input.size()!=70 || actual.size()!=18 || expected.size()!=18)
            throw std::runtime_error("VA15 native dimension mismatch");
        for(int i=0;i<18;++i) {
            if(!std::isfinite(actual[i])) throw std::runtime_error("Nonfinite native actor");
            max_error=std::max(max_error,std::fabs(double(actual[i]-expected[i])));
            close=close && std::fabs(double(actual[i]-expected[i])) <=
                1.e-5 + 2.e-6*std::fabs(double(expected[i]));
        }
    }
    std::cout<<"VA15 native golden max error="<<max_error<<std::endl;
    // Float32 GEMV differs slightly between the export PyTorch and bundled
    // LibTorch versions; use explicit absolute + relative tolerances.
    if(!close) throw std::runtime_error("VA15 native golden mismatch");
    rl.obs.dof_pos=rl.params.Get<std::vector<float>>("default_dof_pos");
    rl.obs.dof_vel.assign(18,0.f);rl.obs.actions.assign(18,0.f);
    rl.obs.commands.assign(3,0.f);rl.obs.ang_vel.assign(3,0.f);
    rl.obs.base_quat={1.f,0.f,0.f,0.f};
    auto obs=rl.ComputeObservation();
    if(obs.size()!=70) throw std::runtime_error("VA15 assembled observation mismatch");
    CheckStandTransitionsOffline(rl);
    {
        // Memory-only board regression: no DDS, UDP, service or constructor.
        auto state=rl.robot_state;
        auto old=rl.robot_command;
        state.motor_state.q=b2_z1_no_gun_fsm::DeploymentStandPosition(rl);
        state.motor_state.dq.assign(18,0.f);
        old.motor_command.q=state.motor_state.q;
        old.motor_command.kp=b2_z1_no_gun_fsm::DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
        old.motor_command.kd=b2_z1_no_gun_fsm::DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
        old.motor_command.dq.assign(18,0.f);old.motor_command.tau.assign(18,0.f);
        for(int i:VA15LegIndices)state.motor_state.q[i]-=.08f;
        auto policy=old;
        policy.motor_command.kp=rl.params.Get<std::vector<float>>("rl_kp");
        policy.motor_command.kd=rl.params.Get<std::vector<float>>("rl_kd");
        VA15PolicyHandoff join;join.Begin(old);
        const auto lower=rl.params.Get<std::vector<float>>("leg_target_safety_lower");
        const auto upper=rl.params.Get<std::vector<float>>("leg_target_safety_upper");
        const auto limits=rl.params.Get<std::vector<float>>("torque_limits");
        auto out=policy;
        if(!join.Apply(state,out,false,.005,lower,upper,limits)||out.motor_command.q!=old.motor_command.q||out.motor_command.kp!=old.motor_command.kp)
            throw std::runtime_error("Native pending actor hold regression");
        out=policy;
        if(!join.Apply(state,out,true,.005,lower,upper,limits))throw std::runtime_error("Native handoff refused");
        for(int i:VA15LegIndices) {
            const float old_tau=old.motor_command.kp[i]*(old.motor_command.q[i]-state.motor_state.q[i]);
            const float new_tau=out.motor_command.kp[i]*(out.motor_command.q[i]-state.motor_state.q[i]);
            if(std::fabs(old_tau-new_tau)>.001f||out.motor_command.kp[i]!=160.f)
                throw std::runtime_error("Native support torque discontinuity");
        }
        va15_blind::Guide blind;
        if(blind.Start(false,true,true)||!blind.Start(true,true,true)||!blind.Tick(.005,true,true,true)||blind.running)
            throw std::runtime_error("Native explicit blind guide regression");
        std::cout<<"PASS: V2.4.2 pending-frame hold, 675->160 torque-equivalent support, explicit no-camera guide stop; no transport initialized"<<std::endl;
    }
    std::cout<<"PASS: stand gain ramp, command continuity and parallel arm/B2 lowering (offline buffers only)"<<std::endl;
    // Exercise the actual linked stop implementation and clock helper on
    // PC4 too. These are MEMORY-ONLY inputs, never real contact overrides.
    {
        std::mutex source_mutex;
        std::uint64_t evaluation=0;
        VA15ReadCoherentSnapshot(source_mutex,[]{return 1000005000ULL;},
            [&](std::uint64_t stamp){evaluation=stamp;});
        if(evaluation!=1000005000ULL)throw std::runtime_error("Coherent snapshot clock failed");
        RobotState<float> state;RobotCommand<float> command;
        state.motor_state.q=b2_z1_no_gun_fsm::DeploymentStandPosition(rl);state.motor_state.dq.assign(18,0.f);
        state.imu.quaternion={1.f,0.f,0.f,0.f};state.imu.gyroscope={0.f,0.f,0.f};
        command.motor_command.q=state.motor_state.q;
        command.motor_command.dq.assign(18,0.f);command.motor_command.tau.assign(18,0.f);
        command.motor_command.kp=b2_z1_no_gun_fsm::DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
        command.motor_command.kd=b2_z1_no_gun_fsm::DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
        const auto stand_q=command.motor_command.q,kp=command.motor_command.kp,kd=command.motor_command.kd;
        auto& f=state.real_stop_feedback;
        f.lowstate_fresh=f.contacts_valid=f.height_valid=f.velocity_valid=true;
        f.sport_fresh=false;f.unavailable_reason=va15_trial::Source;
        f.contacts.fill(true);f.base_height=.52;f.planar_speed=0.;
        VA15ControlledStop stop;stop.ObserveStanding(state,command);stop.Request();
        const auto step=[&]{stop.Step(state,command,stand_q,kp,kd,.005,0.f);};
        step();step();
        if(!stop.captured)throw std::runtime_error("Offline supported catch failed");
        const auto held=command.motor_command.q;
        f.contacts_valid=f.height_valid=f.velocity_valid=false;f.unavailable_reason=va15_trial::HealthOnly;
        f.base_height=f.planar_speed=std::numeric_limits<float>::quiet_NaN();
        for(int i=0;i<200;++i)step();
        if(stop.stage!=VA15ControlledStop::Stage::CatchStand||stop.Ready()||held!=command.motor_command.q)
            throw std::runtime_error("Model confidence gap incorrectly latched or lowered");
        f.contacts_valid=f.height_valid=f.velocity_valid=true;f.unavailable_reason=va15_trial::Source;
        f.base_height=.52;f.planar_speed=0.;
        for(int i=0;i<600;++i)step();
        if(!stop.Ready())throw std::runtime_error("Fresh restored support did not requalify");
        stop.Reset();stop.ObserveStanding(state,command);stop.Request();step();step();
        f.lowstate_fresh=false;step();
        if(stop.stage!=VA15ControlledStop::Stage::Blocked)throw std::runtime_error("Actual stale source did not latch");
        std::cout<<"PASS: V2.4.1 coherent snapshot clock; model gap holds/requalifies; actual stale source still blocks (memory only)"<<std::endl;
    }
    va15_visual::Guide guide;
    constexpr std::uint64_t visual_now=10000000000ULL;
    for(unsigned n=1;n<=6;++n) {
        va15_visual::Packet p;p.valid=p.session_matched=true;p.sequence=n;
        p.capture_ns=visual_now-(6-n)*150000000ULL-100000000ULL;p.publish_ns=p.capture_ns+99000000ULL;
        p.u=935.;p.v=972.;p.confidence=.99;p.width=30.;p.contrast=.8;p.rms=1.;
        p.look_u=935.;p.look_v=.76*1080.;p.look_confidence=.99;p.path_count=50;p.path_span=300.;
        guide.Observe(p,p.publish_ns+1000000ULL);
    }
    if(!guide.Start(visual_now,true,true,true)||
       guide.Tick(visual_now+5000000ULL,.005,true,false,true).x!=.15||
       !guide.Tick(visual_now+va15_visual::RequestNs,.005,true,false,true).stop||
       !(va15_visual::Bearing(900.,972.)>0.&&va15_visual::Bearing(980.,972.)<0.))
        throw std::runtime_error("Native near-bearing contract regression");
    std::cout<<"PASS: native calibrated bearing signs, six-frame entry and latched four-second deadline (no sensor initialized)"<<std::endl;
    va15_motor_contract::Guard lost_guard{va15_motor_contract::LostPolicy::DiagnosticOnly};
    va15_motor_contract::Raw raw;raw.bit_flag=4;raw.mode.fill(10);raw.frequency.fill(535);
    for(unsigned n=0;n<=300;++n){raw.tick=n*10;raw.lost[4]=raw.lost[7]=n%7;lost_guard.Update(raw,double(n)/100.);}
    if(!lost_guard.Snapshot(3.).ready||!lost_guard.Snapshot(3.).lost_diagnostic_only)
        throw std::runtime_error("Diagnostic-only lost native regression");
    raw.tick=3010;raw.lost[4]=raw.lost[7]=5;lost_guard.Update(raw,3.01);
    if(!lost_guard.Snapshot(3.01).ready||raw.lost[4]!=5)
        throw std::runtime_error("Native lost-only transition refused");
    raw.tick=3020;++raw.lost[4];raw.fault[4]=1;lost_guard.Update(raw,3.02);
    if(lost_guard.Snapshot(3.02).ready||!lost_guard.Snapshot(3.02).latched||
       lost_guard.Snapshot(3.02).reason!=va15_motor_contract::Reason::MotorFault)
        throw std::runtime_error("Diagnostic-only lost masked a real fault");
    std::cout<<"PASS: V2.2 LOST_DIAGNOSTIC_ONLY; raw lost retained, real motor fault still latches (no transport initialized)"<<std::endl;
    va15_visual::Guide direct(true,true);va15_trial::Supervisor continuous;
    continuous.Configure(true,true,true);
    for(int k=0;k<100;++k)continuous.Step(.005,0.,0.,0.,true,true,false);
    auto clock=visual_now;unsigned frame=0;
    auto observe_direct=[&]{
        va15_visual::Packet p;p.valid=p.session_matched=true;p.sequence=++frame;
        p.capture_ns=clock-1000000;p.publish_ns=clock;p.u=p.look_u=935.;p.v=972.;p.look_v=.76*1080.;
        p.confidence=p.look_confidence=.99;p.width=30.;p.contrast=.8;p.rms=1.;p.path_count=50;p.path_span=300.;
        direct.Observe(p,clock);
    };
    for(int cycle=0;cycle<2;++cycle) {
        for(int k=0;k<6;++k){clock+=150000000;observe_direct();}
        if(cycle&&!continuous.Rearm(true,true,true,true))throw std::runtime_error("Native continuous rearm failed");
        if(!direct.Start(clock,true,true,true))throw std::runtime_error("Native direct toggle failed");
        for(int k=0;k<1200;++k){clock+=5000000;if(k%30==0)observe_direct();
            const auto out=direct.Tick(clock,.005,true,false,true);
            if(out.x!=.15||continuous.Step(.005,out.x,0.,0.,true,true,false)!=.15)
                throw std::runtime_error("Native continuous session unexpectedly stopped");}
        direct.Stop("explicit stop");continuous.Stop("explicit stop");
        if(direct.Tick(clock,.005,true,false,true).x!=0.)throw std::runtime_error("Native toggle failed to stop");
    }
    std::cout<<"PASS: V2.3 single-event start, 2 continuous 6 s sessions and explicit restart/stop; no static review or R key (no transport initialized)"<<std::endl;
    std::cout<<"PASS: native VA15 model_5000; 70 observations / 18 actions; golden max error="
             <<max_error<<"; no DDS/UDP/robot initialized"<<std::endl;
    return 0;
}

int main(int argc, char **argv)
{
    if(argc==2 && std::string(argv[1])=="--offline-check") {
        try {return CheckVA15Offline();}
        catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
    }
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0]
                  << " network_interface (--trial-preview|--forward-trial)"
                  << " --guide-exchange-dir DIR --guide-session UNIQUE_32_HEX"
                  << " [--z1-ip IP] [--z1-to-port PORT] [--z1-own-port PORT]"
                  << " | --offline-check"
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
        bool enable_real_vision = false;
        bool trial_motion=false,trial_preview=false,blind_fixed=false;
        std::string near_directory,near_session;
        std::string vision_exchange_dir = "/tmp/b2_z1_real_vision";
        for (int i = 2; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if(arg=="--trial-preview")trial_preview=true;
            else if(arg=="--forward-trial")trial_motion=true;
            else if(arg=="--blind-fixed-target")blind_fixed=true;
            else if(arg=="--guide-exchange-dir"&&i+1<argc)near_directory=argv[++i];
            else if(arg=="--guide-session"&&i+1<argc)near_session=argv[++i];
            else if (arg == "--enable-motion-switcher")
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
        if(trial_motion==trial_preview || enable_real_vision || !enable_motion_switcher||
           (blind_fixed?(!trial_motion||!near_directory.empty()||!near_session.empty()):(near_directory.empty()||near_session.empty())))
            throw std::runtime_error("Use the near-visual launcher: exactly one preview/motion mode and unique guide session required; old metric follower forbidden");
        RLRealB2Z1NoGun robot(
            argv[1], enable_motion_switcher, z1_ip, z1_to_port, z1_own_port,
            z1_skip_lowcmd, preflight_only, disable_z1,
            b2_kp_scale, b2_kd_scale, enable_real_vision,
            vision_exchange_dir,trial_motion,near_directory,near_session,blind_fixed);
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
