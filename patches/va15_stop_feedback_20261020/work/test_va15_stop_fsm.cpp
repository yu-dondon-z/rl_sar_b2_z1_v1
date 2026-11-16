#include "fsm_b2_z1_no_gun.hpp"
#include <cassert>
#include <sstream>

using namespace b2_z1_no_gun_fsm;

class TestRL final : public RL
{
public:
    bool real = true;
    bool RequiresRealStopFeedback() const override { return real; }
    std::vector<float> Forward() override { return {}; }
    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
};

static VA15StopFeedback Good()
{
    VA15StopFeedback value;
    value.lowstate_fresh = value.sport_fresh = value.contacts_valid = true;
    value.height_valid = value.velocity_valid = true;
    value.base_height = .52f; value.planar_speed = 0.f;
    value.contacts.fill(true); value.unavailable_reason = "none (synthetic test)";
    return value;
}

static void ClearInput(TestRL& rl)
{
    rl.control.current_keyboard = rl.control.last_keyboard = Input::Keyboard::None;
    rl.control.current_gamepad = rl.control.last_gamepad = Input::Gamepad::None;
}

static void Init(TestRL& rl)
{
    rl.robot_name = "b2_z1_no_gun";
    rl.ReadYaml(rl.robot_name, "base.yaml");
    rl.InitRL(rl.robot_name + "/VA15");
    rl.robot_state.motor_state.q = DeploymentStandPosition(rl);
    rl.robot_state.motor_state.dq.assign(18, 0.f);
    rl.robot_state.motor_state.tau_est.assign(18, 0.f);
    rl.robot_state.real_stop_feedback = Good();
    const auto kp = DeploymentStandGain(rl, "deployment_stand_kp", "fixed_kp");
    const auto kd = DeploymentStandGain(rl, "deployment_stand_kd", "fixed_kd");
    rl.robot_command.motor_command.q = rl.robot_state.motor_state.q;
    rl.robot_command.motor_command.kp = kp; rl.robot_command.motor_command.kd = kd;
}

static void Ready(TestRL& rl, RLFSMStateRLLocomotion& state)
{
    state.fsm_state = &rl.robot_state; state.fsm_command = &rl.robot_command;
    state.Enter();
    for (int i = 0; i < 501; ++i) state.Run();
    assert(!state.policy_running && !rl.rl_init_done);
}

int main()
{
    struct Quiet {
        std::ostringstream stream; std::streambuf* previous;
        Quiet() : previous(std::cout.rdbuf(stream.rdbuf())) {}
        ~Quiet() { std::cout.rdbuf(previous); }
    };
    {
        Quiet quiet;
        TestRL rl; Init(rl);
        RLFSMStateRLLocomotion state(&rl); Ready(rl, state);
        // Reproduce the attached log: default height=0, feet=0000. A new
        // policy must not be allowed to start with no possible stopping gate.
        rl.robot_state.real_stop_feedback = VA15StopFeedback{};
        // Even a numerically quiet kinematic hypothesis must not replace
        // missing measured/validated support evidence in the real gate.
        auto& shadow = rl.robot_state.kinematics_shadow;
        shadow.input_valid = shadow.fresh = shadow.sequence_valid = true;
        shadow.hypothesis_consistent = shadow.conditional_stance_valid = true;
        shadow.hypothesis_height = .52;
        shadow.hypothesis_velocity_world.setZero();
        shadow.accepted_contacts = 4;
        rl.control.x = .1f; state.Run();
        assert(!state.policy_running && !rl.rl_init_done && rl.control.x == 0.f);
        assert(state.real_start_inhibited);
        // A held joystick request cannot auto-start when feedback recovers.
        rl.robot_state.real_stop_feedback = Good();
        for (int i = 0; i < 110; ++i) { rl.control.x = .1f; state.Run(); }
        assert(!state.policy_running);
        rl.control.x = 0.f; state.Run();
        rl.control.x = .1f; state.Run();
        assert(state.policy_running && rl.rl_init_done);
        // A/Y cannot jump from a moving actor into GetUp/reinitialization.
        rl.control.current_gamepad = Input::Gamepad::A;
        assert(state.CheckChange() == "RLFSMStateRLLocomotion");
        rl.control.current_gamepad = Input::Gamepad::RB_DPadUp;
        assert(state.CheckChange() == "RLFSMStateRLLocomotion");
        ClearInput(rl);
        // Missing support never becomes an upright stop after a timeout.
        rl.robot_state.real_stop_feedback.contacts[2] = false;
        rl.control.current_gamepad = Input::Gamepad::B;
        state.Run(); ClearInput(rl);
        for (int i = 0; i < 620; ++i) state.Run();
        assert(state.getdown_requested && state.policy_running);
        assert(state.CheckChange() == "RLFSMStateRLLocomotion");
        // Restore fresh calibrated four-foot evidence. B remains latched,
        // no second press is necessary, and actor stops after the full window.
        rl.robot_state.real_stop_feedback = Good();
        for (int i = 0; i < 29; ++i) state.Run();
        assert(state.policy_running);
        // Pure PD continuity: synthesize the previously applied leg torque.
        for (int i : VA15LegIndices)
            rl.robot_state.motor_state.tau_est[i] = rl.robot_command.motor_command.kp[i] *
                (rl.robot_command.motor_command.q[i] - rl.robot_state.motor_state.q[i]);
        const auto before_tau = rl.robot_state.motor_state.tau_est;
        state.Run();
        assert(!state.policy_running && !rl.rl_init_done && state.captured_support_hold_active);
        for (int i : VA15LegIndices)
        {
            const auto& cmd = rl.robot_command.motor_command;
            const float after = cmd.kp[i] * (cmd.q[i] - rl.robot_state.motor_state.q[i]);
            assert(std::fabs(after - before_tau[i]) < .001f);
        }
        assert(state.CheckChange() == "RLFSMStateRLLocomotion");
        for (int i = 0; i < 400; ++i) state.Run();
        assert(state.CheckChange() == "RLFSMStateGetDown");
        const auto held = rl.robot_command.motor_command;
        RLFSMStateGetDown down(&rl);
        down.fsm_state = &rl.robot_state; down.fsm_command = &rl.robot_command;
        down.Enter(); down.Run();
        for (int i : VA15LegIndices)
        {
            assert(rl.robot_command.motor_command.q[i] == held.q[i]);
            assert(rl.robot_command.motor_command.kp[i] == held.kp[i]);
            assert(rl.robot_command.motor_command.kd[i] == held.kd[i]);
        }
    }
    {
        Quiet quiet;
        TestRL rl; Init(rl);
        RLFSMStateRLLocomotion state(&rl); Ready(rl, state);
        // Y standby + B still works when no calibrated support exists,
        // because no moving actor was ever started.
        rl.robot_state.real_stop_feedback = {};
        rl.control.current_gamepad = Input::Gamepad::B;
        state.Run();
        assert(state.CheckChange() == "RLFSMStateGetDown");
        // Emergency input has priority even with a pending B.
        rl.control.current_keyboard = Input::Keyboard::P;
        assert(state.CheckChange() == "RLFSMStatePassive");
    }
    {
        Quiet quiet;
        TestRL rl; rl.real = false; Init(rl);
        RLFSMStateRLLocomotion state(&rl); Ready(rl, state);
        rl.fsm.AddState(std::make_shared<RLFSMStateGetUp>(&rl));
        rl.fsm.AddState(std::make_shared<RLFSMStateGetDown>(&rl));
        rl.fsm.current_state_ = std::make_shared<RLFSMStateRLLocomotion>(&rl);
        state.policy_running = true; rl.rl_init_done = true;
        rl.obs.lin_vel = {0, 0, 0}; rl.obs.base_height = .52f;
        rl.obs.foot_contacts = {1, 1, 1, 1};
        state.policy_stop_stable_cycles = state.kPolicyStopStableCycles - 1;
        rl.control.current_gamepad = Input::Gamepad::B;
        state.Run();
        assert(rl.fsm.next_state_->GetStateName() == "RLFSMStateGetDown");
    }
    {
        Quiet quiet;
        TestRL rl; Init(rl);
        RLFSMStateRLLocomotion state(&rl); Ready(rl, state);
        rl.control.x = .1f; state.Run();
        assert(state.policy_running);
        rl.control.x = 0.f;
        for (int i = 0; i < 30; ++i) state.Run();
        assert(!state.policy_running && !state.getdown_requested);
        for (int i = 0; i < 401; ++i) state.Run();
        // X stop does not manufacture a lowering request; a later B still
        // transitions normally, and no actor output keeps replaying at rest.
        assert(state.CheckChange() == "RLFSMStateRLLocomotion");
        rl.control.current_gamepad = Input::Gamepad::B;
        state.Run();
        assert(state.CheckChange() == "RLFSMStateGetDown");
    }
    std::cout << "PASS: real start gate, held-request rearm, missing-support timeout, B latch, "
                 "stop PD continuity, arm-stow support preservation, standby B, same-frame B priority, emergency priority\n";
}
