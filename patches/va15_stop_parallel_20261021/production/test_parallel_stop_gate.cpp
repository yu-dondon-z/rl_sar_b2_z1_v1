#include "fsm_b2_z1_no_gun.hpp"
#include <cassert>
#include <sstream>
using namespace b2_z1_no_gun_fsm;

// All inputs below are memory-only fixtures, never sensors or robot clients.
class GateRL final : public RL {
public:
    bool RequiresRealStopFeedback() const override { return true; }
    std::vector<float> Forward() override { return {}; }
    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
};
static VA15StopFeedback Good() {
    VA15StopFeedback v;
    v.lowstate_fresh=v.sport_fresh=v.contacts_valid=v.height_valid=v.velocity_valid=true;
    v.contacts.fill(true);v.base_height=.52f;v.planar_speed=0.f;
    return v;
}
static void Init(GateRL& rl,RLFSMStateRLLocomotion& state) {
    rl.robot_name="b2_z1_no_gun";rl.ReadYaml(rl.robot_name,"base.yaml");rl.InitRL(rl.robot_name+"/VA15");
    rl.robot_state.motor_state.q=DeploymentStandPosition(rl);
    rl.robot_state.motor_state.dq.assign(18,0.f);rl.robot_state.motor_state.tau_est.assign(18,0.f);
    rl.robot_state.imu.quaternion={1,0,0,0};rl.robot_state.imu.gyroscope={0,0,0};
    rl.robot_state.real_stop_feedback=Good();rl.start_state=rl.robot_state;
    for(int i=0;i<4;++i){rl.start_state.motor_state.q[4+i]=1.21f;rl.start_state.motor_state.q[9+i]=-2.8f;}
    auto& c=rl.robot_command.motor_command;c.q=rl.robot_state.motor_state.q;
    c.dq.assign(18,0.f);c.tau.assign(18,0.f);
    c.kp=DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
    c.kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
    state.fsm_state=&rl.robot_state;state.fsm_command=&rl.robot_command;state.Enter();
    for(int i=0;i<501;++i)state.Run();
    assert(!state.policy_running&&!rl.rl_init_done);
}
int main() {
    std::ostringstream output;auto* old=std::cout.rdbuf(output.rdbuf());
    {
        GateRL rl;RLFSMStateRLLocomotion state(&rl);Init(rl,state);
        rl.robot_state.real_stop_feedback={};
        for(int i=0;i<120;++i){rl.control.x=.1f;state.Run();}
        assert(!state.policy_running&&!rl.rl_init_done&&state.real_start_inhibited&&rl.control.x==0.f);
        rl.robot_state.real_stop_feedback=Good();
        for(int i=0;i<110;++i){rl.control.x=.1f;state.Run();}
        assert(!state.policy_running); // a held request cannot silently re-arm
        rl.control.x=0;state.Run();rl.control.x=.1f;state.Run();
        assert(state.policy_running&&rl.rl_init_done);
        rl.robot_state.real_stop_feedback={};rl.control.current_gamepad=Input::Gamepad::B;
        state.Run();rl.control.current_gamepad=Input::Gamepad::None;
        for(int i=0;i<1800;++i)state.Run();
        assert(state.getdown_requested&&!state.controlled_stop.Ready());
        assert(state.CheckChange()=="RLFSMStateRLLocomotion");
        rl.control.current_keyboard=Input::Keyboard::P;
        assert(state.CheckChange()=="RLFSMStatePassive");
    }
    {
        GateRL rl;RLFSMStateRLLocomotion state(&rl);Init(rl,state);
        rl.robot_state.real_stop_feedback={};
        rl.control.current_gamepad=Input::Gamepad::B;state.Run();
        assert(!state.policy_running&&state.CheckChange()=="RLFSMStateGetDown");
        RLFSMStateGetDown down(&rl);down.fsm_state=&rl.robot_state;down.fsm_command=&rl.robot_command;
        const auto before=rl.robot_command.motor_command;
        down.Enter();down.Run();assert(rl.robot_command.motor_command.q==before.q);
        for(int i=0;i<700;++i){rl.robot_state.motor_state.q=rl.robot_command.motor_command.q;down.Run();}
        assert(down.complete&&down.CheckChange()=="RLFSMStatePassive");
    }
    std::cout.rdbuf(old);
    std::cout<<"PASS: unchanged motion gate, explicit rearm, invalid moving-stop never lowers, standby B parallel lowering and emergency priority (memory only)\n";
}
