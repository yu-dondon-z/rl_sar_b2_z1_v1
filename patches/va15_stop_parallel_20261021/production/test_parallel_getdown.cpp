#include "fsm_b2_z1_no_gun.hpp"
#include <sstream>
using namespace b2_z1_no_gun_fsm;

class OfflineRL final: public RL
{
public:
    bool RequiresRealStopFeedback()const override{return true;}
    std::vector<float> Forward()override{return {};}
    void GetState(RobotState<float>*)override{}
    void SetCommand(const RobotCommand<float>*)override{}
};
static void Init(OfflineRL& rl,RLFSMStateGetDown& state)
{
    rl.robot_name="b2_z1_no_gun";rl.ReadYaml(rl.robot_name,"base.yaml");rl.InitRL(rl.robot_name+"/VA15");
    auto stand=DeploymentStandPosition(rl);
    SetArmPosition(stand,rl.params.Get<std::vector<float>>("policy_standby_arm_pos"));
    rl.robot_state.motor_state.q=stand;rl.robot_state.motor_state.dq.assign(18,0.f);
    rl.start_state.motor_state.q=stand;
    for(int i=0;i<4;++i){rl.start_state.motor_state.q[i]=i%2==0?.30f:-.30f;
        rl.start_state.motor_state.q[4+i]=1.21f;rl.start_state.motor_state.q[9+i]=-2.80f;}
    auto& c=rl.robot_command.motor_command;c.q=stand;c.dq.assign(18,0.f);c.tau.assign(18,0.f);
    c.kp=DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
    c.kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
    state.fsm_state=&rl.robot_state;state.fsm_command=&rl.robot_command;state.Enter();
}
int main()
{
    std::ostringstream quiet;auto* previous=std::cout.rdbuf(quiet.rdbuf());
    {
        OfflineRL rl;RLFSMStateGetDown state(&rl);Init(rl,state);
        const auto entry=rl.robot_command.motor_command;state.Run();
        assert(rl.robot_command.motor_command.q==entry.q&&rl.robot_command.motor_command.kp==entry.kp&&
            rl.robot_command.motor_command.kd==entry.kd&&rl.robot_command.motor_command.dq==entry.dq&&
            rl.robot_command.motor_command.tau==entry.tau);
        for(int i=0;i<400;++i){rl.robot_state.motor_state.q=rl.robot_command.motor_command.q;state.Run();}
        assert(state.body_progress>.6&&state.arm_progress>.6&&state.arm_progress<1.);
        assert(!state.complete); // body already descending before stow completion
        for(int i=0;i<300;++i){rl.robot_state.motor_state.q=rl.robot_command.motor_command.q;state.Run();}
        assert(state.complete&&state.CheckChange()=="RLFSMStatePassive");
    }
    {
        OfflineRL rl;RLFSMStateGetDown state(&rl);Init(rl,state);state.Run();
        for(int i=0;i<2000;++i)state.Run(); // frozen arm cannot force full lowering or passive
        assert(state.body_paused&&state.body_progress<.5&&!state.complete&&state.CheckChange()=="RLFSMStateGetDown");
        const auto held=rl.robot_command.motor_command.q;state.Run();
        assert(state.blocked&&rl.robot_command.motor_command.q==held);
        rl.control.current_gamepad=Input::Gamepad::LB_X;
        assert(state.CheckChange()=="RLFSMStatePassive"); // emergency still has priority
    }
    {
        OfflineRL rl;RLFSMStateGetDown state(&rl);Init(rl,state);
        const auto held=rl.robot_command.motor_command.q;
        rl.robot_state.motor_state.q[13]=std::numeric_limits<float>::quiet_NaN();state.Run();
        assert(state.blocked&&!state.complete&&rl.robot_command.motor_command.q==held);
    }
    std::cout.rdbuf(previous);
    std::cout<<"PASS: command continuity, simultaneous start, measured lag pauses descent, no timed passive release, invalid input and emergency priority\n";
}
