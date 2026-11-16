#include "fsm_b2_z1_no_gun.hpp"
#include <cassert>
#include <sstream>

using namespace b2_z1_no_gun_fsm;

class OfflineRL final:public RL
{
public:
    bool RequiresRealStopFeedback() const override{return true;}
    std::vector<float> Forward() override{return {};}
    void GetState(RobotState<float>*) override{}
    void SetCommand(const RobotCommand<float>*) override{}
};

static void Init(OfflineRL& rl)
{
    rl.robot_name="b2_z1_no_gun";
    rl.ReadYaml(rl.robot_name,"base.yaml");
    rl.InitRL(rl.robot_name+"/VA15");
    auto& state=rl.robot_state;
    state.motor_state.q=DeploymentStandPosition(rl);
    state.motor_state.dq.assign(18,0.f);
    state.imu.quaternion={1,0,0,0}; state.imu.gyroscope={0,0,0};
    auto& v=state.real_stop_feedback;
    v.lowstate_fresh=v.sport_fresh=v.contacts_valid=v.height_valid=v.velocity_valid=true;
    v.base_height=.52f; v.planar_speed=0.f; v.contacts.fill(true);
    auto& c=rl.robot_command.motor_command;
    c.q=state.motor_state.q; c.dq.assign(18,0.f); c.tau.assign(18,0.f);
    c.kp=DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
    c.kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
}

static void Tick(VA15ControlledStop& stop,OfflineRL& rl,float command=0.f)
{
    stop.Step(rl.robot_state,rl.robot_command,DeploymentStandPosition(rl),
        DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp"),
        DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd"),.005,command);
}
static void Prepare(VA15ControlledStop& stop,OfflineRL& rl)
{stop.ObserveStanding(rl.robot_state,rl.robot_command);assert(stop.stand_reference_valid);stop.Request();}

int main()
{
    std::ostringstream quiet;auto* old=std::cout.rdbuf(quiet.rdbuf());
    {
        OfflineRL rl;Init(rl); VA15ControlledStop stop;Prepare(stop,rl);
        // Residual moving joints prevent the OLD quiet-stop condition, but a
        // genuine supported catch may stop actor replay first.
        rl.robot_state.motor_state.dq[4]=.8f;
        auto& c=rl.robot_command.motor_command;
        c.q[4]+=.08f;c.dq[4]=.2f;c.tau[4]=2.f;
        const float before=c.kp[4]*(c.q[4]-rl.robot_state.motor_state.q[4])+
            c.kd[4]*(c.dq[4]-.8f)+c.tau[4];
        Tick(stop,rl);assert(!stop.captured);Tick(stop,rl);
        assert(stop.captured && !stop.Ready());
        const auto& after=rl.robot_command.motor_command;
        const float torque=after.kp[4]*(after.q[4]-rl.robot_state.motor_state.q[4])-
            after.kd[4]*rl.robot_state.motor_state.dq[4]+after.tau[4];
        assert(std::fabs(torque-before)<1.e-4f);
        for(int i=0;i<450;++i)Tick(stop,rl);
        assert(!stop.Ready()); // captured is NOT permission to lower
        rl.robot_state.motor_state.dq[4]=0.f;
        for(int i=0;i<100;++i)Tick(stop,rl);
        assert(stop.Ready());
    }
    for(int fault=0;fault<7;++fault)
    {
        OfflineRL rl;Init(rl); VA15ControlledStop stop;Prepare(stop,rl);
        auto& v=rl.robot_state.real_stop_feedback;
        if(fault==0)v.contacts_valid=false;
        if(fault==1)v.velocity_valid=false;
        if(fault==2)v.height_valid=false;
        if(fault==3)v.lowstate_fresh=false;
        if(fault==4)v.sport_fresh=false;
        if(fault==5)v.contacts[2]=false;
        if(fault==6)v.planar_speed=.2f;
        for(int i=0;i<1700;++i)Tick(stop,rl);
        assert(!stop.captured && !stop.Ready() && stop.stage==VA15ControlledStop::Stage::Blocked);
        stop.Request();assert(!stop.Ready()); // repeated B cannot re-arm
    }
    {
        OfflineRL rl;Init(rl);VA15ControlledStop stop;Prepare(stop,rl);
        rl.robot_command.motor_command.q[0]+=.7f;
        for(int i=0;i<10;++i)Tick(stop,rl);
        assert(!stop.captured && stop.stage==VA15ControlledStop::Stage::Decelerate);
        rl.robot_command.motor_command.q=rl.robot_state.motor_state.q;
        Tick(stop,rl);Tick(stop,rl);
        assert(stop.captured); // B survives a rejected high-error gait frame
    }
    {
        OfflineRL rl;Init(rl);VA15ControlledStop stop;
        // Even perfect current geometry/support cannot invent a missing
        // pre-walk standing-effort reference from a moving gait phase.
        stop.Request();
        for(int i=0;i<1700;++i)Tick(stop,rl);
        assert(!stop.captured && !stop.Ready());
    }
    {
        OfflineRL rl;Init(rl);VA15ControlledStop stop;
        rl.robot_state.real_stop_feedback.contacts_valid=false;
        stop.ObserveStanding(rl.robot_state,rl.robot_command);
        assert(!stop.stand_reference_valid);
    }
    {
        OfflineRL rl;Init(rl);VA15ControlledStop stop;Prepare(stop,rl);
        Tick(stop,rl);Tick(stop,rl);assert(stop.captured);
        const auto held=rl.robot_command.motor_command.q;
        rl.robot_state.real_stop_feedback.lowstate_fresh=false;
        for(int i=0;i<3000;++i)Tick(stop,rl);
        assert(!stop.Ready() && rl.robot_command.motor_command.q==held);
    }
    {
        OfflineRL rl;Init(rl);RLFSMStateRLLocomotion state(&rl);
        state.fsm_state=&rl.robot_state;state.fsm_command=&rl.robot_command;
        state.Enter();
        rl.params.config_node["independent_B_stop_candidate"]=true;
        for(int i=0;i<501;++i)state.Run();
        state.policy_running=rl.rl_init_done=true;
        rl.robot_state.motor_state.dq[4]=.8f;
        rl.control.current_gamepad=Input::Gamepad::B;state.Run();
        rl.control.current_gamepad=Input::Gamepad::None;state.Run();
        assert(state.controlled_stop.captured && !state.policy_running && !rl.rl_init_done);
        assert(state.CheckChange()=="RLFSMStateRLLocomotion");
        // Stale queued actor frames cannot overwrite the stand controller.
        rl.output_dof_pos_queue.push(std::vector<float>(18,123.f));state.Run();
        assert(rl.output_dof_pos_queue.empty());
        rl.control.x=.3f;state.Run();assert(rl.control.x==0.f);
        rl.robot_state.motor_state.dq[4]=0.f;
        for(int i=0;i<510;++i)state.Run();
        assert(state.CheckChange()=="RLFSMStateGetDown");
        rl.control.current_keyboard=Input::Keyboard::P;
        assert(state.CheckChange()=="RLFSMStatePassive");
    }
    std::cout.rdbuf(old);
    std::cout<<"PASS: independent catch then quiet stand; exact initial PD+FF torque continuity; missing/stale/partial support never lowers; B latch, actor exclusion and emergency priority\n";
}
