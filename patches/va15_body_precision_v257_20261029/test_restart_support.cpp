// Offline memory regression for the native ordering: rearm, one neutral
// control cycle, then the autonomous positive request and a delayed actor.
#ifdef VA15_RESTART_BASELINE_FSM
#include VA15_RESTART_BASELINE_FSM
#else
#include "production/fsm_b2_z1_no_gun.hpp"
#endif
#include <cassert>
#include <iostream>

using namespace b2_z1_no_gun_fsm;
class RestartRL final : public RL {
public:
    bool RequiresRealStopFeedback() const override { return true; }
    std::vector<float> Forward() override { return {}; }
    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
};
static float Effort(const RobotState<float>& s,const RobotCommand<float>& c,int i) {
    const auto& m=c.motor_command;
    return m.kp[i]*(m.q[i]-s.motor_state.q[i])+
        m.kd[i]*(m.dq[i]-s.motor_state.dq[i])+m.tau[i];
}
int main() {
    RestartRL rl;rl.robot_name="b2_z1_no_gun";
    rl.ReadYaml(rl.robot_name,"base.yaml");rl.InitRL(rl.robot_name+"/VA15");
    rl.params.config_node["isolated_forward_trial"]=true;
    rl.params.config_node["independent_B_stop_candidate"]=true;
    rl.params.config_node["near_visual_trial"]=true;
    rl.params.config_node["repeatable_visual_session"]=true;
    auto& s=rl.robot_state;auto& c=rl.robot_command;
    s.motor_state.q=DeploymentStandPosition(rl);s.motor_state.dq.assign(18,0.f);
    s.motor_state.tau_est.assign(18,0.f);
    s.imu.quaternion={1,0,0,0};s.imu.gyroscope={0,0,0};
    auto& f=s.real_stop_feedback;
    f.lowstate_fresh=f.contacts_valid=f.height_valid=f.velocity_valid=true;
    f.sport_fresh=false;f.unavailable_reason=va15_trial::Source;
    f.contacts.fill(true);f.base_height=.513f;f.planar_speed=.033f;
    c.motor_command.q=DeploymentStandPosition(rl);
    c.motor_command.dq.assign(18,0.f);c.motor_command.tau.assign(18,0.f);
    c.motor_command.kp=DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
    c.motor_command.kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
    // Explicit synthetic loaded stand, close to the recorded restart poses.
    // It does not represent a measured motor acknowledgement.
    for(int i:VA15LegIndices)s.motor_state.q[i]-=i>=9?.065f:.015f;
    RLFSMStateRLLocomotion loco(&rl);
    loco.fsm_state=&s;loco.fsm_command=&c;
    loco.Enter();loco.handoff_hold_cycles=0;
    loco.trial.Configure(true,true,true);
    const auto original=c;
    float maximum=0.f;
    for(int session=0;session<3;++session) {
        c=original;rl.rl_init_done=false;loco.policy_running=false;
        loco.real_start_support_cycles=100;loco.leg_offset_scale=0.f;
        loco.controlled_stop.Reset();loco.controlled_stop.ObserveStanding(s,c);
        assert(loco.controlled_stop.stand_reference_valid);
        loco.controlled_stop.stage=VA15ControlledStop::Stage::ReadyToLower;
        loco.controlled_stop.captured=true;loco.controlled_stop.held=c;
        loco.trial.Stop("synthetic completed previous session");
        assert(loco.RearmVisualSession(true,true));
        rl.control.current_keyboard=Input::Keyboard::None;
        rl.control.current_gamepad=Input::Gamepad::None;
        // Native UpdateNearGuide re-arms BEFORE StateController, whereas its
        // autonomous forward request is assigned AFTER StateController.
        for(int wait=0;wait<1+session*4;++wait) {
            rl.control.x=rl.control.y=rl.control.yaw=0.f;
            loco.Run();
            for(int i:VA15LegIndices) {
                maximum=std::max(maximum,std::fabs(Effort(s,c,i)-Effort(s,original,i)));
                if(std::fabs(Effort(s,c,i)-Effort(s,original,i))>.001f) {
                    std::cerr<<"Restart neutral frame lost support at policy joint "<<i
                        <<": before="<<Effort(s,original,i)<<" after="<<Effort(s,c,i)<<" Nm\n";
                    return 2;
                }
                assert(c.motor_command.q[i]==original.motor_command.q[i]);
                assert(c.motor_command.kp[i]==original.motor_command.kp[i]);
                assert(c.motor_command.kd[i]==original.motor_command.kd[i]);
            }
            assert(!loco.policy_running);
        }
        rl.control.x=.15f;loco.Run();
        if(!loco.policy_running||!loco.policy_handoff.pending)
            std::cerr<<"run="<<loco.policy_running<<" pending="<<loco.policy_handoff.pending
                <<" joined="<<loco.policy_handoff.joining<<" x="<<rl.control.x
                <<" trial="<<int(loco.trial.stage)<<" support="<<loco.real_start_support_cycles
                <<" starter="<<loco.gait_starter_active<<"\n";
        assert(loco.policy_running&&loco.policy_handoff.pending);
        for(int i:VA15LegIndices)assert(c.motor_command.kp[i]==original.motor_command.kp[i]);
        auto actor=original.motor_command.q;
        for(int i:VA15LegIndices)actor[i]+=.025f;
        rl.output_dof_pos_queue.push(actor);
        rl.output_dof_vel_queue.push(std::vector<float>(18,0.f));
        loco.Run();
        assert(loco.policy_handoff.joining&&!loco.policy_handoff.failed);
        for(int i:VA15LegIndices) {
            maximum=std::max(maximum,std::fabs(Effort(s,c,i)-Effort(s,original,i)));
            assert(std::fabs(Effort(s,c,i)-Effort(s,original,i))<.001f);
        }
        // Verify the relearned prewalk support also survives the NEXT catch.
        // After the dynamic effort fades (0.3 s), it must still support load
        // instead of using the low-gain neutral frame as its reference.
        loco.controlled_stop.Request();
        for(int tick=0;tick<64;++tick)
            loco.controlled_stop.Step(s,c,DeploymentStandPosition(rl),
                original.motor_command.kp,original.motor_command.kd,.005,0.f);
        assert(loco.controlled_stop.captured);
        for(int i:VA15LegIndices)if(i>=9)
            assert(Effort(s,c,i)>.85f*Effort(s,original,i));
    }
    std::cout<<"PASS 3 restarts: exact loaded standby through neutral delays, delayed actor and next stop; max handoff difference="
        <<maximum<<" Nm; memory only, no robot/network\n";
}
