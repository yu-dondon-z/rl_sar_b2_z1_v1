// Memory-only native FSM tests: no hardware, SDK, DDS, UDP, or physics.
#include "production/fsm_b2_z1_no_gun.hpp"
#include <cassert>
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

struct Fixture
{
    OfflineRL rl;
    std::unique_ptr<RLFSMStateRLLocomotion> loco;
    Fixture()
    {
        rl.robot_name="b2_z1_no_gun";
        rl.ReadYaml(rl.robot_name,"base.yaml");rl.InitRL(rl.robot_name+"/VA15");
        rl.params.config_node["startup_torque_handoff_candidate"]=true;
        rl.params.config_node["isolated_forward_trial"]=true;
        rl.params.config_node["isolated_forward_trial_motion"]=true;
        rl.params.config_node["independent_B_stop_candidate"]=true;
        auto& s=rl.robot_state;
        s.motor_state.q=DeploymentStandPosition(rl);s.motor_state.dq.assign(18,0.f);
        s.imu.quaternion={1,0,0,0};s.imu.gyroscope={0,0,0};
        // Explicit experimental source; never pretend it is a Sport packet.
        va15_independent::Result r;
        r.fresh=r.input_valid=r.sequence_valid=r.initialized=r.estimate_valid_for_review=true;
        r.accepted_feet=r.support_count=4;r.load_candidate.fill(true);
        r.base_height=.52;r.planar_speed=0.;r.position_sigma=r.velocity_sigma=.001;
        s.real_stop_feedback=va15_trial::Feedback(r);
        auto& c=rl.robot_command.motor_command;
        c.q=s.motor_state.q;c.dq.assign(18,0.f);c.tau.assign(18,0.f);
        c.kp=DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
        c.kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
        // Reproduce loaded standing error WITHOUT integrating any motion.
        for(int i=9;i<=12;++i)s.motor_state.q[i]-=.10f;
        loco=std::make_unique<RLFSMStateRLLocomotion>(&rl);
        loco->fsm_state=&s;loco->fsm_command=&rl.robot_command;loco->Enter();
        for(int i=0;i<620;++i){rl.control.x=0.f;loco->Run();}
        assert(loco->real_start_support_cycles==100);
    }
    void Run(float request=.3f)
    {rl.control.x=request;loco->Run();}
    void Frame()
    {
        rl.output_dof_pos_queue.push(DeploymentStandPosition(rl));
        rl.output_dof_vel_queue.push(std::vector<float>(18,0.f));
        rl.output_dof_tau_queue.push(std::vector<float>(18,0.f));
    }
};
static bool Same(const RobotCommand<float>& a,const RobotCommand<float>& b)
{
    const auto& x=a.motor_command;const auto& y=b.motor_command;
    return x.q==y.q&&x.dq==y.dq&&x.kp==y.kp&&x.kd==y.kd&&x.tau==y.tau;
}
int main()
{
    std::ostringstream quiet;auto* old=std::cout.rdbuf(quiet.rdbuf());
    {
        Fixture f;const auto stand=f.rl.robot_command;
        for(int i=0;i<3;++i){f.Run();assert(Same(f.rl.robot_command,stand));}
        assert(f.loco->startup_handoff.Waiting());
        const auto history=f.rl.obs.actions;
        f.Frame();f.Run();
        assert(f.loco->startup_handoff.ever_applied);
        assert(f.loco->startup_handoff.initial_pd_error<1.e-3f);
        assert(f.rl.obs.actions==history); // handoff must not rewrite raw action history
        for(int i:VA15StartupHandoff::Legs)
        {
            const float q=f.rl.robot_state.motor_state.q[i],dq=f.rl.robot_state.motor_state.dq[i];
            const auto& a=stand.motor_command;const auto& b=f.rl.robot_command.motor_command;
            assert(std::fabs(a.kp[i]*(a.q[i]-q)+a.kd[i]*(a.dq[i]-dq)+a.tau[i]-
                (b.kp[i]*(b.q[i]-q)+b.kd[i]*(b.dq[i]-dq)+b.tau[i]))<1.e-3f);
        }
    }
    {
        Fixture f;const auto stand=f.rl.robot_command;
        for(int i=0;i<24;++i){f.Run();assert(Same(stand,f.rl.robot_command));}
        assert(!f.loco->startup_handoff.ever_applied&&!f.loco->policy_running);
        assert(f.loco->trial.Stopping());
    }
    // X, B, or stick release before the first frame must NEVER drop the
    // stand gains/targets. B remains latched for the existing stop/down gate.
    for(int cancel=0;cancel<3;++cancel)
    {
        Fixture f;f.Run();const auto stand=f.rl.robot_command;
        if(cancel==0)f.rl.control.current_gamepad=Input::Gamepad::X;
        if(cancel==1)f.rl.control.current_gamepad=Input::Gamepad::B;
        f.Run(cancel==2?0.f:.3f);
        assert(Same(stand,f.rl.robot_command));assert(f.loco->trial.Stopping());
        if(cancel==1)assert(f.loco->getdown_requested);
    }
    {
        Fixture f;f.Run();const auto stand=f.rl.robot_command;
        f.rl.robot_state.real_stop_feedback.lowstate_fresh=false;
        f.Frame();f.Run();
        assert(Same(stand,f.rl.robot_command));assert(f.loco->trial.Stopping());
        assert(!f.loco->startup_handoff.ever_applied);
        assert(!f.loco->controlled_stop.Ready());
    }
    std::cout.rdbuf(old);
    std::cout<<"PASS native FSM complete stand wait, first-frame requested PD, raw history, timeout, release/X/B and stale-feedback refusal\n";
}
