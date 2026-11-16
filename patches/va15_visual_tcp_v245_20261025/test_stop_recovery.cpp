#include "production/fsm_b2_z1_no_gun.hpp"
#include <cassert>
#include <iostream>
using namespace b2_z1_no_gun_fsm;
class OfflineRL final:public RL {
public:
    bool RequiresRealStopFeedback()const override{return true;}
    std::vector<float> Forward()override{return {};}
    void GetState(RobotState<float>*)override{}
    void SetCommand(const RobotCommand<float>*)override{}
};
int main() {
    OfflineRL rl;rl.robot_name="b2_z1_no_gun";rl.ReadYaml(rl.robot_name,"base.yaml");rl.InitRL(rl.robot_name+"/VA15");
    auto& s=rl.robot_state;s.motor_state.q=DeploymentStandPosition(rl);s.motor_state.dq.assign(18,0);
    s.imu.quaternion={1,0,0,0};s.imu.gyroscope={0,0,0};
    auto& f=s.real_stop_feedback;f.lowstate_fresh=f.sport_fresh=f.contacts_valid=f.height_valid=f.velocity_valid=true;
    f.base_height=.52;f.planar_speed=0;f.contacts.fill(true);
    auto q=s.motor_state.q;auto kp=DeploymentStandGain(rl,"deployment_stand_kp","fixed_kp");
    auto kd=DeploymentStandGain(rl,"deployment_stand_kd","fixed_kd");
    auto& cmd=rl.robot_command;cmd.motor_command.q=q;cmd.motor_command.kp=kp;cmd.motor_command.kd=kd;
    cmd.motor_command.dq.assign(18,0);cmd.motor_command.tau.assign(18,0);
    VA15ControlledStop stop;stop.ObserveStanding(s,cmd);stop.Request();
    const auto tick=[&](){stop.Step(s,cmd,q,kp,kd,.005,0.);};
    tick();tick();assert(stop.captured&&!stop.Ready());
    f.contacts[0]=false;
    for(int i=0;i<3400;++i)tick();
    assert(stop.stage==VA15ControlledStop::Stage::Settle&&stop.progress==1.&&!stop.Ready());
    f.contacts[0]=true;for(int i=0;i<100;++i)tick();assert(stop.Ready());
    stop.Reset();stop.ObserveStanding(s,cmd);stop.Request();tick();tick();assert(stop.captured);
    const auto held=cmd.motor_command.q;
    f.sport_fresh=false;f.unavailable_reason=va15_trial::HealthOnly;
    f.contacts_valid=f.height_valid=f.velocity_valid=false;
    f.base_height=f.planar_speed=std::numeric_limits<float>::quiet_NaN();
    for(int i=0;i<200;++i)tick();
    assert(stop.stage==VA15ControlledStop::Stage::CatchStand&&!stop.Ready());
    assert(cmd.motor_command.q==held&&stop.quiet==0.);
    f.unavailable_reason=va15_trial::Source;
    f.contacts_valid=f.height_valid=f.velocity_valid=true;
    f.base_height=.52;f.planar_speed=0.;f.contacts.fill(true);
    for(int i=0;i<600;++i)tick();assert(stop.Ready());
    stop.Reset();stop.ObserveStanding(s,cmd);stop.Request();tick();tick();assert(stop.captured);
    f.lowstate_fresh=false;tick();assert(stop.stage==VA15ControlledStop::Stage::Blocked&&!stop.Ready());
    std::cout<<"PASS model-support gap holds/requalifies without actor or lowering; >14 s contact wait recovers; actual stale feedback still latches\n";
}
