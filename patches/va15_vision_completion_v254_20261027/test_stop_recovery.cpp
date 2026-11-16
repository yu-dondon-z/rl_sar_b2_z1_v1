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
    // Reproduce V249's 0.646 rad/s yaw transient 68 ms after capture. It
    // must pause at the exact last command, not latch that asymmetric pose.
    const auto prepare=[&]() {
        f.lowstate_fresh=f.sport_fresh=f.contacts_valid=f.height_valid=f.velocity_valid=true;
        f.base_height=.52;f.planar_speed=0.;f.contacts.fill(true);
        s.motor_state.q=q;s.motor_state.dq.assign(18,0.f);s.imu.gyroscope={0,0,0};
        cmd.motor_command.q=q;cmd.motor_command.kp=kp;cmd.motor_command.kd=kd;
        cmd.motor_command.dq.assign(18,0.f);cmd.motor_command.tau.assign(18,0.f);
        stop.Reset();stop.ObserveStanding(s,cmd);
        cmd.motor_command.q[0]+=.08;cmd.motor_command.q[3]-=.08;
        stop.Request();tick();tick();assert(stop.captured);
    };
    prepare();for(int i=0;i<13;++i)tick();const auto before_pause=cmd.motor_command.q;
    s.imu.gyroscope[2]=.646310866f;tick();
    assert(stop.stage==VA15ControlledStop::Stage::AwaitQuiet&&!stop.Ready());
    for(int j:VA15LegIndices)assert(std::fabs(cmd.motor_command.q[j]-before_pause[j])<.01);
    for(int i=0;i<500;++i)tick();
    assert(stop.recovery_quiet==0.&&stop.progress==1.&&!stop.Ready());
    for(int j:VA15LegIndices)assert(std::fabs(cmd.motor_command.q[j]-q[j])<1.e-5);
    s.imu.gyroscope[2]=0.;f.contacts[0]=false;
    for(int i=0;i<200;++i)tick();
    assert(stop.stage==VA15ControlledStop::Stage::Settle&&!stop.Ready());
    f.contacts[0]=true;
    for(int i=0;i<99;++i)tick();assert(!stop.Ready());
    tick();assert(stop.Ready());
    for(int i=0;i<600;++i)tick();assert(stop.Ready()&&cmd.motor_command.q==q);
    // True stale transport and roll/pitch angular instability still latch;
    // fresh quiet data cannot silently re-arm either failure.
    prepare();s.imu.gyroscope[2]=.65f;tick();f.lowstate_fresh=false;tick();
    assert(stop.stage==VA15ControlledStop::Stage::Blocked);
    f.lowstate_fresh=true;s.imu.gyroscope[2]=0.;for(int i=0;i<200;++i)tick();
    assert(stop.stage==VA15ControlledStop::Stage::Blocked&&!stop.Ready());
    prepare();s.imu.gyroscope[0]=.65f;tick();assert(stop.stage==VA15ControlledStop::Stage::Blocked);
    std::cout<<"PASS model-support gap holds/requalifies without actor or lowering; >14 s contact wait recovers; actual stale feedback still latches\n";
}
