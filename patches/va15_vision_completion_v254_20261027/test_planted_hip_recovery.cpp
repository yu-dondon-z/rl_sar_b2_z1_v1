#include "production/va15_controlled_stop.hpp"
#include <cassert>
#include <iostream>
#include <limits>

namespace {
struct Fixture {
    RobotState<float> state;
    RobotCommand<float> command;
    std::vector<float> nominal=std::vector<float>(18,0.f);
    std::vector<float> kp=std::vector<float>(18,450.f),kd=std::vector<float>(18,15.f);
    VA15ControlledStop stop;
    Fixture() {
        for(int i=4;i<8;++i)nominal[i]=.8f;
        for(int i=9;i<13;++i){nominal[i]=-1.5f;kp[i]=675.f;}
        state.motor_state.q=nominal;state.motor_state.dq.assign(18,0.f);
        state.imu.quaternion={1.f,0.f,0.f,0.f};state.imu.gyroscope={0.f,0.f,0.f};
        auto& f=state.real_stop_feedback;
        f.lowstate_fresh=f.sport_fresh=f.contacts_valid=f.height_valid=f.velocity_valid=true;
        f.base_height=.52f;f.planar_speed=0.f;f.contacts.fill(true);
        command.motor_command.q=nominal;command.motor_command.kp=kp;command.motor_command.kd=kd;
        command.motor_command.dq.assign(18,0.f);command.motor_command.tau.assign(18,0.f);
        // Verified prewalk static effort, not an effort learned while moving.
        for(int i=0;i<4;++i)state.motor_state.q[i]=(i%2==0?1.f:-1.f)*.055f;
        stop.ObserveStanding(state,command);assert(stop.stand_reference_valid);
    }
    void tick() {stop.Step(state,command,nominal,kp,kd,.005,0.f);}
    void plant() {
        for(int i=0;i<4;++i)state.motor_state.q[i]=(i%2==0?1.f:-1.f)*.16f;
        for(int i=0;i<4;++i)command.motor_command.q[i]=state.motor_state.q[i]+(i%2==0?.09f:-.09f);
        stop.Request();tick();tick();
        assert(stop.captured&&stop.planted_hip_reference_valid);
    }
};
}

int main() {
    Fixture f;f.plant();
    const auto anchor=f.stop.recovery_reference_q;
    assert(anchor.size()==18);
    for(int i=0;i<4;++i) {
        const float expected=(i%2==0?1.f:-1.f)*.105f;
        assert(std::fabs(anchor[i]-expected)<1.e-6f);
        assert(std::fabs(anchor[i]-f.nominal[i])<=VA15ControlledStop::HipReferenceEnvelope);
        assert(std::fabs(anchor[i]-f.state.motor_state.q[i])<=VA15ControlledStop::HipReferenceEnvelope);
    }
    for(int i:VA15LegIndices)if(i>=4)assert(anchor[i]==f.nominal[i]);
    // A measured-pose change does not drag the fixed reference with it.
    f.state.motor_state.q[0]+=.02f;
    f.state.real_stop_feedback.contacts[1]=false;
    for(int n=0;n<3400;++n)f.tick();
    assert(f.stop.recovery_reference_q==anchor);
    assert(f.stop.stage==VA15ControlledStop::Stage::Settle&&!f.stop.Ready());
    assert(f.stop.quiet==0.);
    for(int i:VA15LegIndices)assert(std::fabs(f.command.motor_command.q[i]-anchor[i])<1.e-6f);
    assert(f.stop.diagnostic.mask==VA15StopGateDiagnostics::PartialSupport);
    // ORIGINAL all-four final quiet gate still needs 0.5 seconds.
    f.state.real_stop_feedback.contacts[1]=true;
    for(int n=0;n<99;++n)f.tick();assert(!f.stop.Ready());
    f.tick();assert(f.stop.Ready());

    // No supported capture if even one contact is absent.
    Fixture missing;missing.state.real_stop_feedback.contacts[1]=false;
    missing.stop.Request();for(int n=0;n<100;++n)missing.tick();
    assert(!missing.stop.captured&&!missing.stop.planted_hip_reference_valid);
    assert(missing.stop.recovery_reference_q.empty());

    // Unsafe geometry is rejected, NOT clipped into the nominal envelope.
    Fixture wide;wide.state.motor_state.q[3]=-.30f;wide.command.motor_command.q[3]=-.30f;
    const auto untouched=wide.command.motor_command.q;
    wide.stop.Request();wide.tick();wide.tick();
    assert(!wide.stop.captured&&!wide.stop.planted_hip_reference_valid);
    assert(wide.stop.recovery_reference_q.empty()&&wide.stop.held.motor_command.q.empty());
    assert(wide.command.motor_command.q==untouched);

    // Small PLANNED-reference overshoots are allocated inside BOTH original
    // envelopes, not used to reject a genuinely supported catch forever.
    Fixture edge;edge.state.motor_state.q[0]=.22f;edge.command.motor_command.q[0]=.27f;
    const float entry_tau=edge.command.motor_command.kp[0]*(.27f-.22f);
    edge.stop.Request();edge.tick();edge.tick();
    assert(edge.stop.captured&&edge.stop.reference_projections==1);
    assert(std::fabs(edge.stop.recovery_reference_q[0]-.15f)<1.e-6f);
    assert(edge.stop.reference_projection_delta<=VA15ControlledStop::HipReferenceProjectionEnvelope);
    const auto& caught=edge.command.motor_command;
    assert(std::fabs(caught.kp[0]*(caught.q[0]-.22f)-entry_tau)<1.e-4f);
    assert(std::fabs(edge.stop.recovery_reference_q[0]-edge.state.motor_state.q[0])<=.15f);
    assert(!edge.stop.Ready());

    Fixture edge_missing;edge_missing.state.motor_state.q[0]=.22f;
    edge_missing.state.real_stop_feedback.contacts[1]=false;
    edge_missing.stop.Request();for(int i=0;i<100;++i)edge_missing.tick();
    assert(!edge_missing.stop.captured&&edge_missing.stop.reference_projections==0);

    // Verified effort cannot demand a >0.15 rad hip compensation.
    Fixture effort;effort.kp[0]=100.f;
    effort.stop.Request();effort.tick();effort.tick();
    assert(!effort.stop.captured&&effort.stop.recovery_reference_q.empty());

    // A late invalid equivalent target leaves no partially captured command.
    Fixture late;late.command.motor_command.q[12]+=.61f;
    const auto late_command=late.command.motor_command.q;
    late.stop.Request();late.tick();late.tick();
    assert(!late.stop.captured&&late.stop.held.motor_command.q.empty());
    assert(late.stop.recovery_reference_q.empty()&&late.command.motor_command.q==late_command);

    Fixture bad;bad.kp[0]=std::numeric_limits<float>::quiet_NaN();
    bad.stop.Request();bad.tick();
    assert(bad.stop.stage==VA15ControlledStop::Stage::Blocked&&!bad.stop.captured);
    Fixture stale;stale.plant();stale.state.real_stop_feedback.lowstate_fresh=false;stale.tick();
    assert(stale.stop.stage==VA15ControlledStop::Stage::Blocked);
    const auto stale_held=stale.command.motor_command.q;
    stale.state.real_stop_feedback.lowstate_fresh=true;
    for(int n=0;n<600;++n)stale.tick();
    assert(stale.stop.stage==VA15ControlledStop::Stage::Blocked&&!stale.stop.Ready());
    assert(stale.command.motor_command.q==stale_held);
    Fixture tilt;tilt.plant();tilt.state.imu.quaternion={.9950042f,.0998334f,0.f,0.f};tilt.tick();
    assert(tilt.stop.stage==VA15ControlledStop::Stage::Blocked&&!tilt.stop.Ready());

    // Use the same explicitly experimental feedback branch as the real
    // candidate, not the certified/SportMode-style branch above. These are
    // injected unit-test inputs, never claimed to be physical measurements.
    Fixture model;
    model.state.real_stop_feedback.sport_fresh=false;
    model.state.real_stop_feedback.unavailable_reason=va15_trial::Source;
    assert(va15_trial::EstimatedValid(model.state.real_stop_feedback));
    model.plant();
    model.state.real_stop_feedback.contacts[1]=false; // FL/FR/RL/RR = 1011
    for(int n=0;n<16000;++n)model.tick();
    assert(model.stop.stage==VA15ControlledStop::Stage::Settle&&!model.stop.Ready());
    assert(model.stop.diagnostic.contacts==13u&&model.stop.diagnostic.mask==4u);
    model.state.real_stop_feedback.contacts[1]=true;
    for(int n=0;n<99;++n)model.tick();assert(!model.stop.Ready());
    model.state.real_stop_feedback.contacts[1]=false;model.tick();
    assert(model.stop.quiet==0.&&!model.stop.Ready());
    model.state.real_stop_feedback.contacts[1]=true;
    for(int n=0;n<99;++n)model.tick();assert(!model.stop.Ready());
    model.tick();assert(model.stop.Ready());

    // Recorded numerical counterfactual from PLANTED_HIP_REVIEW.md. The
    // prewalk efforts are approximations from decimated snapshots, NOT the
    // unrecorded private stand_tau or a physical replay. In particular the
    // captured FL angle exceeds .15 rad; the ENDPOINT must meet the bounds.
    Fixture recorded;
    const std::array<float,4> prewalk_effort{{-24.919f,24.188f,-25.037f,24.790f}};
    const std::array<float,4> captured_hip{{.167578f,-.137513f,.036986f,-.063344f}};
    const std::array<float,4> expected_endpoint{{.112202f,-.083762f,-.018652f,-.008256f}};
    recorded.stop.Reset();
    for(int i=0;i<4;++i)recorded.state.motor_state.q[i]=-prewalk_effort[i]/recorded.kp[i];
    recorded.stop.ObserveStanding(recorded.state,recorded.command);
    assert(recorded.stop.stand_reference_valid);
    std::array<float,4> entry_effort{};
    for(int i=0;i<4;++i) {
        recorded.state.motor_state.q[i]=captured_hip[i];
        recorded.state.motor_state.dq[i]=.10f;
        recorded.command.motor_command.q[i]=captured_hip[i]+.05f;
        recorded.command.motor_command.dq[i]=.02f;
        recorded.command.motor_command.kp[i]=160.f;
        recorded.command.motor_command.kd[i]=5.f;
        recorded.command.motor_command.tau[i]=.3f;
        const auto& c=recorded.command.motor_command;
        entry_effort[i]=c.kp[i]*(c.q[i]-captured_hip[i])+c.kd[i]*(c.dq[i]-.10f)+c.tau[i];
    }
    recorded.stop.Request();recorded.tick();recorded.tick();assert(recorded.stop.captured);
    for(int i=0;i<4;++i) {
        assert(std::fabs(recorded.stop.recovery_reference_q[i]-expected_endpoint[i])<2.e-6f);
        const auto& c=recorded.command.motor_command;
        const float caught=c.kp[i]*(c.q[i]-captured_hip[i])+c.kd[i]*(c.dq[i]-.10f)+c.tau[i];
        assert(std::fabs(caught-entry_effort[i])<1.e-4f);
    }
    std::cout<<"PASS fixed planted hip references, original sagittal stand/gains, transactional rejection, unchanged all-four/stale/tilt gates; no robot I/O\n";
}
