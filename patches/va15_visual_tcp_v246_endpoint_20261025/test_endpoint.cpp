#include "production/endpoint_follow.hpp"
#include <cassert>
#include <iostream>
using namespace va15_endpoint;
namespace {
constexpr std::uint64_t Second=1000000000ULL;
Pose Capture(std::uint64_t ns,double x=0.) {return {ns,V3(x,0.,.52),Eigen::Quaterniond::Identity(),true};}
double Row(const Pose& pose,double world_x) {
    double a=620.,b=1058.;V3 p;
    for(int j=0;j<60;++j) {
        const double v=(a+b)/2.;
        if(!va15_tcp::Project(pose,{935.,v},p)||p.x()>world_x)a=v;else b=v;
    }
    return (a+b)/2.;
}
va15_visual::Packet Frame(std::uint64_t ns,unsigned seq,double base_x=0.,double end_x=2.6,double near_x=1.25) {
    va15_visual::Packet p;p.valid=p.session_matched=p.image_received=true;p.sequence=seq;p.capture_ns=ns;p.publish_ns=ns+10000000;
    p.u=p.look_u=935.;p.v=972.;p.look_v=820.8;p.confidence=p.look_confidence=.99;
    p.width=30.;p.contrast=.8;p.rms=1.;
    const auto capture=Capture(ns,base_x);
    const double near=std::max(near_x,base_x+1.1),far=Row(capture,end_x),start=Row(capture,near);
    for(int j=0;j<=80;++j)p.path.push_back({935.,start+(far-start)*double(j)/80.});
    p.path_count=p.path.size();p.path_span=start-far;
    p.look_v=std::max(va15_visual::NominalLookV,far+4.);
    // V245 supports exact supported adaptive look row up to MaximumLookV.
    return p;
}
EndpointEvidence Evidence(const va15_visual::Packet& p,bool candidate=true) {
    return {candidate,p.path.back()[0],p.path.back()[1],.995,36.};
}
bool Feed(Follower& f,std::uint64_t ns,unsigned seq,bool candidate=true,double end_x=2.6,double base_x=0.,double near_x=1.25) {
    auto p=Frame(ns,seq,base_x,end_x,near_x);
    return f.Observe(p,Capture(ns,base_x),ns+10000000,0,1,Evidence(p,candidate));
}
void Confirm(Follower& f) {
    f.Reset(0,1);
    for(unsigned j=0;j<6;++j)assert(Feed(f,Second+j*100000000ULL,j+1));
    assert(f.Confirmed()&&f.ConfirmingFrames()==6);
}
Input State(std::uint64_t ns,double tcp_x,double base_x) {
    Input i;i.current=Capture(ns,base_x);i.now_ns=ns;i.camera_ns=ns;
    i.tcp_world=V3(tcp_x,.0,.09);i.generation=0;i.session=1;
    i.healthy=i.camera_healthy=true;i.terminal_metric_verified=true;i.actual_speed_upper=.02;i.clearance=.09;
    i.target_seed=V3(-.775,0.,-.43);i.speed_seed=.15;return i;
}
void Move(Input& in,double tcp_dx,double base_dx=0.) {
    in.now_ns+=5000000;in.current.ns=in.camera_ns=in.now_ns;
    in.tcp_world.x()+=tcp_dx;in.current.p.x()+=base_dx;
}
}
int main() {
    // A dark segment without a positively identified clear cap is not an end.
    Follower f;f.Reset(0,1);
    for(unsigned j=0;j<12;++j)assert(Feed(f,Second+j*100000000ULL,j+1,false));
    assert(!f.Confirmed());
    auto p=Frame(3*Second,20);auto e=Evidence(p);e.clear_pixels=10.;
    assert(f.Observe(p,Capture(3*Second),3*Second+10000000,0,1,e)&&!f.Confirmed());

    // Reading one file repeatedly is not six frames; mutation is an error.
    f.Reset(0,1);p=Frame(Second,1);e=Evidence(p);
    for(int j=0;j<20;++j)assert(f.Observe(p,Capture(Second),Second+10000000,0,1,e));
    assert(f.ConfirmingFrames()==1&&!f.Confirmed());
    e.u+=1.;assert(!f.Observe(p,Capture(Second),Second+10000000,0,1,e));
    assert(f.GetStage()==Stage::Fault);
    f.Reset(0,1);
    for(unsigned j=0;j<6;++j)assert(Feed(f,Second+j*50000000ULL,j+1));
    assert(!f.Confirmed()); // six frames but less than half a second
    f.Reset(0,1);assert(Feed(f,Second,2));
    assert(!Feed(f,Second+100000000ULL,1)&&f.GetStage()==Stage::Fault);
    f.Reset(0,1);p=Frame(Second,1);
    assert(!f.Observe(p,Capture(Second),2*Second,0,1,Evidence(p))&&!f.Confirmed());
    f.Reset(0,1);p=Frame(Second,1);p.image_received=false;
    assert(!f.Observe(p,Capture(Second),p.publish_ns,0,1,Evidence(p))&&!f.Confirmed());

    Confirm(f);auto input=State(1510000000ULL,1.35,2.125);
    // Body can already be beyond endpoint while ACTUAL TCP remains behind.
    assert(f.BeginTail(input));assert(f.Active()&&!f.Completed());
    auto out=f.Step(input);
    if(out.stop)std::cerr<<"unexpected first tail stop: "<<out.reason<<" remaining="<<out.remaining_arc<<" error="<<out.endpoint_error<<'\n';
    assert(!out.stop&&out.remaining_arc>1.1);
    assert(!f.NotifyStableStand(input,true)&&!f.Completed());

    // Cached coverage must include actual TCP, not nominal/base-minus-offset.
    Confirm(f);input=State(1510000000ULL,.9,1.675);
    assert(!f.BeginTail(input)&&f.GetStage()==Stage::Confirmed);
    Confirm(f);input=State(1510000000ULL,1.35,2.125);input.tcp_world.y()=.16;
    assert(!f.BeginTail(input)&&f.GetStage()==Stage::Confirmed&&!f.Active());
    input.tcp_world.y()=0.;assert(f.BeginTail(input)&&f.Active());
    Confirm(f);input=State(1510000000ULL,1.35,2.125);assert(f.BeginTail(input));
    auto invalid=input;invalid.camera_healthy=false;
    assert(f.Step(invalid).stop&&f.GetStage()==Stage::Fault);
    Confirm(f);input=State(1510000000ULL,1.35,2.125);assert(f.BeginTail(input));
    invalid=input;invalid.camera_ns-=va15_visual::MaximumAgeNs+1;
    assert(f.Step(invalid).stop&&f.GetStage()==Stage::Fault);
    Confirm(f);input=State(1510000000ULL,1.35,2.125);assert(f.BeginTail(input));
    invalid=input;invalid.current.ns-=100000000ULL;
    assert(f.Step(invalid).stop&&f.GetStage()==Stage::Fault);

    // Actual FK progress, slew-limited target and same command bounds.
    Confirm(f);input=State(1510000000ULL,1.35,2.125);assert(f.BeginTail(input));
    V3 before=f.Target();double previous_speed=.15,previous_yaw=0.;
    for(int j=0;j<1000&&f.GetStage()==Stage::Tail;++j) {
        Move(input,.0015,.0015);out=f.Step(input);
        assert(f.GetStage()!=Stage::Fault);
        assert((out.target-before).cwiseAbs().maxCoeff()<=.00020001);
        assert(std::fabs(out.x-previous_speed)<=.00015001||out.stop);
        assert(std::fabs(out.yaw-previous_yaw)<=.00020001||out.stop);
        assert(out.x>=0.&&out.x<=.15&&std::fabs(out.yaw)<=.06);
        before=out.target;previous_speed=out.x;previous_yaw=out.yaw;
    }
    assert(f.AwaitingStand()&&!f.Completed());
    input.actual_speed_upper=.1;
    assert(!f.NotifyStableStand(input,false));
    input.actual_speed_upper=.01;
    assert(!f.NotifyStableStand(input,false));
    assert(!f.NotifyStableStand(input,true)&&!f.Completed());
    for(int j=0;j<201;++j){Move(input,0.);f.NotifyStableStand(input,true);}
    assert(f.Completed());
    assert(f.Step(input).completed&&f.Step(input).stop);
    assert(!f.BeginTail(input)); // no automatic restart after completion

    // Healthy IMU/no-slip navigation cannot certify physical tool accuracy.
    Confirm(f);input=State(1510000000ULL,2.59,3.365);input.terminal_metric_verified=false;
    assert(f.BeginTail(input)&&f.Step(input).awaiting_stand);
    for(int j=0;j<201;++j){Move(input,0.);assert(!f.NotifyStableStand(input,true));}
    assert(f.GetStage()==Stage::Stopped&&!f.Completed());

    // Stopping safely is not task completion if the final actual tool is high.
    Confirm(f);input=State(1510000000ULL,2.55,3.325);assert(f.BeginTail(input));
    out=f.Step(input);assert(!out.awaiting_stand); // old6cm gait-peak trigger was premature
    Move(input,.04,.04);
    out=f.Step(input);assert(out.awaiting_stand);
    input.tcp_world.z()=.20;
    for(int j=0;j<601;++j){Move(input,0.);assert(!f.NotifyStableStand(input,true));}
    assert(f.GetStage()==Stage::Stopped&&!f.Completed());

    // A single good post-stand sample is not a completed mission. Drift
    // during the confirmation window resets it, without restarting motion.
    Confirm(f);input=State(1510000000ULL,2.59,3.365);assert(f.BeginTail(input));
    assert(f.Step(input).awaiting_stand);
    for(int j=0;j<100;++j){Move(input,0.);assert(!f.NotifyStableStand(input,true));}
    input.tcp_world.z()=.20;
    for(int j=0;j<601;++j){Move(input,0.);assert(!f.NotifyStableStand(input,true));}
    assert(f.GetStage()==Stage::Stopped&&!f.Completed());

    // A fresh camera may report NO LINE after the confirmed end leaves the
    // image; finite tail still runs, while an unconfirmed loss never starts it.
    Confirm(f);input=State(1510000000ULL,1.35,2.125);assert(f.BeginTail(input));
    auto blank=Frame(1600000000ULL,7);blank.valid=false;blank.path.clear();blank.path_count=0;
    assert(!f.Observe(blank,Capture(blank.capture_ns),blank.publish_ns,0,1,{}));
    Move(input,.001,.001);assert(!f.Step(input).stop&&f.Active());
    f.Reset(0,1);assert(!f.BeginTail(input)&&!f.Active());

    // An operator stop has priority over reaching the end; explicit reset only.
    Confirm(f);input=State(1510000000ULL,1.35,2.125);assert(f.BeginTail(input));
    input.operator_stop=true;assert(f.Step(input).stop&&f.GetStage()==Stage::Stopped);
    input.operator_stop=false;assert(!f.BeginTail(input)&&!f.NotifyStableStand(input,true));
    Confirm(f);input=State(1510000000ULL,1.35,2.125);assert(f.BeginTail(input));
    ++input.generation;assert(f.Step(input).stop&&f.GetStage()==Stage::Fault);

    // Endpoint conflict, unseen path gap, excessive tail time and overshoot fail.
    Confirm(f);assert(!Feed(f,1700000000ULL,7,true,2.9)&&f.GetStage()==Stage::Fault);
    Confirm(f);assert(!Feed(f,1700000000ULL,7,false,2.9)&&f.GetStage()==Stage::Fault);
    assert(!f.Completed()); // later visible continuation disproves an old cap
    Confirm(f);p=Frame(1700000000ULL,7);p.path[20][0]+=500.;
    assert(!f.Observe(p,Capture(p.capture_ns),p.publish_ns,0,1,Evidence(p)));
    Confirm(f);input=State(1510000000ULL,1.35,2.125);assert(f.BeginTail(input));
    input.now_ns+=Follower::MaximumTailNs+1;input.current.ns=input.camera_ns=input.now_ns;
    assert(f.Step(input).stop&&f.GetStage()==Stage::Fault);
    Confirm(f);input=State(1510000000ULL,2.55,3.325);assert(f.BeginTail(input));
    for(int j=0;j<5;++j){Move(input,.04,.04);out=f.Step(input);}
    assert(out.stop&&f.GetStage()==Stage::Fault&&!f.Completed());

    // A stopped/stuck robot cannot consume arbitrary fresh-camera time while
    // issuing forward commands: the finite commanded-travel budget expires.
    Confirm(f);input=State(1510000000ULL,2.1,2.875);assert(f.BeginTail(input));
    for(int j=0;j<9000&&f.GetStage()==Stage::Tail;++j){Move(input,0.,0.);out=f.Step(input);}
    assert(f.GetStage()==Stage::Fault&&out.stop&&!f.Completed());

    // Curve stitching requires measured overlap, not a nearest endpoint gap.
    f.Reset(0,1);assert(Feed(f,Second,1,false,2.6,0.,1.25));
    assert(Feed(f,1100000000ULL,2,false,2.8,.1,1.35));
    const auto points=f.BankPoints();assert(points>81);
    auto gap=Frame(1200000000ULL,3,2.,4.6,3.25);
    assert(!f.Observe(gap,Capture(gap.capture_ns,2.),gap.publish_ns,0,1,Evidence(gap,false)));
    assert(f.BankPoints()==points&&!f.Confirmed());

    // Regression: gait-induced pitch changes move the far image cutoff a few
    // cm backward and forward while the physical curve remains identical.
    // A shorter overlapping view must not append a reverse endpoint segment.
    f.Reset(0,1);assert(Feed(f,Second,1,false,2.6,0.,1.25));
    const auto original_points=f.BankPoints();
    for(unsigned j=0;j<20;++j) {
        const double end=2.6-(j%3+1)*.006;
        assert(Feed(f,1100000000ULL+j*100000000ULL,j+2,false,end,.001*j,1.25));
        assert(f.BankPoints()==original_points);
    }
    assert(Feed(f,3200000000ULL,22,false,2.8,.1,1.35));
    assert(f.BankPoints()>original_points);
    for(unsigned j=0;j<6;++j)assert(Feed(f,3300000000ULL+j*100000000ULL,j+23,true,2.8,.1,1.35));
    assert(f.Confirmed());input=State(3810000000ULL,1.5,2.275);assert(f.BeginTail(input));
    assert(!f.Step(input).stop);

    // Another legitimate partial overlap: the new view reveals more near
    // ground while its far limit lies inside the previously observed bank.
    f.Reset(0,1);assert(Feed(f,Second,1,false,2.8,.1,1.5));
    assert(Feed(f,1100000000ULL,2,false,2.6,0.,1.25));
    assert(f.BankPoints()>81);

    // Consumed run-up older than60s must not invalidate a recent rear-tail
    // segment. The relevant selected nodes still keep real capture stamps.
    f.Reset(0,1);
    for(unsigned j=0;j<=60;++j) {
        const double base=.1*j;
        assert(Feed(f,Second+Second*j,j+1,false,base+2.6,base,base+1.25));
    }
    for(unsigned j=0;j<6;++j)
        assert(Feed(f,61100000000ULL+j*100000000ULL,j+62,true,8.6,6.,7.25));
    assert(f.Confirmed());input=State(61610000000ULL,5.225,6.);
    assert(f.BeginTail(input)&&!f.Step(input).stop);

    // The tail consumes old run-up while moving: a60s-old segment behind the
    // retained15cm margin no longer controls the age of the recent end.
    f.Reset(0,1);assert(Feed(f,Second,1,false,2.6,0.,1.25));
    assert(Feed(f,40*Second,2,false,3.,.1,1.35));
    for(unsigned j=0;j<6;++j)
        assert(Feed(f,59*Second+j*100000000ULL,j+3,true,3.,.1,1.35));
    assert(f.Confirmed());input=State(59510000000ULL,2.5,3.275);
    assert(f.BeginTail(input));
    for(int j=0;j<400;++j) {
        Move(input,.001,.001);out=f.Step(input);
        assert(f.Active()&&!out.stop&&f.GetStage()!=Stage::Fault);
        assert(out.x>=Follower::ApproachSpeedFloor&&out.x<=.15);
    }
    assert(input.now_ns>61*Second&&out.remaining_arc<.12);

    // Conversely, a segment still needed by the current actualTCP cannot
    // be refreshed merely by receiving new camera metadata or the same cap.
    Confirm(f);assert(Feed(f,60400000000ULL,7,true));
    input=State(60410000000ULL,1.35,2.125);assert(f.BeginTail(input));
    for(int j=0;j<200&&f.GetStage()!=Stage::Fault;++j){Move(input,0.,0.);out=f.Step(input);}
    assert(f.GetStage()==Stage::Fault&&out.stop&&!f.Completed());
    std::cout<<"PASS observed terminal cap, independent frames, replay/session isolation, finite overlap cache, actual TCP arc, bounded commands, stop-before-completion, controlled stand and explicit restart\n";
}
