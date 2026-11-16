#include "production/visual_tcp.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
using namespace va15_tcp;
static va15_visual::Packet Frame(unsigned n,std::uint64_t stamp) {
    va15_visual::Packet p;p.valid=p.session_matched=true;p.sequence=n;p.capture_ns=stamp;p.publish_ns=stamp+10000000;
    p.u=p.look_u=935.;p.v=972.;p.look_v=820.8;p.confidence=p.look_confidence=.99;p.width=30.;p.contrast=.8;p.rms=1.;
    for(int v=1044;v>=744;v-=6)p.path.push_back({935.,double(v)});
    p.path_count=p.path.size();p.path_span=300.;return p;
}
int main() {
    // Exact recorded sign failure: an estimated -0.129 slip must not turn a
    // right-side metric reference (-0.0919) into a positive command.
    constexpr std::uint64_t start=10000000000ULL;
    va15_visual::Guide g(true,true,true);
    for(unsigned i=1;i<=6;++i){auto p=Frame(i,start+i*100000000ULL);g.Observe(p,p.publish_ns);}
    g.metric_bearing_valid=true;g.metric_bearing=-.0919;g.course_slip=-.129;
    assert(g.Start(start+610000000,true,true,true));
    assert(g.Tick(start+610000000,.005,true,false,true).yaw<0.);
    g.body_reference_valid=g.body_history_used=true;g.body_lateral=-.2;g.body_heading=0.;g.course_slip=-.30;
    assert(g.Tick(start+615000000,.005,true,false,true).yaw<0.);
    g.body_lateral=.2;g.body_heading=0.;
    for(int i=0;i<500;++i)g.Tick(start+620000000,.005,true,false,true);
    assert(g.yaw_reference>0.);
    // Recorded V249 curve entry: front left, extrapolated body right. No
    // ACTUAL under-body history means no PI authority, even if a caller
    // accidentally sets the validity flag for that extrapolation.
    g.metric_bearing=.041;g.body_lateral=-.06;g.body_heading=.083;g.body_history_used=false;
    g.Stop("test explicit restart");assert(g.Start(start+630000000,true,true,true));
    for(int i=0;i<100;++i)assert(g.Tick(start+630000000,.005,true,false,true).yaw>0.);
    assert(g.body_control_weight==0.&&g.visual_integral==0.);

    // Retain actual front observations through the real slow traverse. The
    // old 192-frame cache at 10 Hz only covered 0.768 m at this speed.
    Tracker tracker;Pose pose{start,{0.,0.,.52},Eigen::Quaterniond::Identity(),true};tracker.Begin(pose);
    unsigned active=0;double first=-1.;
    for(unsigned i=1;i<=700;++i) {
        const double t=i*.1;pose.ns=start+i*100000000ULL;pose.p.x()=.04*t;
        assert(tracker.Observe(Frame(i,pose.ns),pose));
        assert(tracker.Step(pose,.005,true));
        assert(tracker.curves.size()<=Tracker::MaximumCurves);
        if(tracker.active){++active;if(first<0.)first=t;}
    }
    assert(active>50&&first>20.&&first<65.);
    assert(tracker.rear_crossings>0&&tracker.workspace_rejections==0);
    double e=0.,heading=0.;bool history=false;
    assert(tracker.BodyGuidance(pose,e,heading,history)&&history&&std::fabs(e)<.03);
    const auto before=tracker.target;
    pose.p.y()=.5;tracker.Step(pose,.005,true);
    assert(!tracker.active&&tracker.rear_crossings>0&&tracker.workspace_rejections>0&&(before-tracker.target).norm()==0.);
    tracker.ClearObservations();assert(tracker.curves.empty()&&!tracker.live_valid);
    assert(!tracker.Guidance(pose,e));
    tracker.Step(pose,.005,true);assert(!tracker.active);

    // No backward extrapolation of front geometry into a body reference.
    tracker.Reset();pose={start,{0.,0.,.52},Eigen::Quaterniond::Identity(),true};
    Tracker::Curve c;c.ns=start;c.capture=pose;c.near={1.2,-.2,0.};c.look={2.,-.2,0.};
    for(int i=0;i<20;++i)c.points.push_back({1.1+i*.06,-.2,0.});
    tracker.live=c;tracker.live_valid=true;tracker.curves.push_back(c);
    assert(!tracker.BodyGuidance(pose,e,heading,history)&&!history);
    assert(tracker.Step(pose,.005,true)&&!tracker.active);
    for(auto& p:tracker.live.points)p.y()=.2;
    tracker.live.near.y()=tracker.live.look.y()=.2;tracker.curves.clear();
    assert(!tracker.BodyGuidance(pose,e,heading,history)&&!history);
    const auto size=tracker.curves.size();
    for(int i=0;i<20;++i)tracker.BodyGuidance(pose,e,heading,history);
    assert(tracker.curves.size()==size); // no tangent extrapolation in TCP bank
    c.points={{-.1,.04,0.},{.1,.04,0.}};tracker.curves.push_back(c);
    assert(tracker.BodyGuidance(pose,e,heading,history)&&history&&std::fabs(e-.04)<1.e-9);

    // Adaptive fore/aft allocation remains ON a real stored segment and IN
    // the original training workspace. Nominal x=-.775 is unreachable here,
    // but a more forward observed point is reachable without changing y limits.
    tracker.Reset();tracker.Begin(pose);Tracker::Curve rear;
    rear.ns=pose.ns;rear.capture=pose;rear.points={{-1.,.25,0.},{-.5,.05,0.}};
    tracker.curves.push_back(rear);tracker.Step(pose,.005,true);
    assert(tracker.active&&tracker.desired.x()>-.775&&tracker.desired.x()<=-.55);
    assert(std::fabs(tracker.desired.y())<=.15);
    assert(std::fabs(tracker.desired.y()-(.25-.4*(tracker.desired.x()+1.)))<1.e-9);
    assert((tracker.target-V3(-.775,0.,-.35)).cwiseAbs().maxCoeff()<=.000201);

    std::cout<<"PASS body sign/cross-track, slip disabled, slow observed rear activation at "<<first
             <<"s, sparse bounded cache, workspace hold, pose reset and no unseen rear target\n";
}
