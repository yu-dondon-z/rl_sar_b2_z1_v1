#include "production/visual_tcp.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
using namespace va15_tcp;
int main() {
    const std::uint64_t now=10000000000ULL;
    Pose pose{now,{0.,0.,.52},Eigen::Quaterniond::Identity(),true};
    Tracker tracker;Tracker::Curve live;live.ns=now-100000000ULL;live.capture=pose;
    live.near={1.4,.05,0.};live.look={2.,-.3,0.};
    live.points={{1.2,.2,0.},{1.4,.05,0.},{2.,-.3,0.}};
    tracker.live=live;tracker.live_valid=true;
    Tracker::ForwardReference reference;double bearing=0.;
    assert(tracker.Guidance(pose,bearing,&reference));
    assert(bearing>0.&&std::fabs(reference.point_body.x()-1.2)<1.e-9&&!reference.historical);
    const double old_far=.6*std::atan2(.05,1.4)+.4*std::atan2(-.3,2.);
    assert(old_far<0.); // far rays pull right, nearest actual tape is left

    Tracker::Curve history;history.ns=now-1000000000ULL;
    history.points={{.5,.08,0.},{.7,.08,0.}};tracker.curves.push_back(history);
    assert(tracker.Guidance(pose,bearing,&reference)&&reference.historical&&bearing>0.);
    assert(std::fabs(reference.point_body.x()-.6)<1.e-9&&reference.capture_ns==history.ns);
    double e=0.,h=0.;bool under_body=false;
    assert(!tracker.BodyGuidance(pose,e,h,under_body)&&!under_body);

    // Most recent actual support wins, never the historical path with the
    // conveniently smallest cross-track distance.
    auto newer=history;newer.ns+=100000000ULL;
    for(auto& p:newer.points)p.y()=.12;
    tracker.curves.push_back(newer);
    assert(tracker.Guidance(pose,bearing,&reference)&&std::fabs(reference.point_body.y()-.12)<1.e-9);

    // Separate observations do not become an invented bridge across a gap.
    tracker.curves.clear();history.points={{.1,-.1,0.},{.2,-.1,0.}};
    newer.points={{.8,-.1,0.},{.9,-.1,0.}};
    tracker.curves.push_back(history);tracker.curves.push_back(newer);
    assert(tracker.Guidance(pose,bearing,&reference)&&!reference.historical&&bearing>0.);

    // Metric angles outside the original envelope stay METRIC, so they
    // stop rather than silently reverting to conflicting far image rays.
    history.points={{.5,.35,0.},{.7,.35,0.}};tracker.curves={history};
    assert(tracker.Guidance(pose,bearing,&reference)&&bearing>.45);
    va15_visual::Guide guide;guide.metric_bearing_valid=true;guide.metric_bearing=bearing;
    assert(!guide.Envelope());
    tracker.live.ns=now-va15_visual::MaximumAgeNs-1;
    assert(!tracker.Guidance(pose,bearing,&reference)&&!reference.capture_ns);
    tracker.live.ns=now;tracker.ClearObservations();
    assert(!tracker.Guidance(pose,bearing,&reference));

    // Bounded-cache host timing only; this is NOT PC4 timing acceptance.
    tracker.live=live;tracker.live_valid=true;tracker.curves.clear();
    for(unsigned i=0;i<Tracker::MaximumCurves;++i) {
        Tracker::Curve c;c.ns=now-1000000000ULL;c.capture=pose;
        for(unsigned j=0;j<50;++j)c.points.push_back({1.+.03*j,.05,0.});
        tracker.curves.push_back(c);
    }
    const auto start=std::chrono::steady_clock::now();
    for(unsigned i=0;i<1000;++i)assert(tracker.Guidance(pose,bearing,&reference));
    const double average_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count()/1000.;
    std::cout<<"PASS observed forward preview, curve direction, newest support, no fake gap/body, envelope and freshness; host average_us="<<average_us<<" (not board timing qualification)\n";
}
