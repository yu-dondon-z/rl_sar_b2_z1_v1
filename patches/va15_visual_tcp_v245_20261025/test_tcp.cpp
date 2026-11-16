#include "production/visual_tcp.hpp"
#include <cassert>
#include <iostream>
using namespace va15_tcp;
int main() {
    Pose p{1000000000ULL,V3(0,0,.52),Eigen::Quaterniond::Identity(),true};
    V3 point;assert(Project(p,{935.,972.},point));
    assert(std::fabs(point.z())<1.e-8&&point.x()>1.&&point.x()<2.);
    const V3 original=point;
    p.p.x()=2.;p.q=Eigen::Quaterniond(Eigen::AngleAxisd(.7,V3::UnitZ()));
    assert(Project(p,{935.,972.},point));
    assert((point-(p.q*original+V3(2,0,0))).norm()<1.e-6);
    assert(!Project(p,{935.,300.},point));
    History h;assert(h.Add(p));auto p2=p;p2.ns+=20000000;p2.p.x()+=.01;
    assert(h.Add(p2));Pose middle;assert(h.At(1010000000ULL,middle));
    assert(std::fabs(middle.p.x()-2.005)<1.e-10);
    assert(!h.At(1100000000ULL,middle));
    p2.ns+=20000000;p2.p.x()+=1.;assert(h.Add(p2));assert(h.generation==1);
    Tracker tracker;p={1000000000ULL,V3(0,0,.52),Eigen::Quaterniond::Identity(),true};tracker.Begin(p);
    assert(tracker.Step(p,.005,true)&&!tracker.active);assert((tracker.target-V3(-.775,0,-.35)).norm()==0.);
    va15_visual::Packet packet;packet.valid=packet.session_matched=true;packet.sequence=1;packet.capture_ns=p.ns;
    packet.u=packet.look_u=935.;packet.v=972.;packet.look_v=820.8;
    for(int v=1044;v>=744;v-=6)packet.path.push_back({935.,double(v)});
    assert(tracker.Observe(packet,p));
    double bearing=0.;assert(tracker.Guidance(p,bearing)&&std::fabs(bearing)<.02);
    for(int i=0;i<2000;++i) {
        p.ns+=5000000;p.p.x()=2.2;
        const V3 before=tracker.target;assert(tracker.Step(p,.005,true));
        assert((tracker.target-before).cwiseAbs().maxCoeff()<=.00020001);
    }
    assert(tracker.active&&std::fabs(tracker.target.y())>.001);
    assert(std::fabs(tracker.clearance-.09)<1.e-6&&tracker.target.z()<-.40&&tracker.target.z()>=-.55);
    const auto frozen=tracker.target;
    assert(tracker.Step(p,.005,false)&&!tracker.active&&(tracker.target-frozen).norm()==0.);
    p.p.y()=3.;assert(tracker.Step(p,.005,true)&&!tracker.active&&(tracker.target-frozen).norm()==0.);
    p.ns+=46000000000ULL;tracker.Step(p,.005,true);assert(tracker.curves.empty());
    va15_visual::Guide guide(true,true,true);
    std::uint64_t now=10000000000ULL;unsigned seq=0;
    auto frame=[&]() {
        va15_visual::Packet f;f.valid=f.session_matched=true;f.sequence=++seq;f.capture_ns=now-100000000;
        f.publish_ns=now;f.u=f.look_u=935;f.v=972;f.look_v=820.8;f.confidence=f.look_confidence=.99;
        f.width=30;f.contrast=.8;f.rms=1.;f.path_count=50;f.path_span=300.;return f;
    };
    for(int i=0;i<6;++i){now+=150000000;guide.Observe(frame(),now);}assert(guide.Start(now,true,true,true));
    const auto accepted=guide.latest.capture_ns;
    now+=150000000;auto miss=frame();miss.valid=false;guide.Observe(miss,now);
    assert(guide.Tick(now,.005,true,false,true).x==.15&&guide.latest.capture_ns==accepted);
    assert(guide.Tick(accepted+va15_visual::MaximumAgeNs+1,.005,true,false,true).stop);
    // Velocity-course correction is bounded, and a corrected metric angle
    // can remain valid while the old camera-origin ray leaves its envelope.
    va15_visual::Guide course(true,true,true);seq=0;
    for(int i=0;i<6;++i){now+=150000000;course.Observe(frame(),now);}
    for(int i=0;i<1000;++i)course.ObserveBodyVelocity(.15,-.03,.005);
    assert(course.course_slip<-.19&&course.course_slip>-.21);
    course.metric_bearing_valid=true;course.metric_bearing=course.course_slip;
    assert(course.Start(now,true,true,true));assert(std::fabs(course.Tick(now,.005,true,false,true).yaw)<1.e-8);
    auto modified=course.seen;modified.width+=1.;course.Observe(modified,now);
    assert(course.Tick(now,.005,true,false,true).stop);
    std::cout<<"PASS flat projection, full-pose transforms, local history interpolation/reset, rear observed-only TCP, bounded blend, stale freeze, short visual dropout\n";
}
