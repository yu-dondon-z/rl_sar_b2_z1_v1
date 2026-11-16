#include "production/visual_rear_completion.hpp"
#include <cassert>
#include <iostream>
using va15_visual_end::RearCompletion;
using namespace va15_tcp;
static va15_visual::Packet Frame(unsigned seq,std::uint64_t stamp) {
    va15_visual::Packet p;p.valid=p.session_matched=p.image_received=true;
    p.sequence=seq;p.capture_ns=stamp;p.publish_ns=stamp+10000000ULL;
    p.u=p.look_u=935.;p.v=972.;p.look_v=820.8;p.confidence=p.look_confidence=.99;
    p.width=30.;p.contrast=.8;p.rms=1.;
    for(int v=1044;v>=744;v-=6)p.path.push_back({935.,double(v)});
    p.path_count=p.path.size();p.path_span=300.;p.endpoint_candidate=true;
    p.endpoint_u=935.;p.endpoint_v=744.;p.endpoint_confidence=.99;p.endpoint_clear_pixels=126.;return p;
}
static constexpr std::uint64_t epoch=10000000000ULL;
static Pose Capture(std::uint64_t stamp){return {stamp,{0.,0.,.52},Eigen::Quaterniond::Identity(),true};}
static RearCompletion Confirm() {
    RearCompletion e;e.Begin(7);
    for(unsigned n=1;n<=6;++n){auto p=Frame(n,epoch+n*100000000ULL);e.Observe(p,Capture(p.capture_ns),p.publish_ns,true,7);}
    assert(e.Confirmed()&&!e.Requested()&&!e.Stood());assert(e.frames==6);
    return e;
}
int main() {
    auto e=Confirm();auto pose=Capture(epoch+700000000ULL);
    const V3 endpoint=e.endpoint;assert(endpoint.x()>1.&&std::fabs(endpoint.z())<1.e-6);
    const V3 before=endpoint-2.*e.tangent+V3(0.,0.,.09);
    auto out=e.Tick(pose.ns,.005,pose,before,true,7,true,false,true,0.);
    assert(out.x==va15_visual::Speed&&!out.stop&&!e.Requested());
    // Old camera data is not refreshed. Only a previously confirmed finite
    // endpoint permits terminal progress with fresh measured pose/FK.
    pose.ns+=1000000000ULL;
    out=e.Tick(pose.ns,.005,pose,before,true,7,true,false,true,0.);
    assert(out.x>0.&&!out.stop);
    const V3 at=endpoint+V3(0.,0.,.09);
    for(int i=0;i<20;++i){pose.ns+=5000000;e.Tick(pose.ns,.005,pose,at,true,7,true,false,true,0.);}
    assert(e.Requested()&&!e.Stood());
    for(int i=0;i<150;++i)e.Park(.005,true,false,at,true);
    assert(!e.Stood());
    for(int i=0;i<100;++i)e.Park(.005,true,true,at,true);
    assert(e.Stood());
    // FK, rather than commanded-target closeness, owns arrival.
    for(int bad=0;bad<12;++bad) {
        auto t=Confirm();pose=Capture(epoch+700000000ULL);V3 measured=before;
        bool valid=true,healthy=true,stop=false,center=true;unsigned gen=7;
        std::uint64_t now=pose.ns;
        if(bad==0)stop=true;
        if(bad==1)healthy=false;
        if(bad==2)center=false;
        if(bad==3)gen=8;
        if(bad==4)pose.valid=false;
        if(bad==5)now+=100000001ULL;
        if(bad==6)valid=false;
        if(bad==7)measured.x()=NAN;
        if(bad==8){pose.ns+=61000000000ULL;now=pose.ns;}
        if(bad==9)measured=endpoint+.13*t.tangent+V3(0.,0.,.09);
        if(bad==10)measured=before+V3(0.,.4,0.);
        if(bad==11)pose.q=Eigen::Quaterniond(Eigen::AngleAxisd(.7,V3::UnitZ()));
        auto r=t.Tick(now,.005,pose,measured,valid,gen,healthy,stop,center,0.);
        assert(r.stop&&!t.Requested()&&!t.Stood()&&t.stage==RearCompletion::Stage::Interrupted);
    }
    // A misplaced or elevated measured TCP cannot complete, even at an
    // apparently correct commanded target or a static four-foot stand.
    for(const V3 error:{V3(0.,.08,0.),V3(0.,0.,.08)}) {
        auto t=Confirm();pose=Capture(epoch+700000000ULL);
        for(int i=0;i<100;++i){pose.ns+=5000000;t.Tick(pose.ns,.005,pose,at+error,true,7,true,false,true,0.);}
        assert(!t.Requested()&&!t.Stood());
    }
    // Arrival is rechecked during supported stopping; a post-brake drift
    // never gets reported as normal completion.
    e=Confirm();pose=Capture(epoch+700000000ULL);
    for(int i=0;i<20;++i){pose.ns+=5000000;e.Tick(pose.ns,.005,pose,at,true,7,true,false,true,0.);}
    for(int i=0;i<2100;++i)e.Park(.005,true,true,at+V3(.2,0.,0.),true);
    assert(!e.Stood()&&e.stage==RearCompletion::Stage::Interrupted);
    // Single/duplicate caps, stale frames, unknown loss, projection failure,
    // wrong capture pose and moving endpoints cannot authorize cached motion.
    for(int bad=0;bad<7;++bad) {
        RearCompletion t;t.Begin(7);
        for(unsigned n=1;n<=10;++n) {
            auto p=Frame(bad==0?1:n,epoch+(bad==0?1:n)*100000000ULL);auto c=Capture(p.capture_ns);
            if(bad==1)p.endpoint_candidate=n==1;
            if(bad==2)p.valid=false;
            if(bad==3)p.endpoint_u=1600.;
            if(bad==4)c.ns+=1;
            if(bad==5)c.p.x()=.15*n;
            t.Observe(p,c,p.publish_ns+(bad==6?600000000ULL:0),true,7);
        }
        assert(!t.Confirmed()&&!t.Requested());
    }
    Tracker tracker;pose=Capture(epoch);tracker.Begin(pose);
    const auto seed=tracker.target;assert(!tracker.StepEndpoint(pose,.005,endpoint));
    assert((tracker.target-seed).norm()==0.);
    pose.p=endpoint+V3(.775,0.,.52);
    for(int i=0;i<1000;++i) {
        const auto prior=tracker.target;assert(tracker.StepEndpoint(pose,.005,endpoint));
        assert((tracker.target-prior).cwiseAbs().maxCoeff()<=.000200001);
        assert(tracker.desired.x()>=-1.&&tracker.desired.x()<=-.55&&std::fabs(tracker.desired.y())<=.15);
    }
    std::cout<<"PASS finite observed endpoint, no early stop, measured FK arrival, supported completion, interruption and original TCP workspace/rates\n";
}
