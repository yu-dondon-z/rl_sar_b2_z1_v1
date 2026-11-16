#include "production/tcp_priority_session.hpp"
#include "production/visual_tcp.hpp"
#include "production/trial_supervisor.hpp"
#include <cassert>
#include <iostream>
#include <limits>

namespace {
using namespace va15_visual;
struct Session {
    Guide guide{true,true,true};
    va15_trial::Supervisor trial;
    bool active=false,pending=false;
    std::uint64_t now=10'000'000'000ULL;
    unsigned sequence=0;
    Packet packet;
    Session() {
        trial.Configure(true,true,true);
        for(int n=0;n<100;++n)trial.Step(.005,0.,0.,0.,true,true,false);
        FreshFrames(6);
        assert(guide.Start(now,true,true,true));active=true;
        // Match the actual Armed -> Forward handoff; a positive request is
        // needed while Armed, before Forward can become true.
        assert(trial.Step(.005,.20,0.,0.,true,true,false)>.0);
    }
    Packet Frame() {
        Packet p;p.valid=p.session_matched=true;p.sequence=++sequence;
        p.capture_ns=now-1'000'000ULL;p.publish_ns=now;
        p.u=p.look_u=935.;p.v=972.;p.look_v=820.8;
        p.confidence=p.look_confidence=.99;p.width=30.;p.contrast=.8;p.rms=1.;
        for(int v=1044;v>=744;v-=6)p.path.push_back({935.,double(v)});
        p.path_count=p.path.size();p.path_span=300.;return p;
    }
    void FreshFrames(int count) {
        for(int n=0;n<count;++n){now+=150'000'000ULL;packet=Frame();guide.Observe(packet,now);}
    }
    Output Tick(bool pose=true,bool policy=true,bool healthy=true,bool stop=false,double dt=.005) {
        auto o=va15_tcp_priority::Tick(guide,packet,now,dt,healthy,stop,true,pose,policy,active,pending);
        if(o.stop)trial.Stop(guide.reason);
        return o;
    }
    void Stopped() {
        const auto o=Tick();
        assert(o.stop&&o.x==0.&&o.yaw==0.&&!active&&!pending&&trial.Stopping());
        FreshFrames(6); // camera recovery is NOT a start request
        assert(Tick().x==0.&&!active&&trial.Stopping());
        assert(trial.Step(.005,.20,0.,0.,true,true,false)==0.);
    }
};
}
int main() {
    // Valid live frames sustain both the guide and its original supervisor.
    Session live;
    for(int n=0;n<2400;++n) {
        live.now+=5'000'000ULL;
        if(n%30==0){live.packet=live.Frame();live.guide.Observe(live.packet,live.now);}
        assert(!live.Tick().stop&&live.active&&live.trial.Forward());
    }
    // A new explicit invalid packet must stop even though tolerant Guide
    // still retains a fresh valid previous packet (the pre-fix failure).
    Session invalid;invalid.pending=true;
    invalid.now+=5'000'000ULL;invalid.packet=invalid.Frame();invalid.packet.valid=false;
    invalid.guide.Observe(invalid.packet,invalid.now);
    assert(Valid(invalid.guide.latest,invalid.now));
    invalid.Stopped();
    for(int fault=0;fault<9;++fault) {
        Session s;s.pending=true;
        switch(fault) {
        case 0:s.now=s.packet.capture_ns+MaximumAgeNs+1;break;
        case 1:s.packet.session_matched=false;break;
        case 2:s.packet.publish_ns=s.now+1;break;
        case 3:s.packet.confidence=.1;break;
        case 4:assert(s.Tick(false).stop);break;
        case 5:assert(s.Tick(true,false).stop);break;
        case 6:assert(s.Tick(true,true,false).stop);break;
        case 7:assert(s.Tick(true,true,true,true).stop);break;
        case 8:assert(s.Tick(true,true,true,false,.021).stop);break;
        }
        s.Stopped();
    }
    // Retained rear geometry cannot keep moving a stopped target. This
    // exercises the real Tracker with previously observed rear history.
    Session s;va15_tcp::Tracker tracker;
    va15_tcp::Pose p{s.packet.capture_ns,{0.,0.,.52},Eigen::Quaterniond::Identity(),true};
    tracker.Begin(p);assert(tracker.Observe(s.packet,p));
    p.p.x()=2.2;p.ns=s.now;
    for(int n=0;n<1000;++n)assert(tracker.Step(p,.005,s.active));
    assert(tracker.active&&!tracker.curves.empty());const auto target=tracker.target;
    assert(s.Tick(false).stop);assert(tracker.Step(p,.005,s.active));
    assert(!tracker.active&&tracker.target==target);
    // Restart remains an explicit, supported action after six fresh frames.
    s.FreshFrames(6);assert(s.trial.Rearm(true,true,true,true));
    assert(s.guide.Start(s.now,true,true,true));s.active=true;
    assert(!s.Tick().stop&&s.active);
    std::cout<<"PASS V256 live TCP session: valid 12 s, explicit invalid frame, expiry/session/clock/health/pose/policy/manual/time stops, no blind cache or automatic restart\n";
}
