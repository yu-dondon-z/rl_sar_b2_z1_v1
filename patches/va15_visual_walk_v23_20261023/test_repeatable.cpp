#include "production/near_visual_guide.hpp"
#include "production/trial_supervisor.hpp"
#include <cassert>
#include <iostream>
using namespace va15_visual;
static Packet Frame(unsigned n,std::uint64_t now) {
    Packet p;p.valid=p.session_matched=true;p.sequence=n;p.capture_ns=now-1000000;p.publish_ns=now;
    p.u=p.look_u=935.;p.v=972.;p.look_v=.76*1080.;p.confidence=p.look_confidence=.99;
    p.width=30.;p.contrast=.8;p.rms=1.;p.path_count=50;p.path_span=300.;return p;
}
int main() {
    Guide g(true,true);va15_trial::Supervisor s;s.Configure(true,true,true);
    unsigned sequence=0;std::uint64_t now=10000000000ULL;
    for(int i=0;i<100;++i)s.Step(.005,0.,0.,0.,true,true,false);
    for(int cycle=0;cycle<3;++cycle) {
        for(int n=0;n<6;++n){now+=150000000;g.Observe(Frame(++sequence,now),now);}
        assert(g.Ready(now));
        if(cycle) {
            for(int bad=0;bad<4;++bad)assert(!s.Rearm(bad!=0,bad!=1,bad!=2,bad!=3));
            assert(s.Rearm(true,true,true,true));
        }
        assert(!g.Start(now,false,true,true));assert(!g.Start(now,true,false,true));
        assert(g.Start(now,true,true,true)); // single event, NO Review object
        for(int k=0;k<2400;++k) {
            now+=5000000;if(k%30==0)g.Observe(Frame(++sequence,now),now);
            const auto out=g.Tick(now,.005,true,false,true);
            assert(out.x==.15&&!out.stop&&s.Step(.005,out.x,0.,0.,true,true,false)==.15);
        }
        assert(s.requested>11.99&&g.stage==Guide::Stage::Running); // no four-second deadline
        g.Stop("explicit R1+Y/X/B stop");s.Stop(g.reason);
        for(int k=0;k<600;++k){now+=5000000;g.Observe(Frame(++sequence,now),now);
            assert(g.Tick(now,.005,true,false,true).x==0.&&s.Step(.005,.15,0.,0.,true,true,false)==0.);}
        assert(s.Stopping()); // fresh vision alone never restarts a session
    }
    for(int bad=0;bad<5;++bad) {
        Guide f(true,true);auto t=now;
        for(int n=0;n<6;++n){t+=150000000;f.Observe(Frame(++sequence,t),t);}
        assert(f.Start(t,true,true,true));
        const auto stop=f.Tick(t+(bad==0?MaximumAgeNs+1:5000000),bad==1?.03:.005,bad!=2,bad==3,bad!=4);
        assert(stop.stop&&stop.x==0.);
    }
    std::cout<<"PASS: single-event direct start, 3 continuous 12 s sessions, explicit rearm only, X/B/T and live faults, no automatic restart\n";
}
