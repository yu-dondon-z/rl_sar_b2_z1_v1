#include "production/visual_front_end_stop.hpp"
#include <cassert>
#include <iostream>
using va15_visual_end::Stop;
static va15_visual::Packet Frame(unsigned seq,std::uint64_t t,bool cap=true) {
    va15_visual::Packet p;p.valid=p.session_matched=p.image_received=true;
    p.sequence=seq;p.capture_ns=t;p.publish_ns=t+10000000ULL;
    p.u=p.look_u=935.;p.v=972.;p.look_v=820.8;p.confidence=p.look_confidence=.99;
    p.width=30.;p.contrast=.8;p.rms=1.;
    for(int v=1044;v>=744;v-=6)p.path.push_back({935.,double(v)});
    p.path_count=p.path.size();p.path_span=300.;
    p.endpoint_candidate=cap;p.endpoint_u=935.;p.endpoint_v=744.;p.endpoint_confidence=.99;p.endpoint_clear_pixels=126.;return p;
}
int main() {
    constexpr std::uint64_t t=10000000000ULL;
    Stop idle;idle.Observe(Frame(1,t),t+10000000ULL,true);assert(!idle.Requested());
    Stop s;s.Begin();auto p=Frame(1,t);
    for(int k=0;k<100;++k)s.Observe(p,t+10000000ULL,true);
    assert(s.frames==1&&!s.Requested());
    for(unsigned n=2;n<=6;++n){auto q=Frame(n,t+(n-1)*100000000ULL);s.Observe(q,q.publish_ns,true);}
    assert(s.Requested()&&!s.Stood());
    for(int n=0;n<1000;++n)s.Park(.005,true,false);
    assert(!s.Stood());
    for(int n=0;n<100;++n)s.Park(.005,true,true);
    assert(s.Stood());
    assert(s.Requested()); // no automatic resume or getdown
    for(int bad=0;bad<8;++bad) {
        s.Begin();
        for(unsigned n=1;n<=20;++n) {
            auto q=Frame(n,t+n*100000000ULL);
            if(bad==0)q.endpoint_candidate=false;
            if(bad==1)q.endpoint_confidence=.5;
            if(bad==2)q.endpoint_clear_pixels=0.;
            if(bad==3)q.image_received=false;
            if(bad==4)q.endpoint_v=618.;
            if(bad==5)q.endpoint_u=1100.;
            if(bad==6)q.valid=false;
            s.Observe(q,q.publish_ns,bad!=7);
        }
        assert(!s.Requested()&&!s.Stood());
    }
    // Neural/RGB cap flicker and generic lost images cannot be END COMPLETE.
    s.Begin();for(unsigned n=1;n<=20;++n){auto q=Frame(n,t+n*100000000ULL,n%4==0);s.Observe(q,q.publish_ns,true);}
    assert(!s.Requested());
    s.Begin();for(unsigned n=1;n<=6;++n){auto q=Frame(n,t+n*20000000ULL);s.Observe(q,q.publish_ns,true);}
    assert(!s.Requested()); // six frames shorter than .5s is not enough
    s.Begin();for(unsigned n=1;n<=6;++n){auto q=Frame(n,t+n*100000000ULL);s.Observe(q,q.publish_ns,true);}
    s.Interrupt("operator X/B/fault");s.Park(.005,true,true);assert(!s.Requested()&&!s.Stood());
    s.Begin();p=Frame(5,t);s.Observe(p,p.publish_ns,true);p=Frame(4,t+100000000ULL);
    s.Observe(p,p.publish_ns,true);assert(s.stage==Stop::Stage::Interrupted);
    std::cout<<"PASS front-camera cap confirmation, no lost-line completion, supported stop only and explicit restart\n";
}
