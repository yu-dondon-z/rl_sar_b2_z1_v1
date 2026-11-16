#include "production/near_visual_guide.hpp"
#include <cassert>
#include <iostream>
using namespace va15_visual;

static Packet Frame(unsigned seq,std::uint64_t now,bool short_curve=false,double shift=0.) {
    Packet p;p.sequence=seq;p.capture_ns=now-100000000;p.publish_ns=now;
    p.valid=p.session_matched=true;p.u=935.+shift;p.v=972.;
    p.width=30.;p.rms=1.;p.confidence=p.look_confidence=.99;p.contrast=.8;
    p.look_v=short_curve?852.:820.8;
    for(int v=1044;v>=(short_curve?840:744);v-=6)
        p.path.push_back({935.+shift+.0006*(v-972.)*(v-972.),double(v)});
    p.path_count=p.path.size();p.path_span=p.path.front()[1]-p.path.back()[1];
    p.look_u=CurveU(p,p.look_v);
    return p;
}
int main() {
    Guide g(true,true,true);std::uint64_t now=10000000000ULL;unsigned seq=0;
    for(int k=0;k<6;++k){now+=120000000;g.Observe(Frame(++seq,now),now);}
    assert(g.Ready(now)&&g.Start(now,true,true,true));
    for(int k=0;k<120;++k) {
        now+=120000000;g.Observe(Frame(++seq,now,k%2),now);
        const auto out=g.Tick(now,.005,true,false,true);
        assert(!out.stop&&out.x==.15&&g.qualified>=6);
    }
    auto bad=Frame(++seq,now+120000000,true);bad.look_v=900.;
    assert(!Valid(bad,now+120000000));
    bad=Frame(++seq,now+120000000,true);bad.path_span=180.;assert(!Valid(bad,now+120000000));
    auto missing=Frame(++seq,now+120000000,true);missing.path.clear();
    assert(!std::isfinite(AssociatedLookChange(missing,Frame(1,now,false))));
    auto different=Frame(++seq,now+120000000,true,140.);
    g.Observe(different,now+120000000);
    assert(g.Tick(now+120000000,.005,true,false,true).stop);
    std::cout<<"PASS bounded observed lookahead/common-row identity, 120 alternating frames, real switch and missing support rejection\n";
}
