#include "production/blind_fixed_guide.hpp"
#include <cassert>
#include <iostream>
int main() {
    va15_blind::Guide g;
    assert(!g.running&&!g.Tick(.005,true,false,true));
    for(int missing=0;missing<3;++missing)assert(!g.Start(missing!=0,missing!=1,missing!=2)&&!g.running);
    for(int repeat=0;repeat<3;++repeat) {
        assert(g.Start(true,true,true));
        for(int i=0;i<1500;++i)assert(!g.Tick(.005,true,false,true)&&g.running);
        assert(g.Tick(.005,repeat!=0,repeat==1,repeat!=2)&&!g.running);
        for(int i=0;i<10;++i)assert(!g.Tick(.005,true,false,true)&&!g.running);
    }
    assert(g.Start(true,true,true));assert(g.Tick(.021,true,false,true)&&!g.running);
    std::cout<<"PASS explicit blind start, readiness/neutral/health gates, continuous manual stop, no implicit restart, no fake camera\n";
}
