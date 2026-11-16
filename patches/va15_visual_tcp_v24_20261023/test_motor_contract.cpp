#include "production/trial_motor_contract.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace va15_motor_contract;
static Raw Good()
{Raw r;r.bit_flag=4;r.mode.fill(10);r.lost={{0,0,0,6,5,5,5,0,0,0,0,0}};r.frequency.fill(535);return r;}
static void Ready(Guard& g,Raw& r)
{
    for(unsigned i=0;i<=300;++i){r.tick=i*10;g.Update(r,i*.01);}
    assert(g.Snapshot(3.).ready&&!g.Snapshot(3.).latched);
}
int main()
{
    Guard g;Raw r=Good();assert(!g.Snapshot(0).ready);Ready(g,r);
    assert(r.lost==g.Snapshot(3.).baseline&&g.Snapshot(3.).baseline_frozen); // unchanged raw values
    assert(!g.Snapshot(3.051).ready&&g.Snapshot(3.051).latched);
    for(int which=0;which<12;++which)for(int kind=0;kind<6;++kind)
    {
        Guard f;Raw a=Good();Ready(f,a);a.tick=3010;
        if(kind==0)++a.lost[which];
        if(kind==1){if(a.lost[which])--a.lost[which];else a.lost[which]=0xffffffffu;}
        if(kind==2)a.fault[which]=0x100;
        if(kind==3)a.frequency[which]=399;
        if(kind==4)a.frequency[which]=801;
        if(kind==5)a.mode[which]=0;
        f.Update(a,3.01);auto v=f.Snapshot(3.01);
        assert(!v.ready&&v.latched&&v.joint==which);
        a=Good();a.tick=3020;f.Update(a,3.02);
        assert(f.Snapshot(3.02).latched&&!f.Snapshot(3.02).ready);
    }
    for(unsigned flag:{0u,1u,5u,6u,12u,20u,36u,68u,132u,260u})
    {
        Guard f;Raw a=Good();Ready(f,a);a.bit_flag=flag;a.tick=3010;f.Update(a,3.01);
        assert(!f.Snapshot(3.01).ready&&f.Snapshot(3.01).latched);
    }
    Guard boot;Raw a=Good();a.bit_flag=5;
    for(int i=0;i<600;++i){a.tick=i*10;boot.Update(a,i*.01);assert(!boot.Snapshot(i*.01).ready);}
    a.bit_flag=4;
    for(int i=600;i<901;++i){a.tick=i*10;boot.Update(a,i*.01);}
    assert(boot.Snapshot(9.).ready); // timeout may clear BEFORE readiness, never after it
    Guard old;Raw historical=Good();historical.lost={{0,0,0,0,0,5,5,0,0,0,0,0}};
    Ready(old,historical);
    assert(old.Snapshot(3.).baseline!=g.Snapshot(3.).baseline); // different processes, not one hardcoded absolute vector
    Guard startup;Raw changed=Good();
    for(int i=0;i<200;++i){changed.tick=i*10;startup.Update(changed,i*.01);}
    ++changed.lost[3];changed.tick=2000;startup.Update(changed,2.);
    assert(!startup.Snapshot(2.).ready&&!startup.Snapshot(2.).latched);
    assert(startup.Snapshot(2.).stable_seconds==0.&&startup.Snapshot(2.).startup_changes==1);
    for(int i=201;i<500;++i){changed.tick=i*10;startup.Update(changed,i*.01);assert(!startup.Snapshot(i*.01).ready);}
    changed.tick=5000;startup.Update(changed,5.);
    assert(startup.Snapshot(5.).ready&&startup.Snapshot(5.).baseline_frozen);
    changed.tick=5010;--changed.lost[3];startup.Update(changed,5.01);
    assert(startup.Snapshot(5.01).latched&&!startup.Snapshot(5.01).ready);
    Guard churn;Raw noisy=Good();
    for(int i=0;i<1000;++i)
    {
        noisy.tick=i*10;if(i%100==0)++noisy.lost[3];churn.Update(noisy,i*.01);
        assert(!churn.Snapshot(i*.01).ready&&!churn.Snapshot(i*.01).baseline_frozen);
    }
    for(int which=0;which<5;++which)
    {
        Guard f;Raw a=Good();Ready(f,a);
        if(which==0){f.Update(a,3.2);assert(!f.Snapshot(3.2).ready);}
        if(which==1){a.tick=1;f.Update(a,3.01);assert(f.Snapshot(3.01).latched);}
        if(which==2){a.tick=3100;f.Update(a,3.1);assert(f.Snapshot(3.1).latched);}
        if(which==3){a.tick=3010;f.Update(a,2.);assert(f.Snapshot(3.).latched);}
        if(which==4){f.Update(a,std::numeric_limits<double>::quiet_NaN());assert(f.Snapshot(3.).latched);}
    }
    std::cout<<"PASS per-process baseline, distinct historical/current counts, startup window reset/churn, immutable READY, 72 per-motor faults, timeout/sequence/expiry, no runtime rebase/rearm\n";
}
