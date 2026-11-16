#include "production/trial_motor_contract.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace va15_motor_contract;
static Raw Good(){Raw r;r.bit_flag=4;r.mode.fill(10);r.frequency.fill(535);return r;}
static void Feed(Guard& g,Raw& r,int begin,int end) {
    for(int i=begin;i<=end;++i){r.tick=i*10;g.Update(r,double(i)/100.);}
}
int main() {
    Guard g{LostPolicy::DiagnosticOnly};Raw r=Good();
    // Lost changes, decreases, wraps and churn cannot reset health warmup.
    for(int i=0;i<=300;++i){for(int j=0;j<12;++j)r.lost[j]=(i%3==0?0u:i%3==1?5u:0xffffffffu);Feed(g,r,i,i);if(i<300)assert(!g.Snapshot(double(i)/100.).ready);}
    assert(g.Snapshot(3).ready&&g.Snapshot(3).lost_diagnostic_only);
    const auto reference=g.Snapshot(3).baseline;
    for(int i=301;i<1000;++i){r.lost[i%12]=unsigned(i%7);Feed(g,r,i,i);assert(g.Snapshot(double(i)/100.).ready&&!g.Snapshot(double(i)/100.).latched);}
    assert(g.Snapshot(9.99).baseline==reference&&g.Snapshot(9.99).lost_change_events>300);
    assert(!g.Snapshot(10.05).ready); // lost diagnostic mode does not mask stale data
    // Exact observed 0 -> 5 on DDS joints 4/7: no refusal, no raw zeroing.
    Guard observed{LostPolicy::DiagnosticOnly};Raw a=Good();Feed(observed,a,0,300);
    a.lost[4]=a.lost[7]=5;Feed(observed,a,301,301);
    if(!observed.Snapshot(3.01).ready)std::cerr<<"Observed transition refused: "<<Text(observed.Snapshot(3.01).reason)<<" stable="<<observed.Snapshot(3.01).stable_seconds<<" policy="<<observed.Snapshot(3.01).lost_diagnostic_only<<'\n';
    assert(observed.Snapshot(3.01).ready&&a.lost[4]==5&&a.lost[7]==5);
    assert(observed.Snapshot(3.01).lost_change_events==1&&observed.Snapshot(3.01).baseline[4]==0);
    // Fault checks have priority even when lost changes in the SAME packet.
    unsigned checked=0;
    for(int joint=0;joint<12;++joint)for(int kind=0;kind<4;++kind) {
        Guard f{LostPolicy::DiagnosticOnly};Raw q=Good();Feed(f,q,0,300);q.tick=3010;++q.lost[joint];
        if(kind==0)q.fault[joint]=0x100;if(kind==1)q.mode[joint]=0;
        if(kind==2)q.frequency[joint]=399;if(kind==3)q.frequency[joint]=801;
        f.Update(q,3.01);assert(f.Snapshot(3.01).latched&&!f.Snapshot(3.01).ready);
        q=Good();Feed(f,q,302,700);assert(!f.Snapshot(7).ready);++checked;
    }
    for(unsigned flag:{0u,1u,5u,6u,12u,20u,36u,68u,132u,260u}) {
        Guard f{LostPolicy::DiagnosticOnly};Raw q=Good();Feed(f,q,0,300);
        q.tick=3010;q.bit_flag=flag;++q.lost[4];f.Update(q,3.01);
        assert(f.Snapshot(3.01).latched&&!f.Snapshot(3.01).ready);++checked;
    }
    for(int kind=0;kind<6;++kind) {
        Guard f{LostPolicy::DiagnosticOnly};Raw q=Good();Feed(f,q,0,300);++q.lost[4];
        if(kind==0){q.tick=1;f.Update(q,3.01);}
        if(kind==1){q.tick=3100;f.Update(q,3.1);}
        if(kind==2){q.tick=3010;f.Update(q,2.);}
        if(kind==3){q.tick=3010;f.Update(q,std::numeric_limits<double>::quiet_NaN());}
        if(kind==4){f.Update(q,3.02);assert(f.Snapshot(3.02).ready);assert(!f.Snapshot(3.051).ready);continue;}
        if(kind==5){q.tick=3010;f.Update(q,3.05);}
        assert(!f.Snapshot(3.1).ready);++checked;
    }
    // Old versions retain the strict default; this is an isolated opt-in.
    Guard strict;Raw q=Good();Feed(strict,q,0,300);++q.lost[4];Feed(strict,q,301,301);
    assert(strict.Snapshot(3.01).latched&&strict.Snapshot(3.01).reason==Reason::LostChanged);
    std::cout<<"PASS: diagnostic-only raw lost, immutable initial reference, no warmup/motion refusal on lost alone; "<<checked<<" real-fault/timing cases retained; strict default preserved\n";
}
