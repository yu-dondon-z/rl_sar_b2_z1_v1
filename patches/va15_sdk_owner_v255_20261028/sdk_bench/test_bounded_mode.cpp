#include "bounded_mode.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace va15_z1_bench;

int main() {
    Feedback f;f.valid=f.healthy=true;f.generation=1;f.received_ns=1000000000;f.mode=Mode::PASSIVE;
    f.q={.01,.02,.03,.04,.05,.06};
    BoundedMode mode;
    auto now=f.received_ns;
    auto d=mode.Tick(now,true,false,f);
    assert(d.request==Mode::PASSIVE&&!d.permit_policy_frame);
    // A pre-request buffered PASSIVE is not a fresh handshake reply.
    d=mode.Tick(++now,true,false,f);assert(mode.stage==BoundedMode::Stage::WaitPassive);
    for(unsigned cycle=0;cycle<3;++cycle) {
        f.received_ns=now+=2000000;++f.generation;f.mode=Mode::PASSIVE;
        d=mode.Tick(now,true,false,f);assert(mode.stage==BoundedMode::Stage::Passive);
        d=mode.Tick(++now,true,false,f);
        assert(mode.stage==BoundedMode::Stage::WaitLowCmd&&d.request==Mode::LOWCMD);
        assert(d.initialize_sdk_defaults&&d.seed_measured_q&&d.seed==f.q&&!d.permit_policy_frame);
        d=mode.Tick(++now,true,false,f);assert(!d.initialize_sdk_defaults&&!d.permit_policy_frame);
        f.received_ns=now+=2000000;++f.generation;f.mode=Mode::LOWCMD;
        d=mode.Tick(now,true,false,f);assert(d.activation_edge&&!d.permit_policy_frame&&d.seed==f.q);
        d=mode.Tick(++now,true,false,f);assert(d.permit_policy_frame);
        // Stop wins even with a simultaneous/repeated active request.
        d=mode.Tick(++now,true,true,f);
        assert(mode.stage==BoundedMode::Stage::WaitPassive&&d.request==Mode::PASSIVE&&!d.permit_policy_frame);
    }
    BoundedMode timeout;timeout.Tick(now,false,false,f);
    const auto started=now;
    for(unsigned i=1;i<500;++i) {
        d=timeout.Tick(started+i*1000000,true,false,f);
        assert(!d.permit_policy_frame&&d.request==Mode::PASSIVE);
    }
    d=timeout.Tick(started+BoundedMode::BudgetNs,true,false,f);
    assert(timeout.stage==BoundedMode::Stage::Fault&&d.request==Mode::PASSIVE);
    f.received_ns=started+BoundedMode::BudgetNs+1;++f.generation;f.mode=Mode::PASSIVE;
    d=timeout.Tick(f.received_ns,true,false,f);assert(timeout.stage==BoundedMode::Stage::Fault);
    // Stale, corrupt and regressing sources never earn policy permission.
    BoundedMode bad;bad.Tick(now,true,false,f);f.generation=0;
    assert(!bad.Tick(now+1,true,false,f).permit_policy_frame&&bad.stage==BoundedMode::Stage::Fault);
    std::cout<<"PASS bounded software coordinator: fresh explicit PASSIVE/LOWCMD cycles, stop priority, timeout, no auto re-arm; prototype not deployed\n";
}
