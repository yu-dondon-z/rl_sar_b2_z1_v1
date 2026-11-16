#include "va15_remote_input.hpp"
#include <cassert>
#include <iostream>
using R=VA15RemoteInput;
int main() {
    R r;r.Update(0,true);r.Update(0,true);
    assert(r.Update(R::A,true).event==R::Event::GetUp);
    assert(r.Update(R::A,true).event==R::Event::None);
    r.Update(0,true);
    assert(r.Update(R::Y,true).event==R::Event::Policy);
    r.Update(0,true);
    assert(r.Update(R::R1|R::Y,true).event==R::Event::Vision);
    for(int i=0;i<100;++i) assert(r.Update(R::R1|R::Y,true).event==R::Event::None);
    assert(r.Update(R::Y,true).event==R::Event::None); // release R1 cannot enter policy
    r.Update(0,true);
    assert(r.Update(R::R1|R::Up,true).event==R::Event::Policy);
    r.Update(0,true);r.Update(0,true);
    auto s=r.Update(R::X,false);
    assert(s.event==R::Event::Stop && s.zero && s.inhibit);
    assert(r.Update(0,false).inhibit); // holding a stick cannot undo X
    assert(r.Update(0,true).zero);
    assert(!r.Update(0,false).inhibit);
    assert(r.Update(R::B|R::Y|R::R1,false).event==R::Event::GetDown);
    r.Update(R::B,false);
    assert(r.Update(R::B|R::Y|R::R1,false).event==R::Event::None);
    for(int i=0;i<20;++i) assert(r.Update(0,true).inhibit);
    assert(r.Update(R::Y,true).inhibit); // cannot release lowering inhibit with Y
    r.Update(0,true);
    assert(r.Update(R::A,true).event==R::Event::GetUp);
    r.Update(0,true);assert(!r.Update(0,true).inhibit);
    auto e=r.Update(R::L1|R::X|R::B|R::R1|R::Y,false);
    assert(e.event==R::Event::Emergency && e.inhibit);
    r.Update(0,true);
    assert(r.Update(R::A,true).event==R::Event::None);
    assert(r.Update(0,true).inhibit); // only process restart clears emergency
    R held;
    assert(held.Update(R::Y,true).event==R::Event::None);
    assert(held.Update(R::Y,true).event==R::Event::None);
    held.Update(0,true);
    assert(held.Update(R::Y,true).event==R::Event::Policy);
    R emergency;
    assert(emergency.Update(R::L1|R::X,false).event==R::Event::Emergency);
    assert(R::VisionReady(true,true,false,false)); // prepared standby, actor can remain idle
    assert(!R::VisionReady(false,true,false,false));
    assert(!R::VisionReady(true,false,false,false));
    assert(!R::VisionReady(true,true,true,false));
    assert(!R::VisionReady(true,true,false,true));
    std::cout<<"PASS: familiar remote keys, chord precedence, one event per press, stop/recenter, lowering and emergency latches\n";
}
