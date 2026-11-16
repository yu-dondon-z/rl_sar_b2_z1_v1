#pragma once
#include <cstdint>

// Unitree wireless_remote key bits. Edges are consumed by the control loop
// only; reading state for policy inference must not consume key events.
class VA15RemoteInput {
public:
    enum class Event { None, GetUp, Policy, Vision, Stop, GetDown, Emergency };
    struct Result { Event event=Event::None; bool zero=false; bool inhibit=false; };
    static constexpr std::uint16_t R1=1u<<0, L1=1u<<1,
        A=1u<<8, B=1u<<9, X=1u<<10, Y=1u<<11, Up=1u<<12;
    static bool VisionReady(bool in_policy, bool prepared, bool lowering, bool fault) {
        return in_policy && prepared && !lowering && !fault;
    }
    Result Update(std::uint16_t keys, bool sticks_centered) {
        Result out;
        const auto previous=previous_;
        previous_=keys;
        const auto pressed=[&](std::uint16_t bit) {return (keys&bit) && !(previous&bit);};
        const bool emergency=(keys&(L1|X))==(L1|X);
        if(emergency && !fault_) {out.event=Event::Emergency;fault_=true;}
        if(!initialized_) {
            initialized_=true;
            out.zero=out.inhibit=true;
            return out; // held startup buttons cannot start movement
        }
        if(!fault_) {
            if(keys&B) {if(pressed(B)) out.event=Event::GetDown;}
            else if(keys&X) {if(pressed(X)) out.event=Event::Stop;}
            else if((keys&(R1|Y))==(R1|Y) && (previous&(R1|Y))!=(R1|Y)) out.event=Event::Vision;
            else if(pressed(Y) && !(keys&R1)) out.event=Event::Policy;
            else if(pressed(A)) out.event=Event::GetUp;
            else if((keys&(R1|Up))==(R1|Up) && (previous&(R1|Up))!=(R1|Up)) out.event=Event::Policy;
        }
        if(out.event==Event::GetDown) lowering_=true;
        if(out.event==Event::GetUp) lowering_=false;
        if(out.event!=Event::None && out.event!=Event::Vision) wait_center_=true;
        out.zero=out.inhibit=wait_center_ || lowering_ || fault_;
        if(!keys && sticks_centered && !lowering_ && !fault_) wait_center_=false;
        return out;
    }
private:
    std::uint16_t previous_=0;
    bool initialized_=false,wait_center_=true,lowering_=false,fault_=false;
};
