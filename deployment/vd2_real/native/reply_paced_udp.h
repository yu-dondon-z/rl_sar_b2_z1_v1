#pragma once
#include <unitree_arm_sdk/control/ctrlComponents.h>
#include <chrono>
#include <cstring>

// The controller receives one SDK datagram per iteration. Two independent
// nominal 500 Hz clocks otherwise build a FIFO of stale position references.
// A controller reply grants ONE send, not an acknowledgement of that command.
// This changes neither encoder timestamps nor motor freshness semantics.
class ReplyPacedUDP: public UNITREE_ARM::UDPPort {
public:
    using UDPPort::UDPPort;
    double received=0,source_time=0,sent_time=0,sent_source_time=0;
    double sent_q[6]{};
    unsigned long long sends=0,suppressed=0,replies=0;
    bool pacing=true;

    static double clock_now(){
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    size_t send(uint8_t* data,size_t length)override{
        if(length!=sizeof(UNITREE_ARM::SendCmd))return 0;
        UNITREE_ARM::SendCmd command{};std::memcpy(&command,data,length);
        const double time=clock_now();
        const bool passive=command.state==UNITREE_ARM::ArmFSMState::PASSIVE;
        // A transition to PASSIVE must not wait behind a missing reply.
        // Only PASSIVE retries after loss (50 Hz). Active references must not
        // accumulate while a stalled controller cannot consume its FIFO.
        const bool passive_retry=passive && time-sent_time>=.020;
        if(pacing && sends && !grant && !(passive && !last_passive) && !passive_retry){
            ++suppressed;return 0;
        }
        const auto count=UDPPort::send(data,length);
        if(count==length){
            grant=false;last_passive=passive;sent_time=time;sent_source_time=source_time;++sends;
            for(int j=0;j<6;++j)sent_q[j]=command.valueUnion.jointCmd[j].Pos;
        }
        return count;
    }
    size_t recv(uint8_t* data,size_t length)override{
        const auto count=UDPPort::recv(data,length);
        if(count==sizeof(UNITREE_ARM::RecvState)){
            received=clock_now();grant=true;++replies;
        }
        return count;
    }
    size_t recv(uint8_t* data)override{return recv(data,sizeof(UNITREE_ARM::RecvState));}
private:
    bool grant=true,last_passive=true;
};
