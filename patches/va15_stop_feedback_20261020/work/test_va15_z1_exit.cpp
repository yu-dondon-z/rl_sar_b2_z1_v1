// No robot endpoints. The generic helper is checked with a fake transport.
#include "va15_z1_exit.hpp"
#include <cassert>
#include <iostream>

struct FakeTransport
{
    enum class Reply{Passive,Silent,OtherState,WrongHeader,Truncated};
    Reply reply;
    unsigned sends=0;
    bool reset=false;
    explicit FakeTransport(Reply value):reply(value){}
    void resetIO(UNITREE_ARM::BlockYN block,std::size_t bytes,std::size_t timeout)
    {
        assert(block==UNITREE_ARM::BlockYN::NO);
        assert(bytes==UNITREE_ARM::RECVSTATE_LENGTH);
        assert(timeout==20000);
        reset=true;
    }
    std::size_t send(std::uint8_t* buffer,std::size_t bytes)
    {
        assert(reset && bytes==sizeof(UNITREE_ARM::SendCmd));
        UNITREE_ARM::SendCmd frame;
        std::memcpy(&frame,buffer,bytes);
        assert(frame.head[0]==0xfe && frame.head[1]==0xef);
        assert(frame.state==UNITREE_ARM::ArmFSMState::PASSIVE && !frame.track);
        const auto* payload=reinterpret_cast<const unsigned char*>(&frame.valueUnion);
        for(std::size_t i=0;i<sizeof(frame.valueUnion);++i)assert(payload[i]==0);
        ++sends;
        return bytes;
    }
    std::size_t recv(std::uint8_t* buffer,std::size_t bytes)
    {
        if(reply==Reply::Silent)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));return 0;
        }
        UNITREE_ARM::RecvState state{};
        state.head[0]=0xfe;state.head[1]=reply==Reply::WrongHeader?0x00:0xef;
        state.state=reply==Reply::OtherState?UNITREE_ARM::ArmFSMState::LOWCMD:UNITREE_ARM::ArmFSMState::PASSIVE;
        std::memcpy(buffer,&state,bytes);
        return reply==Reply::Truncated?bytes-1:bytes;
    }
};

int main()
{
    UNITREE_ARM::SendCmd previous{};
    std::memset(&previous,0xab,sizeof(previous));
    previous.head[0]=0xfe;previous.head[1]=0xef;
    previous.state=UNITREE_ARM::ArmFSMState::LOWCMD;
    UNITREE_ARM::RecvState state{};
    state.head[0]=0xfe;state.head[1]=0xef;
    for(const auto mode:{FakeTransport::Reply::Passive,FakeTransport::Reply::Silent,
        FakeTransport::Reply::OtherState,FakeTransport::Reply::WrongHeader,FakeTransport::Reply::Truncated})
    {
        FakeTransport transport(mode);
        const auto started=std::chrono::steady_clock::now();
        const auto result=VA15RequestZ1PassiveOnExit(transport,previous,state);
        assert(result.passive_reply==(mode==FakeTransport::Reply::Passive));
        assert(result.attempts==transport.sends && transport.sends>0 && transport.sends<=10);
        if(mode==FakeTransport::Reply::Passive)assert(transport.sends==1);
        assert(std::chrono::steady_clock::now()-started<std::chrono::seconds(2));
        assert(previous.state==UNITREE_ARM::ArmFSMState::LOWCMD); // no falsified SDK state
        assert(state.state==UNITREE_ARM::ArmFSMState::INVALID);
    }
    std::cout<<"PASS: five bounded PASSIVE-only shutdown cases; no SDK/network/robot initialized\n";
}
