// OFFLINE ONLY: the real controller SDK receiver with an encoder echo fixture.
// No CtrlComponents/FSM/IOUDP is constructed, and both ports are loopback only.
#include <control/armSDK.h>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
int main(int argc,char** argv){
    if(argc!=4)return 2;
    const unsigned sdk=std::stoul(argv[1]),own=std::stoul(argv[2]);
    const double period=std::stod(argv[3]);
    if(sdk<1024 || own<1024 || period<.002 || period>.01)return 2;
    std::vector<KeyAction*> events;
    ARMSDK panel(events,EmptyAction(0),"127.0.0.1",sdk,own,period);
    RecvState reply{};reply.head[0]=0xfe;reply.head[1]=0xef;reply.state=ArmFSMState::LOWCMD;
    panel.setRecvState(reply);static_cast<CmdPanel&>(panel).start();
    const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(6);
    std::cout<<"loopback controller SDK ready\n"<<std::flush;
    while(std::chrono::steady_clock::now()<end){
        auto command=panel.getSendCmd();
        reply.jointState[0].Pos=command.valueUnion.jointCmd[0].Pos;
        panel.setRecvState(reply);std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}
