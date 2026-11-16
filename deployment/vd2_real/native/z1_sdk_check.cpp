// SDK serialization against an in-memory reply. The base SDK object binds a
// loopback UDP socket, but overridden methods send no network datagrams.
#include <unitree_arm_sdk/control/ctrlComponents.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
using namespace UNITREE_ARM;
struct MemoryUDP:UDPPort {
    MemoryUDP():UDPPort("127.0.0.1",9,0,sizeof(RecvState)){}
    SendCmd command{};RecvState reply{};bool fresh=true;unsigned receives=0;
    size_t send(uint8_t* data,size_t length)override{
        if(length!=sizeof(SendCmd))throw std::runtime_error("unexpected Z1 command length");
        std::memcpy(&command,data,length);return length;
    }
    size_t recv(uint8_t* data,size_t length)override{
        if(!fresh)return 0;
        if(length!=sizeof(RecvState))throw std::runtime_error("unexpected Z1 state length");
        std::memcpy(data,&reply,length);++receives;return length;
    }
    size_t recv(uint8_t* data)override{return recv(data,sizeof(RecvState));}
};
int main(){
    CtrlComponents component;component.armModel=nullptr;component.dt=.002;
    auto* memory=new MemoryUDP;component.udp=memory;
    component.recvState={};component.sendCmd.track=false;
    memory->reply.state=ArmFSMState::LOWCMD;
    const std::vector<double> kp={20,30,30,20,15,10},kd(6,2000);
    try{
        for(int tick=0;tick<4;++tick){
            component.lowcmd->setControlGain(kp,kd);
            for(int j=0;j<6;++j){
                component.lowcmd->q[j]=.1*(j+1);component.lowcmd->dq[j]=0;component.lowcmd->tau[j]=0;
                memory->reply.jointState[j].Pos=.02*j;
                memory->reply.jointState[j].W=.03*j;
            }
            component.sendCmd.state=ArmFSMState::LOWCMD;component.sendRecv();
            if(tick){
                if(memory->command.state!=ArmFSMState::LOWCMD)throw std::runtime_error("LOWCMD overwritten");
                for(int j=0;j<6;++j){
                    const auto& sent=memory->command.valueUnion.jointCmd[j];
                    if(std::abs(sent.K_P-kp[j])>1e-5 || std::abs(sent.K_W-kd[j])>1e-5 ||
                       std::abs(sent.Pos-.1*(j+1))>1e-6 || std::abs(component.lowstate->q[j]-.02*j)>1e-6)
                        throw std::runtime_error("SDK changed position/gain/state units");
                }
            }
        }
        const auto received=memory->receives;memory->fresh=false;component.sendRecv();
        if(memory->receives!=received)throw std::runtime_error("cached receive counted as a new packet");
        std::cout<<"PASS: Z1 actual SDK raw gain/position serialization and encoder decoding; memory replies only\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
    return 0;
}
