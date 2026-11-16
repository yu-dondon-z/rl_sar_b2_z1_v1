// OFFLINE ONLY. Encodes elapsed time as joint1 in a loopback echo fixture.
// These values must never be sent to a real controller/robot.
#include "reply_paced_udp.h"
#include <algorithm>
#include <iostream>
#include <thread>
#include <vector>
using namespace UNITREE_ARM;
int main(int argc,char** argv){
    if(argc!=4)return 2;
    const unsigned peer=std::stoul(argv[1]),own=std::stoul(argv[2]);
    if(peer<1024 || own<1024)return 2;
    CtrlComponents component;component.armModel=nullptr;component.dt=.002;
    auto* udp=new ReplyPacedUDP("127.0.0.1",peer,own,sizeof(RecvState),BlockYN::NO,1000);
    udp->pacing=std::stoi(argv[3])!=0;component.udp=udp;component.recvState={};
    component.sendCmd.track=false;
    const double start=udp->clock_now();auto deadline=std::chrono::steady_clock::now();
    std::vector<double> ages;
    while(udp->clock_now()-start<2.){
        const double elapsed=udp->clock_now()-start;
        component.lowcmd->setControlGain(std::vector<double>(6,20),std::vector<double>(6,2000));
        component.lowcmd->q[0]=elapsed;component.sendCmd.state=ArmFSMState::LOWCMD;
        component.sendRecv();
        if(elapsed>.4 && component.lowstate->q[0]>.01)
            ages.push_back(1000*(elapsed-component.lowstate->q[0]));
        deadline+=std::chrono::milliseconds(2);std::this_thread::sleep_until(deadline);
        if(std::chrono::steady_clock::now()>deadline+std::chrono::milliseconds(4))
            deadline=std::chrono::steady_clock::now(); // no catch-up send burst
    }
    if(ages.size()<100)return 3;
    std::sort(ages.begin(),ages.end());
    std::cout<<"{\"paced\":"<<(udp->pacing?"true":"false")<<",\"samples\":"<<ages.size()
        <<",\"p50_ms\":"<<ages[ages.size()/2]<<",\"p95_ms\":"<<ages[ages.size()*95/100]
        <<",\"max_ms\":"<<ages.back()<<",\"sends\":"<<udp->sends
        <<",\"suppressed\":"<<udp->suppressed<<"}\n";
}
