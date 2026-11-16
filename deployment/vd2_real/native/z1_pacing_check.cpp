// Loopback only: real UDP datagrams test latest-reference delivery, loss,
// malformed replies and immediate PASSIVE. No B2/arm transports are opened.
#include "reply_paced_udp.h"
#include <sys/socket.h>
#include <unistd.h>
#include <stdexcept>
#include <thread>
#include <iostream>
using namespace UNITREE_ARM;
int bind_loopback(unsigned& port){
    int fd=socket(AF_INET,SOCK_DGRAM,0);sockaddr_in address{};
    address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if(fd<0 || bind(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address)))throw std::runtime_error("bind");
    socklen_t size=sizeof(address);getsockname(fd,reinterpret_cast<sockaddr*>(&address),&size);
    port=ntohs(address.sin_port);return fd;
}
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){
    try{
        unsigned peer_port,own_port;int peer=bind_loopback(peer_port),reservation=bind_loopback(own_port);
        close(reservation);
        ReplyPacedUDP udp("127.0.0.1",peer_port,own_port,sizeof(RecvState),BlockYN::NO,1000);
        SendCmd command{};command.state=ArmFSMState::LOWCMD;command.valueUnion.jointCmd[0].Pos=.1;
        auto send=[&](){return udp.send(reinterpret_cast<uint8_t*>(&command),sizeof(command));};
        SendCmd got{};auto take=[&](){return recv(peer,&got,sizeof(got),MSG_DONTWAIT);};
        require(send()==sizeof(command),"initial send missing");require(take()==sizeof(got),"loopback initial frame");
        command.valueUnion.jointCmd[0].Pos=.2;require(send()==0 && take()<0,"ungranted send queued");
        RecvState reply{};sockaddr_in own{};own.sin_family=AF_INET;own.sin_port=htons(own_port);
        own.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        auto inject=[&](size_t size){sendto(peer,&reply,size,0,reinterpret_cast<sockaddr*>(&own),sizeof(own));};
        inject(3);require(udp.recv(reinterpret_cast<uint8_t*>(&reply),sizeof(reply))==3,"short fixture");
        require(udp.received==0 && udp.replies==0 && send()==0,"short reply granted freshness/send");
        inject(sizeof(reply));require(udp.recv(reinterpret_cast<uint8_t*>(&reply),sizeof(reply))==sizeof(reply),"valid reply");
        command.valueUnion.jointCmd[0].Pos=.3;udp.source_time=123.;
        require(send()==sizeof(command) && take()==sizeof(got),"grant missing");
        require(std::abs(got.valueUnion.jointCmd[0].Pos-.3)<1e-6 && udp.sent_source_time==123.,"latest reference missing");
        const double receipt=udp.received;
        std::this_thread::sleep_for(std::chrono::milliseconds(22));
        require(send()==0 && take()<0,"active retry accumulated after controller stall");
        command.state=ArmFSMState::PASSIVE;
        require(send()==sizeof(command) && take()==sizeof(got) && got.state==ArmFSMState::PASSIVE,"PASSIVE waited for reply");
        require(send()==0,"passive bypass repeats without bound");
        std::this_thread::sleep_for(std::chrono::milliseconds(22));
        require(send()==sizeof(command) && take()==sizeof(got),"loss retry missing");
        require(udp.received==receipt,"send/loss forged feedback time");
        close(peer);std::cout<<"PASS: latest command, reply grants, short packet rejection, loss retry, immediate PASSIVE; loopback only\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
