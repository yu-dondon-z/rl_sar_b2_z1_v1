// Uses the real SDK UDPPort against an ephemeral 127.0.0.1 fake controller.
// Never binds the real 8071/8072 endpoints or initializes B2/Z1 hardware.
#include "va15_z1_exit.hpp"
#include <atomic>
#include <cassert>
#include <iostream>
#include <poll.h>
#include <sys/socket.h>

int main()
{
    for(int mode=0;mode<3;++mode)
    {
        const int server=socket(AF_INET,SOCK_DGRAM,0);assert(server>=0);
        sockaddr_in address{};address.sin_family=AF_INET;
        address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);address.sin_port=0;
        assert(bind(server,reinterpret_cast<const sockaddr*>(&address),sizeof(address))==0);
        socklen_t length=sizeof(address);
        assert(getsockname(server,reinterpret_cast<sockaddr*>(&address),&length)==0);
        const auto port=ntohs(address.sin_port);assert(port!=8071 && port!=8072);
        std::atomic<bool> stop{false};
        std::atomic<unsigned> requests{0};
        std::thread fake([&]
        {
            while(!stop.load())
            {
                pollfd ready{server,POLLIN,0};if(poll(&ready,1,10)<=0)continue;
                UNITREE_ARM::SendCmd command{};sockaddr_in peer{};socklen_t n=sizeof(peer);
                const auto bytes=recvfrom(server,&command,sizeof(command),0,reinterpret_cast<sockaddr*>(&peer),&n);
                assert(bytes==sizeof(command));
                assert(command.state==UNITREE_ARM::ArmFSMState::PASSIVE && !command.track);
                ++requests;
                if(mode==0)continue; // controller is alive but never acknowledges
                UNITREE_ARM::RecvState reply{};reply.head[0]=0xfe;reply.head[1]=0xef;
                reply.state=mode==1?UNITREE_ARM::ArmFSMState::PASSIVE:UNITREE_ARM::ArmFSMState::LOWCMD;
                assert(sendto(server,&reply,sizeof(reply),0,reinterpret_cast<sockaddr*>(&peer),n)==sizeof(reply));
            }
        });
        UNITREE_ARM::SendCmd command{};command.head[0]=0xfe;command.head[1]=0xef;
        UNITREE_ARM::RecvState state{};state.head[0]=0xfe;state.head[1]=0xef;
        const auto started=std::chrono::steady_clock::now();
        {
            UNITREE_ARM::UDPPort udp("127.0.0.1",port,0,
                UNITREE_ARM::RECVSTATE_LENGTH,UNITREE_ARM::BlockYN::NO,20000);
            const auto result=VA15RequestZ1PassiveOnExit(udp,command,state);
            assert(result.passive_reply==(mode==1));
            assert(result.attempts>0 && result.attempts<=10);
        }
        assert(std::chrono::steady_clock::now()-started<std::chrono::seconds(2));
        stop.store(true);fake.join();close(server);
        assert(requests.load()>0);
    }
    std::cout<<"PASS: real SDK UDP bounded exit, 3 loopback-only scenarios; no hardware endpoints\n";
}
