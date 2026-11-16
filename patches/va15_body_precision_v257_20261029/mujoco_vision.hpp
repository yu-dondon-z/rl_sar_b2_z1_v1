#pragma once
#include "production/near_visual_io.hpp"
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

// Only an owned local subprocess and socketpair. No robot SDK or DDS.
class MuJoCoVision {
    int socket_=-1;
    pid_t child_=-1;
    void Transfer(void* bytes,std::size_t size,bool writing) {
        auto* p=static_cast<char*>(bytes);
        while(size) {
            const auto count=writing?send(socket_,p,size,MSG_NOSIGNAL):recv(socket_,p,size,0);
            if(count<0&&errno==EINTR)continue;
            if(count<=0)throw std::runtime_error("MuJoCo vision worker disconnected; see its log");
            p+=count;size-=count;
        }
    }
    std::string Receive(std::vector<unsigned char>& overlay) {
        std::uint64_t sizes[2]{};Transfer(sizes,sizeof(sizes),false);
        if(sizes[0]>1000000||sizes[1]>640*360*3)throw std::runtime_error("Invalid MuJoCo vision response");
        std::string json(sizes[0],'\0');Transfer(json.data(),json.size(),false);
        overlay.resize(sizes[1]);Transfer(overlay.data(),overlay.size(),false);
        return json;
    }
public:
    double last_latency_ms=0.,max_latency_ms=0.;
    std::vector<unsigned char> overlay;
    MuJoCoVision()=default;
    MuJoCoVision(const MuJoCoVision&)=delete;
    MuJoCoVision& operator=(const MuJoCoVision&)=delete;
    ~MuJoCoVision() {
        if(socket_>=0){shutdown(socket_,SHUT_RDWR);close(socket_);}
        if(child_>0) {
            int status=0;
            for(int i=0;i<100;++i) {
                if(waitpid(child_,&status,WNOHANG)==child_)return;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            kill(child_,SIGTERM);waitpid(child_,&status,0);
        }
    }
    void Start(const std::filesystem::path& script,const std::string& output) {
        int sockets[2];if(socketpair(AF_UNIX,SOCK_STREAM,0,sockets))throw std::runtime_error("vision socketpair failed");
        child_=fork();
        if(child_<0){close(sockets[0]);close(sockets[1]);throw std::runtime_error("vision fork failed");}
        if(child_==0) {
            close(sockets[0]);const auto fd=std::to_string(sockets[1]);
            const char* python=std::getenv("RL_SAR_SIM_PYTHON");
            if(!python)python="/home/dong/anaconda3/bin/python";
            execlp(python,python,"-u",script.c_str(),"--fd",fd.c_str(),"--output",output.c_str(),static_cast<char*>(nullptr));
            _exit(127);
        }
        close(sockets[1]);socket_=sockets[0];
        timeval timeout{120,0};setsockopt(socket_,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
        const auto response=YAML::Load(Receive(overlay));
        if(!response["ready"].as<bool>(false))throw std::runtime_error("MuJoCo vision worker not ready");
        std::cout<<"MuJoCo front camera -> "<<response["network"].as<std::string>()<<" -> production LongTracker ready\n";
    }
    va15_visual::Packet Predict(std::vector<unsigned char>& rgb,std::uint64_t sequence,
                                std::uint64_t capture,unsigned phase) {
        std::uint64_t header[5]{640,360,sequence,capture,phase};
        Transfer(header,sizeof(header),true);Transfer(rgb.data(),rgb.size(),true);
        const auto json=Receive(overlay);
        auto packet=va15_visual::Parse(json,std::string(32,'c'));
        if(packet.sequence!=sequence||packet.capture_ns!=capture||!packet.image_received)
            throw std::runtime_error("MuJoCo camera packet failed production parser or frame identity");
        return packet;
    }
};
