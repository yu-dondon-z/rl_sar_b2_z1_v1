// Hardware transport only. No policy, vision, world pose, or simulated constraints.
#include "io.h"
#include "reply_paced_udp.h"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/b2/front_video/front_video_client.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree_arm_sdk/control/ctrlComponents.h>
#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using namespace unitree::robot;
using namespace UNITREE_ARM;
using Clock=std::chrono::steady_clock;
double now(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
// Actor order -> DDS motor slot; -1 entries are Z1 joint1..6.
constexpr int map[18]={3,0,9,6,4,1,10,7,-1,5,2,11,8,-1,-1,-1,-1,-1};
constexpr int arm[6]={8,13,14,15,16,17};
// Preserve existing SDK RAW gain units, do not equate them with simulated Nm/rad.
const std::vector<double> arm_kp={20,30,30,20,15,10}, arm_kd(6,2000);
constexpr double lower[18]={-.87,-.87,-.87,-.87,-.94,-.94,-.94,-.94,-2.6179939,-2.82,-2.82,-2.82,-2.82,0,-2.8797933,-1.5184364,-1.3439035,-2.7925268};
constexpr double upper[18]={.87,.87,.87,.87,4.69,4.69,4.69,4.69,2.6179939,-.43,-.43,-.43,-.43,2.9670597,0,1.5184364,1.3439035,2.7925268};
std::string error;
std::mutex mutex;
VD2State state{};
VD2Command target{};
std::atomic<bool> running{false}, owned{false};
std::atomic<double> last_command{0};
std::thread writer;
ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> publisher;
ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> subscriber;
ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> remote_subscriber;
double remote_time=0;
unsigned remote_keys=0;
std::unique_ptr<CtrlComponents> z1;
std::unique_ptr<unitree::robot::b2::FrontVideoClient> front;

std::uint32_t crc(const void* data, size_t words){
    std::uint32_t result=0xffffffff;
    const auto* bytes=static_cast<const unsigned char*>(data);
    for(size_t i=0;i<words;++i){
        std::uint32_t word;std::memcpy(&word,bytes+4*i,4);
        for(std::uint32_t bit=0x80000000;bit;bit>>=1){
            const bool high=result&0x80000000;result<<=1;
            if(high) result^=0x04c11db7;
            if(word&bit) result^=0x04c11db7;
        }
    }
    return result;
}
bool valid(const VD2Command& command){
    if(command.mode>1 || !std::isfinite(command.policy_blend) || command.policy_blend<0 || command.policy_blend>1) return false;
    for(int i=0;i<18;++i){
        if(!std::isfinite(command.q[i]) || (command.mode &&
            (command.q[i]<lower[i] || command.q[i]>upper[i]))) return false;
        const double max_kp=i>=9 && i<=12 ? 675 : 450;
        if(!std::isfinite(command.leg_kp[i]) || !std::isfinite(command.leg_kd[i]) ||
           command.leg_kp[i]<0 || command.leg_kp[i]>max_kp ||
           command.leg_kd[i]<0 || command.leg_kd[i]>15) return false;
        if(map[i]<0 && (command.leg_kp[i]!=0 || command.leg_kd[i]!=0)) return false;
    }
    return true;
}

unitree_go::msg::dds_::LowCmd_ b2_packet(const VD2Command& command,bool active){
    unitree_go::msg::dds_::LowCmd_ output{};
    output.head()[0]=0xfe;output.head()[1]=0xef;output.level_flag()=0xff;
    for(auto& motor:output.motor_cmd()){
        motor.mode()=0x0a;motor.q()=2.146e9f;motor.dq()=16000.f;
        motor.kp()=0;motor.kd()=0;motor.tau()=0;
    }
    for(int i=0;i<18;++i) if(map[i]>=0){
        auto& motor=output.motor_cmd()[map[i]];motor.dq()=0;
        motor.kd()=active ? command.leg_kd[i] : 3;
        if(active){motor.q()=command.q[i];motor.kp()=command.leg_kp[i];}
    }
    output.crc()=crc(&output,(sizeof(output)>>2)-1);
    return output;
}

void callback(const void* message){
    const auto& data=*static_cast<const unitree_go::msg::dds_::LowState_*>(message);
    if(crc(&data,(sizeof(data)>>2)-1)!=data.crc()) return;
    std::lock_guard<std::mutex> guard(mutex);
    // Duplicated cached DDS packets must not refresh the sensor clock.
    if(state.b2_time && data.tick()==state.tick) return;
    state.b2_time=now();state.tick=data.tick();state.b2_flags=data.bit_flag();
    const auto& radio=data.wireless_remote();
    const bool embedded_valid=radio[0]==0xfe && radio[1]==0xef;
    state.keys=embedded_valid ? static_cast<unsigned>(radio[2]) | (static_cast<unsigned>(radio[3])<<8) :
               now()-remote_time<.5 ? remote_keys : 0;
    if((state.keys&((1u<<1)|(1u<<10)))==((1u<<1)|(1u<<10))) state.fault=5;
    for(int i=0;i<18;++i) if(map[i]>=0){
        const auto& m=data.motor_state()[map[i]];
        state.q[i]=m.q();state.dq[i]=m.dq();state.tau[i]=m.tau_est();
    }
    for(int i=0;i<12;++i){
        const auto& m=data.motor_state()[i];
        state.motor_fault[i]=m.reserve()[0];state.motor_lost[i]=m.lost();
        if(owned && (m.reserve()[0] || !std::isfinite(m.q()) || !std::isfinite(m.dq()))) state.fault=6;
    }
    for(int i=0;i<4;++i){state.quaternion[i]=data.imu_state().quaternion()[i];state.foot_force[i]=data.foot_force()[i];}
    for(int i=0;i<3;++i)state.omega[i]=data.imu_state().gyroscope()[i];
}

void remote_callback(const void* message){
    const auto& data=*static_cast<const unitree_go::msg::dds_::WirelessController_*>(message);
    std::lock_guard<std::mutex> guard(mutex);
    remote_time=now();remote_keys=data.keys();
    if((remote_keys&((1u<<1)|(1u<<10)))==((1u<<1)|(1u<<10)))state.fault=5;
}

void io_loop(){
    auto deadline=Clock::now();
    auto* udp=static_cast<ReplyPacedUDP*>(z1->udp);
    while(running){
        deadline+=std::chrono::milliseconds(2);
        VD2State measured;VD2Command command;double source_time;
        {std::lock_guard<std::mutex> guard(mutex);measured=state;command=target;source_time=last_command.load();}
        const double time=now();
        if(owned && !measured.fault){
            unsigned fault=0;
            if(time-last_command.load()>.15)fault=1;
            if(!measured.b2_time || time-measured.b2_time>.10)fault=2;
            if(!measured.z1_time || time-measured.z1_time>.10)fault=3;
            if(fault){std::lock_guard<std::mutex> guard(mutex);state.fault=fault;measured.fault=fault;}
        }
        const bool active=owned && command.mode==1 && !measured.fault;
        if(owned){
            auto output=b2_packet(command,active);
            publisher->Write(output);
        }
        // This thread is the sole Z1 send/receive and gain owner.
        if(active){
            z1->lowcmd->setControlGain(arm_kp,arm_kd);
            for(int i=0;i<6;++i){z1->lowcmd->q[i]=command.q[arm[i]];z1->lowcmd->dq[i]=0;z1->lowcmd->tau[i]=0;}
            z1->sendCmd.state=ArmFSMState::LOWCMD;
        }else{
            z1->lowcmd->setPassive();z1->sendCmd.state=ArmFSMState::PASSIVE;
        }
        udp->source_time=source_time;
        z1->sendRecv(); // No setFsm() here: that can overwrite gains.
        {
            std::lock_guard<std::mutex> guard(mutex);
            if(udp->received>state.z1_time){
                state.z1_time=udp->received;state.z1_mode=static_cast<int>(z1->recvState.state);
                for(int i=0;i<6;++i){
                    state.q[arm[i]]=z1->lowstate->q[i];state.dq[arm[i]]=z1->lowstate->dq[i];state.tau[arm[i]]=z1->lowstate->tau[i];
                    state.z1_error[i]=z1->lowstate->errorstate[i];state.z1_connected[i]=z1->lowstate->isMotorConnected[i];
                    if(owned && ((state.z1_error[i]&0x3f) || state.z1_connected[i] ||
                        !std::isfinite(state.q[arm[i]]) || !std::isfinite(state.dq[arm[i]])))state.fault=7;
                }
            }
            // A requested LOWCMD that never appears is a real actuator fault.
            static double mode_since=0;
            if(active && state.z1_mode!=static_cast<int>(ArmFSMState::LOWCMD)){
                if(!mode_since)mode_since=time;
                if(time-mode_since>.5)state.fault=4;
            }else mode_since=0;
            state.command_time=last_command.load();
            state.z1_sent_time=udp->sent_time;state.z1_sent_source_time=udp->sent_source_time;
            std::copy(udp->sent_q,udp->sent_q+6,state.z1_sent_q);
            state.z1_sends=udp->sends;state.z1_suppressed=udp->suppressed;state.z1_replies=udp->replies;
        }
        std::this_thread::sleep_until(deadline);
        if(Clock::now()>deadline+std::chrono::milliseconds(20)) deadline=Clock::now();
    }
}
template<class Function>int attempt(Function function){
    try{function();return 0;}catch(const std::exception& e){error=e.what();return -1;}
}
}

extern "C" {
const char* vd2_error(){return error.c_str();}
unsigned long vd2_state_size(){return sizeof(VD2State);}
unsigned long vd2_command_size(){return sizeof(VD2Command);}
int vd2_open(const char* interface_name,const char* ip,int to_port,int own_port){
    return attempt([&]{
        if(running)throw std::runtime_error("IO already open");
        ChannelFactory::Instance()->Init(0,interface_name);
        subscriber=std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>("rt/lowstate");
        subscriber->InitChannel(callback,1);
        remote_subscriber=std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>>("rt/wirelesscontroller");
        remote_subscriber->InitChannel(remote_callback,1);
        z1=std::make_unique<CtrlComponents>();z1->dt=.002;z1->armModel=nullptr;
        z1->recvState={};z1->sendCmd.track=false;
        z1->udp=new ReplyPacedUDP(ip,to_port,own_port,sizeof(RecvState),BlockYN::NO,1000);
        running=true;writer=std::thread(io_loop);
    });
}
int vd2_snapshot(VD2State* output){
    if(!output)return -1;
    std::lock_guard<std::mutex> guard(mutex);*output=state;return 0;
}
int vd2_acquire(){
    return attempt([]{
        VD2State measured;vd2_snapshot(&measured);
        if(!running || owned || measured.fault || now()-measured.b2_time>.10 || now()-measured.z1_time>.10)
            throw std::runtime_error("fresh B2/Z1 state and idle ownership required");
        for(auto fault:measured.motor_fault) if(fault)throw std::runtime_error("B2 motor fault");
        for(int i=0;i<6;++i)if((measured.z1_error[i]&0x3f)||measured.z1_connected[i])throw std::runtime_error("Z1 motor fault");
        publisher=std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>("rt/lowcmd");publisher->InitChannel();
        unitree::robot::b2::MotionSwitcherClient switcher;switcher.SetTimeout(2.0f);switcher.Init();
        std::string form,name;
        if(switcher.CheckMode(form,name)!=0)throw std::runtime_error("cannot read B2 motion service");
        if(!name.empty() && switcher.ReleaseMode()!=0)throw std::runtime_error("B2 ReleaseMode failed");
        const auto end=Clock::now()+std::chrono::seconds(3);
        while(Clock::now()<end){
            if(switcher.CheckMode(form,name)==0 && name.empty())break;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if(!name.empty())throw std::runtime_error("B2 motion service still owns control");
        // RPCs can take time. The first captured hold uses the latest sensor
        // sample, rather than the pre-RPC joint angles.
        vd2_snapshot(&measured);
        {std::lock_guard<std::mutex> guard(mutex);
            std::copy(measured.q,measured.q+18,target.q);target.policy_blend=0;target.mode=1;
            for(int i=0;i<18;++i){
                target.leg_kp[i]=map[i]<0 ? 0 : i>=9 && i<=12 ? 300 : 200;
                target.leg_kd[i]=map[i]<0 ? 0 : 3;
            }
        }
        last_command=now();owned=true;
    });
}
int vd2_command(const VD2Command* command){
    if(!command || !valid(*command)){error="nonfinite/out-of-limit actuator target";return -1;}
    std::lock_guard<std::mutex> guard(mutex);target=*command;last_command=now();return 0;
}
void vd2_close(){
    if(owned){VD2Command damping{};vd2_command(&damping);std::this_thread::sleep_for(std::chrono::milliseconds(100));}
    running=false;if(writer.joinable())writer.join();
    if(subscriber)subscriber->CloseChannel();
    if(remote_subscriber)remote_subscriber->CloseChannel();
    publisher.reset();subscriber.reset();remote_subscriber.reset();z1.reset();owned=false;
}
int vd2_front_open(const char* interface_name){
    return attempt([&]{ChannelFactory::Instance()->Init(0,interface_name);front=std::make_unique<unitree::robot::b2::FrontVideoClient>();front->SetTimeout(1.0f);front->Init();});
}
int vd2_front_image(unsigned char* bytes,int capacity,double* requested,double* received){
    if(!front)return -1;
    std::vector<std::uint8_t> sample;
    *requested=now();const int result=front->GetImageSample(sample);*received=now();
    if(result || sample.empty() || sample.size()>static_cast<size_t>(capacity))return -1;
    std::memcpy(bytes,sample.data(),sample.size());return static_cast<int>(sample.size());
}
int vd2_offline_check(){
    VD2Command command{};
    const double defaults[18]={.1,-.1,.1,-.1,.8,.8,1,1,0,-1.5,-1.5,-1.5,-1.5,2.55,-1.6,-1.4,0,0};
    std::copy(defaults,defaults+18,command.q);command.mode=1;
    if(!valid(command))return 1;
    command.q[13]=-1;if(valid(command))return 2;command.q[13]=defaults[13];
    bool slots[12]={};for(int m:map)if(m>=0){if(m>=12 || slots[m])return 3;slots[m]=true;}
    for(bool slot:slots)if(!slot)return 4;
    const std::uint32_t zero=0;if(crc(&zero,1)!=0xc704dd7b)return 5;
    // Exercise the EXACT packet builder used by the 500 Hz writer, including
    // damping fallback and all 12 policy-to-DDS routes. No transports opened.
    for(int i=0;i<18;++i)if(map[i]>=0){
        command.leg_kp[i]=i>=9 && i<=12 ? 675 : 450;command.leg_kd[i]=15;
    }
    const auto packet=b2_packet(command,true),damping=b2_packet(command,false);
    for(int i=0;i<18;++i)if(map[i]>=0){
        const auto& m=packet.motor_cmd()[map[i]];
        if(std::abs(m.q()-command.q[i])>1.e-6 || m.kp()!=command.leg_kp[i] ||
           m.kd()!=command.leg_kd[i] || m.dq()!=0 || m.tau()!=0 || m.mode()!=0x0a)return 6;
        if(damping.motor_cmd()[map[i]].kp()!=0 || damping.motor_cmd()[map[i]].kd()!=3)return 7;
    }
    if(packet.crc()!=crc(&packet,(sizeof(packet)>>2)-1))return 8;
    return 0;
}
}
