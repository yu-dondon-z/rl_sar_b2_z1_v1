// TEST ONLY. Actual hashed SDK, kernel-denied networking/thread creation.
// FSM replies are EXPLICIT SYNTHETIC MEMORY ACKS, NOT hardware feedback.
// No UDPPort construction, sendRecv(), SDK thread start or model dynamics.
// Never link the nanosleep interposition below into production.
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <linux/audit.h>
#include <linux/filter.h>
#include <linux/seccomp.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <cassert>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>

static UNITREE_ARM::CtrlComponents* memory_peer=nullptr;
static unsigned synthetic_replies=0;

extern "C" int nanosleep(const timespec*,timespec*) {
    if(!memory_peer)_exit(90); // any unexpected sleep fails, never communicates
    // SDK's blocking FSM loop normally waits for its network thread. Here
    // only its memory state is changed, single-threaded, after it requests it.
    memory_peer->recvState.state=memory_peer->sendCmd.state;
    ++synthetic_replies;
    return 0;
}

static void DenyNetworkAndThreads() {
#define DENY_SYSCALL(number) BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,number,0,1), BPF_STMT(BPF_RET|BPF_K,SECCOMP_RET_KILL_PROCESS)
    const sock_filter filter[]={
        BPF_STMT(BPF_LD|BPF_W|BPF_ABS,offsetof(seccomp_data,arch)),
        BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,AUDIT_ARCH_X86_64,1,0),
        BPF_STMT(BPF_RET|BPF_K,SECCOMP_RET_KILL_PROCESS),
        BPF_STMT(BPF_LD|BPF_W|BPF_ABS,offsetof(seccomp_data,nr)),
        BPF_JUMP(BPF_JMP|BPF_JSET|BPF_K,0x40000000,0,1),
        BPF_STMT(BPF_RET|BPF_K,SECCOMP_RET_KILL_PROCESS),
        DENY_SYSCALL(__NR_socket),DENY_SYSCALL(__NR_socketpair),
        DENY_SYSCALL(__NR_connect),DENY_SYSCALL(__NR_bind),
        DENY_SYSCALL(__NR_listen),DENY_SYSCALL(__NR_accept),DENY_SYSCALL(__NR_accept4),
        DENY_SYSCALL(__NR_sendto),DENY_SYSCALL(__NR_recvfrom),
        DENY_SYSCALL(__NR_sendmsg),DENY_SYSCALL(__NR_recvmsg),
        DENY_SYSCALL(__NR_sendmmsg),DENY_SYSCALL(__NR_recvmmsg),
        DENY_SYSCALL(__NR_shutdown),DENY_SYSCALL(__NR_setsockopt),DENY_SYSCALL(__NR_getsockopt),
        DENY_SYSCALL(__NR_clone),DENY_SYSCALL(__NR_clone3),
        DENY_SYSCALL(__NR_fork),DENY_SYSCALL(__NR_vfork),
        BPF_STMT(BPF_RET|BPF_K,SECCOMP_RET_ALLOW)
    };
#undef DENY_SYSCALL
    sock_fprog program{static_cast<unsigned short>(sizeof(filter)/sizeof(filter[0])),
                       const_cast<sock_filter*>(filter)};
    if(prctl(PR_SET_NO_NEW_PRIVS,1,0,0,0)!=0||prctl(PR_SET_SECCOMP,SECCOMP_MODE_FILTER,&program)!=0)
        _exit(91);
    assert(prctl(PR_GET_SECCOMP)==SECCOMP_MODE_FILTER);
}

static void Equal6(const std::vector<double>& values,const std::array<double,6>& expected) {
    assert(values.size()>=6);
    for(unsigned i=0;i<6;++i)assert(std::isfinite(values[i])&&std::fabs(values[i]-expected[i])<1.e-12);
}

int main(int argc,char** argv) {
    // Guard installed BEFORE any SDK object exists. Negative probes are
    // separate processes, expected SIGSYS; they cannot open a socket/thread.
    const rlimit no_core{0,0};
    if(setrlimit(RLIMIT_CORE,&no_core)!=0)return 94;
    DenyNetworkAndThreads();
    if(argc==2&&std::strcmp(argv[1],"--negative-network")==0) {
        (void)socket(AF_INET,SOCK_DGRAM,0);return 92;
    }
    if(argc==2&&std::strcmp(argv[1],"--negative-process")==0) {
        (void)syscall(__NR_fork);return 93;
    }
    assert(argc==1);
    auto* ctrl=new UNITREE_ARM::CtrlComponents();
    ctrl->dt=.002;ctrl->udp=nullptr;ctrl->armModel=nullptr;
    std::memset(&ctrl->recvState,0,sizeof(ctrl->recvState));
    ctrl->recvState.state=UNITREE_ARM::ArmFSMState::PASSIVE;
    // unitreeArm owns CtrlComponents on destruction; do not delete it twice.
    auto arm=std::make_unique<UNITREE_ARM::unitreeArm>(ctrl);
    memory_peer=ctrl;
    const std::array<double,6> defaults{20,30,30,20,15,10},kd{2000,2000,2000,2000,2000,2000};
    const std::vector<double> scaled{7,10.5,10.5,7,5.25,3.5};
    const Vec6 q=(Vec6()<<.01,.02,.03,.04,.05,.06).finished();
    for(unsigned cycle=0;cycle<3;++cycle) {
        assert(ctrl->recvState.state==UNITREE_ARM::ArmFSMState::PASSIVE);
        arm->setArmCmd(q,Vec6::Zero(),Vec6::Zero());
        auto gains=ctrl->lowcmd->kp,derivative=ctrl->lowcmd->kd;
        for(unsigned i=0;i<6;++i)gains[i]=scaled[i];
        ctrl->lowcmd->setControlGain(gains,derivative);
        assert(arm->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD));
        Equal6(ctrl->lowcmd->kp,defaults);Equal6(ctrl->lowcmd->kd,kd);
        assert((ctrl->lowcmd->getQ()-q).norm()<1.e-12);

        ctrl->lowcmd->setPassive();
        Equal6(ctrl->lowcmd->kp,{0,0,0,0,0,0});
        Equal6(ctrl->lowcmd->kd,{10,10,10,10,10,10});
        // A passive COMMAND BUFFER does not request a PASSIVE FSM.
        assert(ctrl->recvState.state==UNITREE_ARM::ArmFSMState::LOWCMD);
        assert(ctrl->sendCmd.state==UNITREE_ARM::ArmFSMState::LOWCMD);
        assert(!arm->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD));
        assert(ctrl->sendCmd.state==UNITREE_ARM::ArmFSMState::PASSIVE);
        assert(ctrl->recvState.state==UNITREE_ARM::ArmFSMState::LOWCMD);
        // Explicit handshake repairs the software precondition; ACK is still
        // synthetic memory, not a test of physical arm restart safety.
        assert(arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE));
        assert(ctrl->recvState.state==UNITREE_ARM::ArmFSMState::PASSIVE);
    }
    assert(synthetic_replies==6);
    arm.reset();memory_peer=nullptr;
    std::cout<<"PASS actual audited SDK: gain overwrite, passive-buffer/FSM separation, 3 explicit FSM cycles; synthetic memory ACKs="
             <<synthetic_replies<<"; seccomp network/process denial; NO hardware qualification\n";
}
