#pragma once

#include <unitree_arm_sdk/message/arm_common.h>
#include <unitree_arm_sdk/message/udp.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>

// EXIT ONLY, after all control writers AND the SDK send/recv loop are joined.
// setFsm(PASSIVE) in this SDK waits forever for recvState.state and does not
// consult setWait(false). Do not wrap it in a detached timeout thread: that
// thread would access a destroyed arm. Use its public wire format/socket for
// a finite PASSIVE-only exchange, and report absence of a reply honestly.
struct VA15Z1ExitResult
{
    unsigned attempts=0;
    bool passive_reply=false;
};

template<class Transport>
VA15Z1ExitResult VA15RequestZ1PassiveOnExit(Transport& transport,
    const UNITREE_ARM::SendCmd& previous_command,
    const UNITREE_ARM::RecvState& previous_state)
{
    UNITREE_ARM::SendCmd request{};
    std::memset(&request,0,sizeof(request));
    // Header comes from this SDK's initialized packet, not guessed constants.
    std::memcpy(request.head,previous_command.head,sizeof(request.head));
    request.state=UNITREE_ARM::ArmFSMState::PASSIVE;
    request.track=false;
    // Every motion field remains zero. Never send a LOWCMD/position request.
    VA15Z1ExitResult result;
    transport.resetIO(UNITREE_ARM::BlockYN::NO,UNITREE_ARM::RECVSTATE_LENGTH,20000);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(250);
    for(unsigned attempt=0;attempt<10 && std::chrono::steady_clock::now()<deadline;++attempt)
    {
        ++result.attempts;
        const auto sent=transport.send(reinterpret_cast<std::uint8_t*>(&request),sizeof(request));
        UNITREE_ARM::RecvState reply{};
        const auto bytes=transport.recv(reinterpret_cast<std::uint8_t*>(&reply),sizeof(reply));
        if(sent==sizeof(request) && bytes==sizeof(reply) &&
           std::memcmp(reply.head,previous_state.head,sizeof(reply.head))==0 &&
           reply.state==UNITREE_ARM::ArmFSMState::PASSIVE)
        {
            // Protocol has no per-request sequence; this is an observed
            // PASSIVE reply, NOT a proof of physical safety or motion consent.
            result.passive_reply=true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return result;
}
