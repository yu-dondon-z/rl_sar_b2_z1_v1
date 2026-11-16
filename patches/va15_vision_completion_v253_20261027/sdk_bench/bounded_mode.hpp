#pragma once
// SOFTWARE PROTOTYPE, not connected to the deployment controller or UDP.
// One owner must apply an entire decision + command frame and perform receive.
// No SDK blocking setFsm(), threads, sleeps, fabricated ACKs or gain tuning.
#include <unitree_arm_sdk/message/arm_common.h>
#include <array>
#include <cmath>
#include <cstdint>

namespace va15_z1_bench {
using Mode=UNITREE_ARM::ArmFSMState;
struct Feedback {
    bool valid=false,healthy=false;
    std::uint64_t generation=0,received_ns=0;
    Mode mode=Mode::INVALID;
    std::array<double,6> q{};
};
struct Decision {
    Mode request=Mode::PASSIVE;
    bool passive_buffer=true,initialize_sdk_defaults=false,seed_measured_q=false;
    bool permit_policy_frame=false,activation_edge=false;
    std::array<double,6> seed{};
};
class BoundedMode {
public:
    enum class Stage {WaitPassive,Passive,WaitLowCmd,Active,Fault};
    static constexpr bool production_integrated=false,hardware_qualified=false;
    static constexpr std::uint64_t BudgetNs=500000000,FreshNs=100000000;
    Stage stage=Stage::WaitPassive;
    const char* reason="requesting fresh PASSIVE reply";
    Decision Tick(std::uint64_t now,bool want_active,bool stop_or_shutdown,const Feedback& f) {
        // All time is supplied by one owner. Bad/regressing clocks or receive
        // generations latch a fault, never an automatic reset/re-arm.
        if((started&&now<last_now)||f.generation<last_generation)return Fault("clock/receive generation regressed");
        last_now=now;last_generation=f.generation;
        if(!started){started=true;Begin(Stage::WaitPassive,now,f.generation);}
        if(stage==Stage::Fault)return {};
        const bool active=want_active&&!stop_or_shutdown;
        const bool finite=[](const auto& q){for(double v:q)if(!std::isfinite(v))return false;return true;}(f.q);
        const bool fresh=f.valid&&f.healthy&&finite&&f.generation&&f.received_ns<=now&&now-f.received_ns<=FreshNs;
        if(!active&&(stage==Stage::Active||stage==Stage::WaitLowCmd))
            Begin(Stage::WaitPassive,now,f.generation); // stop always preempts arming
        if(stage==Stage::WaitPassive||stage==Stage::WaitLowCmd) {
            if(now-phase_ns>=BudgetNs)return Fault("bounded FSM handshake timeout; passive requests only");
            const Mode expected=stage==Stage::WaitPassive?Mode::PASSIVE:Mode::LOWCMD;
            const bool reply=fresh&&f.generation>phase_generation&&f.received_ns>=phase_ns&&f.mode==expected;
            if(reply) {
                if(stage==Stage::WaitPassive) {
                    stage=Stage::Passive;reason="fresh PASSIVE reply; software precondition only";
                    return {};
                }
                stage=Stage::Active;reason="fresh LOWCMD reply; seed frame first";
                Decision d=Low();d.activation_edge=true;return d;
            }
            return stage==Stage::WaitLowCmd?Low():Decision{};
        }
        if(stage==Stage::Passive) {
            if(!active)return {};
            if(!fresh||f.mode!=Mode::PASSIVE){reason="waiting for fresh healthy PASSIVE feedback";return {};}
            seed=f.q;Begin(Stage::WaitLowCmd,now,f.generation);
            Decision d=Low();d.initialize_sdk_defaults=true;return d;
        }
        if(!fresh||f.mode!=Mode::LOWCMD)return Fault("active receive/source/mode invalid; no automatic re-arm");
        Decision d=Low();d.seed_measured_q=false;d.permit_policy_frame=true;return d;
    }
private:
    bool started=false;
    std::uint64_t last_now=0,last_generation=0,phase_ns=0,phase_generation=0;
    std::array<double,6> seed{};
    void Begin(Stage next,std::uint64_t now,std::uint64_t generation) {
        stage=next;phase_ns=now;phase_generation=generation;
        reason=next==Stage::WaitPassive?"waiting bounded PASSIVE handshake":"waiting bounded LOWCMD handshake";
    }
    Decision Low() const {
        Decision d;d.request=Mode::LOWCMD;d.passive_buffer=false;d.seed_measured_q=true;d.seed=seed;return d;
    }
    Decision Fault(const char* why){stage=Stage::Fault;reason=why;return {};}
};
}
