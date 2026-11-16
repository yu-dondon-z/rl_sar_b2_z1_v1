#pragma once
#include <array>
#include <cmath>
#include <cstdint>

// Private, explicitly bounded experiment. NOT a firmware certification.
// Unitree B2 Basic Service Interface and Fault Service Interface document
// reserve[0] as current motor faults, reserve[1] as communication frequency,
// and bit_flag bit 0 as command timeout. No cumulative/reset semantics for
// lost are assumed. Original raw values are retained. Before any entry is
// allowed, this process establishes a baseline from THREE continuous seconds
// of unchanged counts AND documented healthy current fields. That baseline
// is then immutable for the process; any change latches refusal (even a drop).
namespace va15_motor_contract
{
inline constexpr double WarmupSeconds=3., MaxSourceAge=.05;
// Experimental engineering margins for the observed 533..540 Hz, NOT
// thresholds stated or certified by Unitree.
inline constexpr unsigned MinFrequency=400, MaxFrequency=800;
enum class Reason { NoData, Warmup, Ready, ReceiveTime, Sequence, Stale,
    ControllerTimeout, ComponentFault, Charging, LostChanged, MotorMode,
    MotorFault, MotorFrequency, StartupCountsChanged };
inline const char* Text(Reason r)
{
    switch(r)
    {
    case Reason::NoData:return "no raw feedback";
    case Reason::Warmup:return "startup: collecting 3 s stable counts and current-health checks; A/Y disabled";
    case Reason::Ready:return "READY: this process baseline frozen; current health valid; not hardware qualified";
    case Reason::ReceiveTime:return "receive clock invalid/reversed";
    case Reason::Sequence:return "source tick/gap/receive-time disagreement";
    case Reason::Stale:return "raw feedback older than 50 ms";
    case Reason::ControllerTimeout:return "bit_flag 0x01: motion command timeout; NOT ignored";
    case Reason::ComponentFault:return "bit_flag component fault/emergency";
    case Reason::Charging:return "charging state; no motion permission";
    case Reason::LostChanged:return "raw lost changed AFTER baseline freeze; latched, NOT re-based";
    case Reason::StartupCountsChanged:return "startup counts changed; stability window restarted, A/Y still disabled";
    case Reason::MotorMode:return "motor mode is not 10";
    case Reason::MotorFault:return "motor reserve[0] reports a fault";
    case Reason::MotorFrequency:return "motor reserve[1] outside experimental 400..800 Hz";
    }
    return "unknown";
}
struct Raw
{
    std::uint32_t tick=0;
    unsigned bit_flag=0;
    std::array<unsigned,12> mode{},lost{},fault{},frequency{};
};
struct Status
{
    bool ready=false,latched=false;
    Reason reason=Reason::NoData;
    int joint=-1;
    double stable_seconds=0.;
    bool baseline_frozen=false;
    std::array<unsigned,12> baseline{};
    unsigned startup_changes=0;
};
class Guard
{
    Status status_;
    bool have_=false,ever_ready_=false,candidate_valid_=false;
    std::uint32_t tick_=0;
    double received_=0.;
    void Reject(Reason reason,int joint=-1)
    {
        status_.ready=false;status_.stable_seconds=0.;
        status_.latched=ever_ready_;
        status_.reason=reason;status_.joint=joint;
        if(!ever_ready_)candidate_valid_=false;
    }
public:
    // No reset/rebase method: once ready, a detected fault cannot be re-armed
    // by holding a stick or pressing Y, nor by changing back to old values.
    void Update(const Raw& r,double now)
    {
        if(status_.latched)return;
        if(!std::isfinite(now)||now<0.||(have_&&now<received_))
        {Reject(Reason::ReceiveTime);return;}
        if(have_&&r.tick==tick_)return; // a duplicate never refreshes freshness
        const auto delta=static_cast<std::uint32_t>(r.tick-tick_);
        const double dt=have_?now-received_:0.;
        const bool continuous=have_&&delta>0&&delta<=50&&dt>0.&&dt<=MaxSourceAge&&
            std::fabs(delta*.001-dt)<=.035;
        const bool previous=have_;
        have_=true;tick_=r.tick;received_=now;
        if(previous&&!continuous){Reject(Reason::Sequence);return;}
        if(r.bit_flag>255||(r.bit_flag&0xfau)){Reject(Reason::ComponentFault);return;}
        if(r.bit_flag&0x01u){Reject(Reason::ControllerTimeout);return;}
        if(!(r.bit_flag&0x04u)){Reject(Reason::Charging);return;}
        for(int i=0;i<12;++i)
        {
            if(r.mode[i]!=10){Reject(Reason::MotorMode,i);return;}
            if(r.fault[i]!=0){Reject(Reason::MotorFault,i);return;}
            if(r.frequency[i]<MinFrequency||r.frequency[i]>MaxFrequency)
            {Reject(Reason::MotorFrequency,i);return;}
        }
        // Prior runs' absolute values are not the current process baseline.
        // This only applies BEFORE the initial READY; no control entry is
        // permitted until the new stable-health window is complete.
        if(!candidate_valid_)
        {
            status_.baseline=r.lost;candidate_valid_=true;
            status_.stable_seconds=0.;status_.reason=Reason::Warmup;
            status_.joint=-1;return;
        }
        for(int i=0;i<12;++i)if(r.lost[i]!=status_.baseline[i])
        {
            if(ever_ready_){Reject(Reason::LostChanged,i);return;}
            status_.baseline=r.lost;status_.stable_seconds=0.;
            ++status_.startup_changes;status_.reason=Reason::StartupCountsChanged;
            status_.joint=i;return;
        }
        status_.joint=-1;
        status_.stable_seconds+=continuous?dt:0.;
        status_.ready=status_.stable_seconds+1.e-9>=WarmupSeconds;
        status_.reason=status_.ready?Reason::Ready:Reason::Warmup;
        ever_ready_=ever_ready_||status_.ready;
        status_.baseline_frozen=ever_ready_;
    }
    Status Snapshot(double now)const
    {
        auto r=status_;
        if(!r.latched&&(!have_||!std::isfinite(now)||now<received_||now-received_>MaxSourceAge))
        {
            r.ready=false;r.stable_seconds=0.;r.latched=ever_ready_;
            r.reason=have_?Reason::Stale:Reason::NoData;
        }
        return r;
    }
};
}
