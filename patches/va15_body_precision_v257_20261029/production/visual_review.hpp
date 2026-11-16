#pragma once
#include "near_visual_guide.hpp"
#include <atomic>
#include <cstring>

namespace va15_visual {
// Writer-owned measurements, never a motor acknowledgement. Used only to
// refuse approval of the stationary, zero-output review under camera load.
class WriteAudit {
    std::atomic<bool> active_{false},failed_{false};
    std::atomic<unsigned> samples_{0};
public:
    void Begin(){active_.store(false);failed_.store(false);samples_.store(0);active_.store(true);}
    void Notify(std::uint64_t end,std::uint64_t interval,std::uint64_t source,bool write_ok) {
        if(!active_.load())return;
        if(!write_ok||interval>=20000000ULL||source==0||source>end||end-source>=50000000ULL)failed_.store(true);
        samples_.fetch_add(1);
    }
    bool Healthy()const{return !failed_.load();}
    unsigned Samples()const{return samples_.load();}
    bool End(){active_.store(false);return Healthy()&&Samples()>=100;}
};

class Review {
public:
    Guide guide;
    WriteAudit audit;
    Output would;
    bool approved=false;
    std::uint64_t approved_ns=0;
    double approved_bearing=0.;
    const char* reason="first T: four-second zero-output camera-loaded review";
    bool Start(const Guide& current,std::uint64_t now,bool quiet,bool centered,bool healthy) {
        if(approved||guide.stage==Guide::Stage::Running)return false;
        guide=current;would={};
        if(!guide.Start(now,quiet,centered,healthy)){reason=guide.reason;return false;}
        audit.Begin();reason="ZERO OUTPUT REVIEW RUNNING; no walking";return true;
    }
    void Observe(const Packet& p,std::uint64_t now){guide.Observe(p,now);}
    void Cancel(const char* why){guide.Stop(why);audit.End();approved=false;would={};reason=why;}
    void Tick(const Guide& current,std::uint64_t now,double dt,bool quiet,bool healthy,bool stop,bool centered) {
        if(approved) {
            if(!quiet||!healthy||stop||!centered||!current.Ready(now)||now-approved_ns>30000000000ULL||
               std::fabs(SteeringBearing(current.latest)-approved_bearing)>.035) {
                approved=false;reason="review expired/inputs changed; T explicitly reviews again";
            }
            return;
        }
        if(guide.stage!=Guide::Stage::Running){would={};return;}
        would=guide.Tick(now,dt,quiet&&healthy&&audit.Healthy(),stop,centered);
        if(guide.stage==Guide::Stage::Stopped) {
            const bool complete=std::strcmp(guide.reason,"four-second monotonic visual request deadline")==0;
            const bool writes_ok=audit.End();
            approved=complete&&writes_ok&&quiet&&healthy&&!stop&&centered;
            approved_ns=approved?now:0;
            if(approved)approved_bearing=SteeringBearing(guide.latest);
            reason=approved?"SHADOW PASSED: still ZERO OUTPUT; press T AGAIN to request one four-second walk":guide.reason;
            if(complete&&!writes_ok)reason="review failed: writer timing/state freshness or sample count";
        }
    }
};
}
