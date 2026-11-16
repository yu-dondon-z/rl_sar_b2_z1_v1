#pragma once
#include <cmath>

// Explicitly named no-camera diagnostic mode. It never creates fake camera
// packets, contact evidence, odometry, or a dynamic TCP path.
namespace va15_blind {
class Guide {
public:
    bool running=false;
    const char* reason="fixed TCP, no vision; explicit R1+Y required";
    bool Start(bool quiet_ready,bool centered,bool healthy) {
        if(running||!quiet_ready||!centered||!healthy) {
            reason="blind start refused: pending stop, readiness, neutral input or health";return false;
        }
        running=true;reason="blind fixed 0.15 m/s + original TCP; explicit stop required";return true;
    }
    void Stop(const char* why) {running=false;reason=why;}
    bool Tick(double dt,bool healthy,bool stop,bool centered) {
        if(!running)return false;
        if(!std::isfinite(dt)||dt<=0.||dt>.02||!healthy||stop||!centered) {
            Stop(stop?"operator/state stop":!healthy?"source/motor health lost":
                !centered?"manual input outside neutral":"invalid control timing");return true;
        }
        return false;
    }
};
}
