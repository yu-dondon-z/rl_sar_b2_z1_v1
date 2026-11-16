#pragma once
#include "near_visual_guide.hpp"

namespace va15_priority {
// A get-down INTENT preempts visual work, not the physical support checks.
// No frame, timestamp, model contact or lowering permission is created here.
// Caller must latch the FSM request and still execute its supported stop.
inline bool PreemptVisualForGetDown(bool pressed,bool pending,
    va15_visual::Guide& guide,va15_visual::Output& output) {
    if(!pressed&&!pending)return false;
    guide.Stop("operator get-down priority; visual session cancelled");
    guide.reason="operator get-down priority; visual session cancelled";
    output={0.,0.,true};
    return true;
}
}
