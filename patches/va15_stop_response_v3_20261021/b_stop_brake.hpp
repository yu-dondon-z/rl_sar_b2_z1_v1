#pragma once
#ifndef VA15_ESTIMATED_STOP_SIMULATION_ONLY
#error "B-stop experiment is simulation-only, not a real control implementation"
#endif
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace va15_sim_brake {
// This changes only in-memory XY command DECELERATION after the latched B
// request. No pose, gain, actor, acceleration or support threshold is changed.
// The experiment exercises forward motion only, not lateral/yaw qualification.
inline std::vector<float> Limits(const std::vector<float>& ordinary, bool requested, double b_limit)
{
    if (ordinary.size()!=3 || !std::isfinite(b_limit) || b_limit<.12 || b_limit>.6)
        throw std::invalid_argument("Invalid simulation B-deceleration envelope");
    for (float value:ordinary)
        if (!std::isfinite(value) || value<=0.f)
            throw std::invalid_argument("Invalid ordinary command limits");
    auto result=ordinary;
    if (requested) for (int i=0;i<2;++i) result[i]=std::max(result[i],static_cast<float>(b_limit));
    return result;
}
}
