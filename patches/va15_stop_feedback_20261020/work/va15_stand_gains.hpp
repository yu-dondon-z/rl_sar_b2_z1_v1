#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

inline constexpr std::array<int,12> VA15LegIndices{{0,1,2,3,4,5,6,7,9,10,11,12}};

inline std::vector<float> VA15BlendGains(const std::vector<float>& first,
    const std::vector<float>& last,float progress)
{
    if(first.size()!=18 || last.size()!=18 || !std::isfinite(progress))
        throw std::runtime_error("Invalid stand gain dimensions/progress");
    const float t=std::clamp(progress,0.f,1.f);
    const float blend=t*t*(3.f-2.f*t);
    std::vector<float> out(18);
    for(int i=0;i<18;++i) {
        if(!std::isfinite(first[i]) || !std::isfinite(last[i]) || first[i]<0 || last[i]<0)
            throw std::runtime_error("Invalid stand gain");
        out[i]=first[i]+blend*(last[i]-first[i]);
    }
    return out;
}
