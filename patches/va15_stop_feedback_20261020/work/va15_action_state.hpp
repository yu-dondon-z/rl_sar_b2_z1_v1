#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

// VA15 last_action is bounded raw actor output; actuator EMA is separate.
class VA15ActionState {
public:
    std::vector<float> Update(const std::vector<float>& actor,
        std::vector<float>& observation, float alpha, float clip, bool reset) {
        if (actor.size()!=18 || !std::isfinite(alpha) || alpha<=0 || alpha>1 ||
            !std::isfinite(clip) || clip<=0)
            throw std::runtime_error("Invalid VA15 action contract");
        for(float x:actor) if(!std::isfinite(x))
            throw std::runtime_error("Nonfinite VA15 actor output");
        if(reset) applied_.assign(18,0.f);
        observation.resize(18);
        for(int i=0;i<18;++i) {
            observation[i]=std::clamp(actor[i],-clip,clip);
            applied_[i]=alpha*observation[i]+(1-alpha)*applied_[i];
        }
        return applied_;
    }
    const std::vector<float>& Applied() const {return applied_;}
private:
    std::vector<float> applied_=std::vector<float>(18,0.f);
};
