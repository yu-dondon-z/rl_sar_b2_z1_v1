#include "b_stop_brake.hpp"
#include <cassert>
#include <limits>
int main()
{
    const std::vector<float> ordinary{.12f,.12f,.30f};
    assert(va15_sim_brake::Limits(ordinary,false,.4)==ordinary);
    assert(va15_sim_brake::Limits(ordinary,true,.12)==ordinary);
    const auto brake=va15_sim_brake::Limits(ordinary,true,.3);
    assert(brake[0]==.3f && brake[1]==.3f && brake[2]==ordinary[2]);
    assert(va15_sim_brake::Limits(brake,true,.3)==brake);
    assert(ordinary[0]==.12f); // caller's baseline never modified
    for (double invalid: {.0,.119,.61,std::numeric_limits<double>::quiet_NaN()}) {
        bool refused=false;
        try {va15_sim_brake::Limits(ordinary,true,invalid);} catch(const std::invalid_argument&) {refused=true;}
        assert(refused);
    }
    bool refused=false;
    try {va15_sim_brake::Limits({.12f,.12f},true,.3);} catch(const std::invalid_argument&) {refused=true;}
    assert(refused);
}
