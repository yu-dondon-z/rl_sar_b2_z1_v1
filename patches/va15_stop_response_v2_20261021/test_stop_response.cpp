// Isolated transition-shape regression; no transport is linked.
#define VA15ControlledStop VA15ArchivedStopNotSelected
#include "../va15_controlled_getdown_20261020/va15_controlled_stop.hpp"
#undef VA15ControlledStop
#include "stop_response_controller.hpp"
#define main archived_controlled_stop_tests
#include "archived_stop_tests.cpp"
#undef main

int main()
{
    archived_controlled_stop_tests();
    std::ostringstream quiet;auto* old=std::cout.rdbuf(quiet.rdbuf());
    for(double duration: {.5,.75,1.,1.5,2.}) {
        OfflineRL rl;Init(rl); VA15ControlledStop stop;
        stop.pose_blend_duration=duration;Prepare(stop,rl);
        Tick(stop,rl);Tick(stop,rl);assert(stop.captured);
        for(int i=0;i<int(duration/.005);++i)Tick(stop,rl);
        assert(!stop.Ready()); // never shorten the independent 0.5 s quiet gate
        for(int i=0;i<80;++i)Tick(stop,rl);
        assert(!stop.Ready());
        for(int i=0;i<22;++i)Tick(stop,rl);
        assert(stop.Ready());
    }
    for(double invalid: {.49,2.01,std::numeric_limits<double>::quiet_NaN()}) {
        OfflineRL rl;Init(rl);VA15ControlledStop stop;Prepare(stop,rl);
        stop.pose_blend_duration=invalid;Tick(stop,rl);
        assert(stop.stage==VA15ControlledStop::Stage::Blocked&&!stop.Ready());
    }
    std::cout.rdbuf(old);
    std::cout<<"PASS: bounded transition duration cannot shorten support qualification\n";
}
