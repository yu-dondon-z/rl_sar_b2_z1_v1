#include "production/visual_tcp.hpp"
#include "production/visual_rear_completion.hpp"
#include <cassert>
#include <iostream>
using namespace va15_tcp;
int main() {
    Pose pose{10000000000ULL,{0.,0.,.52},Eigen::Quaterniond::Identity(),true};
    Tracker strict,recovery;strict.Begin(pose);recovery.Begin(pose);
    recovery.training_workspace_recovery=true;
    Tracker::Curve c;c.ns=pose.ns;c.capture=pose;c.points={{-1.,.24,0.},{-.55,.24,0.}};
    strict.curves.push_back(c);recovery.curves.push_back(c);
    strict.Step(pose,.005,true);assert(!strict.active&&strict.workspace_rejections);
    const auto real_points=c.points;
    for(int i=0;i<1000;++i) {
        const auto before=recovery.target;
        assert(recovery.Step(pose,.005,true));
        assert(recovery.active&&recovery.workspace_limited);
        assert(std::fabs(recovery.observed_goal.y()-.24)<1.e-9);
        assert(std::fabs(recovery.desired.y()-.15)<1.e-9);
        assert(std::fabs(recovery.allocation_error-.09)<1.e-9);
        assert((recovery.target-before).cwiseAbs().maxCoeff()<=.000200001);
        assert(recovery.target.y()<=.15&&recovery.target.y()>=-.15);
        assert(recovery.curves.front().points==real_points); // no invented crack
    }
    const auto held=recovery.target;recovery.Step(pose,.005,false);
    assert(!recovery.active&&(recovery.target-held).norm()==0.);
    for(const std::vector<V3> bad:{std::vector<V3>{{1.,0.,0.},{2.,0.,0.}},
                                  std::vector<V3>{{-1.,.6,0.},{-.55,.6,0.}}}) {
        recovery.curves.front().points=bad;
        recovery.Step(pose,.005,true);assert(!recovery.active);
    }
    recovery.ClearObservations();assert(recovery.training_workspace_recovery);
    recovery.Step(pose,.005,true);assert(!recovery.active);
    V3 endpoint(-.775,.24,0.);
    assert(recovery.StepEndpoint(pose,.005,endpoint));
    assert(recovery.workspace_limited&&recovery.observed_goal.y()==.24&&recovery.desired.y()==.15);
    const auto before=recovery.target;assert(!recovery.StepEndpoint(pose,.005,V3(1.,0.,0.)));
    assert((before-recovery.target).norm()==0.);
    std::cout<<"PASS archived training allocation, actual-vs-bounded goal separation, original rates/workspace, no unseen rear path and no target-based completion\n";
}
