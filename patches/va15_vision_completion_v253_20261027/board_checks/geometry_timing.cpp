// Memory-only geometry benchmark. No SDK, DDS/UDP, camera or motor commands.
// Synthetic full caches exercise the production geometry; this is NOT a
// qualification of the real control loop, estimator or physical support.
#include "../production/visual_tcp.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>
using namespace va15_tcp;
int main() {
    Pose pose{100000000000ULL,{0.,0.,.52},Eigen::Quaterniond::Identity(),true};
    std::vector<double> elapsed;
    volatile double sink=0.;
    for(bool behind:{false,true}) {
        Tracker tracker;tracker.Begin(pose);
        for(unsigned i=0;i<Tracker::MaximumCurves;++i) {
            Tracker::Curve c;c.ns=pose.ns-100000000ULL;c.capture=pose;
            for(unsigned j=0;j<50;++j) {
                const double x=(behind?-1.5:1.2)+i*.035+j*.04;
                c.points.push_back({x,.04*std::sin(.8*x),0.});
            }
            c.near=c.points.front();c.look=c.points.back();
            tracker.curves.push_back(c);
        }
        tracker.live=tracker.curves.back();tracker.live_valid=true;
        tracker.live.near={1.2,.02,0.};tracker.live.look={2.,.03,0.};
        for(unsigned i=0;i<650;++i) {
            const auto start=std::chrono::steady_clock::now();
            double bearing=0.,e=0.,h=0.;bool history=false;
            Tracker::ForwardReference reference;
            tracker.Guidance(pose,bearing,&reference);
            tracker.BodyGuidance(pose,e,h,history);
            tracker.Step(pose,.005,true);
            sink+=bearing+e+h+tracker.target.x();
            const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            if(i>=50)elapsed.push_back(us);
        }
    }
    std::sort(elapsed.begin(),elapsed.end());double sum=0.;
    for(const auto v:elapsed)sum+=v;
    std::cout<<"{\"memory_only\":true,\"robot_initialized\":false,\"hardware_qualified\":false,"
             <<"\"full_cache_curves\":"<<Tracker::MaximumCurves<<",\"points_per_curve\":50,"
             <<"\"geometry_calls_per_sample\":\"Guidance + BodyGuidance (including its Guidance) + TCP Step\","
             <<"\"samples\":"<<elapsed.size()<<",\"mean_us\":"<<sum/elapsed.size()
             <<",\"p50_us\":"<<elapsed[elapsed.size()/2]<<",\"p95_us\":"<<elapsed[elapsed.size()*95/100]
             <<",\"p99_us\":"<<elapsed[elapsed.size()*99/100]<<",\"max_us\":"<<elapsed.back()
             <<",\"synthetic_sink\":"<<sink<<"}\n";
}
