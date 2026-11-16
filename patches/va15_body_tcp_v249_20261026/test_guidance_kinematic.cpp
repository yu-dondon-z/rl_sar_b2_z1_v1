// Controller-only geometry check; not a robot dynamics/hardware test.
#include "production/visual_tcp.hpp"
#include <cassert>
#include <iostream>
using namespace va15_tcp;
static va15_visual::Packet Packet(unsigned seq,std::uint64_t ns) {
    va15_visual::Packet p;p.valid=p.session_matched=true;p.sequence=seq;p.capture_ns=ns;p.publish_ns=ns;
    p.u=p.look_u=935.;p.v=972.;p.look_v=820.8;p.confidence=p.look_confidence=.99;p.width=30.;p.contrast=.8;p.rms=1.;p.path_count=50;p.path_span=300.;return p;
}
int main() {
    for(double initial_y:{-.20,.20})for(double lateral_bias:{-.012,0.,.012}) {
        const auto path=[](double x){return .08*std::sin(.4*x);};
        double x=0.,y=initial_y,theta=.08,omega=0.,max_error=0.;
        va15_visual::Guide guide(true,true,true);Tracker tracker;unsigned seq=0;
        const std::uint64_t base=10000000000ULL;
        for(unsigned i=1;i<=6;++i)guide.Observe(Packet(++seq,base+i*100000000ULL),base+i*100000000ULL);
        assert(guide.Start(base+600000000ULL,true,true,true));
        for(unsigned step=0;step<24000;++step) {
            const auto ns=base+605000000ULL+step*5000000ULL;
            Pose pose{ns,{x,y,.52},Eigen::Quaterniond(Eigen::AngleAxisd(theta,V3::UnitZ())),true};
            if(step%30==0) {
                guide.Observe(Packet(++seq,ns),ns);
                tracker.live={};tracker.live.ns=ns;tracker.live_valid=true;
                tracker.live.near={x+1.2,path(x+1.2),0.};tracker.live.look={x+2.,path(x+2.),0.};
            }
            guide.metric_bearing_valid=tracker.Guidance(pose,guide.metric_bearing);
            guide.body_reference_valid=tracker.BodyGuidance(pose,guide.body_lateral,guide.body_heading,guide.body_history_used);
            // Deliberately unreliable velocity-course estimate must have no
            // command effect; the exact same controller is tested both ways.
            guide.course_slip=.25*std::sin(step*.002);
            auto shadow=guide;shadow.course_slip=-guide.course_slip;
            const auto a=guide.Tick(ns,.005,true,false,true),b=shadow.Tick(ns,.005,true,false,true);
            assert(!a.stop&&a.yaw==b.yaw&&std::fabs(a.yaw)<=va15_visual::YawLimit);
            omega+=.005/(.25+.005)*(a.yaw-omega);theta+=omega*.005;
            x+=(.15*std::cos(theta)-lateral_bias*std::sin(theta))*.005;
            y+=(.15*std::sin(theta)+lateral_bias*std::cos(theta))*.005;
            if(step>16000)max_error=std::max(max_error,std::fabs(y-path(x)));
        }
        assert(max_error<.10);
        std::cout<<"PASS geometry y0="<<initial_y<<" lateral_bias="<<lateral_bias<<" final_error="<<y-path(x)
                 <<" late_max_error="<<max_error<<"; synthetic geometry only\n";
    }
}
