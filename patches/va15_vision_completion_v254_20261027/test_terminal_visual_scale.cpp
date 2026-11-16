#include "production/terminal_visual_scale.hpp"
#include <cassert>
#include <iostream>
using namespace va15_tcp;
int main() {
    TerminalVisualScale s;Pose p{1000000000ULL,{0.,0.,.52},Eigen::Quaterniond::Identity(),true},out;
    assert(s.Correct(p,out)&&!s.valid);
    for(int i=0;i<25;++i) {
        p.ns+=150000000;p.p.x()=.05*i;
        s.Add(p,V3(5.-.05*p.p.x(),0.,0.),V3::UnitX());
    }
    assert(s.valid&&!s.fault&&std::fabs(s.scale-1.05)<1.e-10&&s.rms<1.e-10);
    p.ns+=100000000;p.p.x()+=2.;
    assert(s.Correct(p,out)&&std::fabs(out.p.x()-p.p.x()-.10)<1.e-10);
    const auto timestamp=p.ns;assert(out.ns==timestamp&&out.q.coeffs()==p.q.coeffs());
    p.p.x()+=4.;assert(!s.Correct(p,out)); // finite segment only
    TerminalVisualScale still;
    for(int i=0;i<30;++i){p.ns+=150000000;p.p.x()=.001*i;still.Add(p,{5.,0.,0.},V3::UnitX());}
    assert(!still.valid); // stationary feature jitter is not a scale calibration
    TerminalVisualScale short_view;
    for(int i=0;i<25;++i) {
        p.ns+=150000000;p.p.x()=.408*i/24.;
        short_view.Add(p,{5.-.04*p.p.x()+.0005*std::sin(i),0.,0.},V3::UnitX());
    }
    assert(short_view.valid&&short_view.consistent==1&&std::fabs(short_view.scale-1.04)<.005);
    assert(short_view.slope_error<.01); // only one fit, but many independent observations
    TerminalVisualScale wrong;
    for(int i=0;i<25;++i){p.ns+=150000000;p.p.x()=.05*i;wrong.Add(p,{5.-.30*p.p.x(),0.,0.},V3::UnitX());}
    assert(!wrong.valid); // cannot silently calibrate arbitrary odom failure
    TerminalVisualScale delayed=s;
    p=delayed.anchor;p.ns+=46000000000ULL;assert(!delayed.Correct(p,out));
    auto n=s.samples.size();s.Add(s.anchor,{NAN,0.,0.},V3::UnitX());assert(s.samples.size()==n);
    std::cout<<"PASS static-cap relative visual scale, finite anchoring, no stationary calibration, no excessive scale or stale pose authority\n";
}
