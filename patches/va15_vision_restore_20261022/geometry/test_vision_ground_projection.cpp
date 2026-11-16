#include "vision_ground_projection.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <string>
using namespace vision_ground_projection;
int main(int argc, char** argv) {
    if(argc>1 && std::string(argv[1])=="--project") {
        Vec3 t{},o{},d{},p{}; Quat q{}; double z;
        while(std::cin>>t[0]>>t[1]>>t[2]>>q[0]>>q[1]>>q[2]>>q[3]
              >>o[0]>>o[1]>>o[2]>>d[0]>>d[1]>>d[2]>>z) {
            bool ok=Project(t,q,o,d,z,&p);
            std::cout.precision(15);
            std::cout<<ok<<' '<<p[0]<<' '<<p[1]<<' '<<p[2]<<'\n';
        }
        return 0;
    }
    Vec3 p{}; const Vec3 t{{0,0,.52}},o{{.446,0,0}};
    const Quat q{{1,0,0,0}};
    assert(Project(t,q,o,{{1.,0.,-.52}},0.,&p));
    assert(std::abs(p[2]+.52)<1e-10);
    assert(!Project(t,q,o,{{1.,0.,0.}},0.,&p));
    assert(!Project(t,q,o,{{1.,0.,.5}},0.,&p));
    assert(!Project(t,q,o,{{0.,0.,0.}},0.,&p));
    assert(!Project({{0,0,.15}},q,o,{{1,0,-.52}},0.,&p));
    assert(!Project(t,{{2,0,0,0}},o,{{1,0,-.52}},0.,&p));
    assert(!Project(t,q,o,{{1,0,std::numeric_limits<double>::quiet_NaN()}},0.,&p));
    assert(Synchronized(1000000000,1001000000,1100000000,true));
    assert(!Synchronized(1000000000,1001000000,1100000000,false));
    assert(!Synchronized(1000000000,1100000000,1100000000,true));
    assert(!Synchronized(1000000000,1000000000,1600000000,true));
    assert(!Synchronized(1000000000,1200000000,1100000000,true));
    assert(!Synchronized(1200000000,1100000000,1100000000,true));
    std::cout<<"PASS vision ground geometry and timestamp guards\n";
}
