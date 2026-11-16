#include "production/va15_controlled_stop.hpp"
#include <cassert>
#include <iostream>

int main() {
    VA15StopFeedback f;f.lowstate_fresh=f.sport_fresh=f.contacts_valid=f.height_valid=f.velocity_valid=true;
    f.base_height=.52;f.planar_speed=.035;f.contacts.fill(true);
    auto review=[&](float tilt=0,float gyro=0,float dq=0,float drift=0){return VA15StopGateDiagnostics::Review(f,tilt,gyro,dq,drift,.48,.15);};
    assert(review().mask==0&&review().contacts==15);
    for(unsigned i=0;i<4;++i) {
        f.contacts[i]=false;auto d=review();assert(d.mask==VA15StopGateDiagnostics::PartialSupport);
        assert(d.contacts==(15u^(1u<<i)));assert(!va15_trial::Settled(f,0,0,0,.48,.15));f.contacts[i]=true;
    }
    f.planar_speed=.04;assert(review().mask==VA15StopGateDiagnostics::Speed);f.planar_speed=.035;
    assert(review(.16).mask==VA15StopGateDiagnostics::Tilt);
    assert(review(0,.18).mask==VA15StopGateDiagnostics::Gyro);
    assert(review(0,0,.55).mask==VA15StopGateDiagnostics::JointSpeed);
    assert(review(0,0,0,.031).mask==VA15StopGateDiagnostics::JointDrift);
    f.lowstate_fresh=false;assert(review().mask&VA15StopGateDiagnostics::Stale);f.lowstate_fresh=true;
    f.contacts_valid=false;assert(review().mask&VA15StopGateDiagnostics::ContactsUnknown);f.contacts_valid=true;
    f.base_height=NAN;assert(review().mask&VA15StopGateDiagnostics::Height);
    std::cout<<"PASS same final stop thresholds, per-foot failure bits, stale/invalid/gyro/drift distinction; diagnostics grant no authority\n";
}
