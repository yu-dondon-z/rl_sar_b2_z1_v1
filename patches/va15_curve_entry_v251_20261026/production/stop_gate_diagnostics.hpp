#pragma once
#include "trial_feedback.hpp"
#include <cmath>
#include <limits>

// Read-only explanation of the EXISTING final stop predicate. This object
// has no authority to supply contacts, clear faults, or permit lowering.
struct VA15StopGateDiagnostics {
    enum Failure : unsigned {
        Source=1, ContactsUnknown=2, PartialSupport=4, Height=8, Speed=16,
        Tilt=32, Gyro=64, JointSpeed=128, JointDrift=256, Stale=512, Config=1024
    };
    unsigned mask=0,contacts=0;
    bool contacts_valid=false,source_valid=false;
    float height=NAN,speed=NAN,tilt=NAN,gyro=NAN,joint_speed=NAN,drift=NAN;
    static VA15StopGateDiagnostics Review(const VA15StopFeedback& f,float tilt,
        float gyro,float dq,float drift,float h,float max_tilt) {
        VA15StopGateDiagnostics d;d.height=f.base_height;d.speed=f.planar_speed;
        d.tilt=tilt;d.gyro=gyro;d.joint_speed=dq;d.drift=drift;
        d.contacts_valid=f.contacts_valid;d.source_valid=va15_trial::FeedbackValid(f);
        for(unsigned i=0;i<4;++i)if(f.contacts[i])d.contacts|=1u<<i;
        if(!d.source_valid)d.mask|=Source;
        if(!f.lowstate_fresh)d.mask|=Stale;
        if(!f.contacts_valid)d.mask|=ContactsUnknown;
        if(d.contacts!=15)d.mask|=PartialSupport;
        if(!std::isfinite(h)||h<=0.f||!std::isfinite(max_tilt)||max_tilt<=0.f)d.mask|=Config;
        if(!std::isfinite(f.base_height)||f.base_height<h)d.mask|=Height;
        if(!std::isfinite(f.planar_speed)||f.planar_speed<0.f||f.planar_speed>=.04f)d.mask|=Speed;
        if(!std::isfinite(tilt)||tilt<0.f||tilt>max_tilt)d.mask|=Tilt;
        if(!std::isfinite(gyro)||gyro<0.f||gyro>=.18f)d.mask|=Gyro;
        if(!std::isfinite(dq)||dq<0.f||dq>=.55f)d.mask|=JointSpeed;
        if(!std::isfinite(drift)||drift<0.f||drift>.03f)d.mask|=JointDrift;
        return d;
    }
};
