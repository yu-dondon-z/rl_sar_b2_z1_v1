#include "independent_model_io.hpp"
#include <cassert>
#include <iostream>
#include <chrono>

using namespace va15_independent;
static Sample Standing(const Model& m,double load=190.)
{
    Sample s;s.specific_force=V3(0,0,9.81);s.mode.fill(10);
    for(int i=0;i<4;++i){s.q[3*i]=0.;s.q[3*i+1]=.8;s.q[3*i+2]=-1.6;}
    Dynamics dynamics(m);std::array<double,12> zero{};
    for(int i=0;i<4;++i)
    {const auto f=dynamics.Evaluate(i,s,zero,V3::Zero());const V3 tau=f.dynamics_torque-f.jacobian.transpose()*V3(0,0,load);
     for(int j=0;j<3;++j)s.tau[3*i+j]=tau[j];}
    return s;
}
static Result Run(Observer& o,const Sample& s,int begin,int end)
{for(int i=begin;i<end;++i)o.Update(s,static_cast<std::uint32_t>(5*i),.005*i);return o.Snapshot(.005*(end-1));}
int main(int argc,char** argv)
{
    if(argc!=2)throw std::runtime_error("Expected explicit model");
    const Model m=LoadIndependentModel(argv[1]);const Sample s=Standing(m);
    {
        Observer o(m);const auto r=Run(o,s,0,300);assert(r.estimate_valid_for_review&&r.stop_candidate_for_review);
        assert(r.support_count==4&&r.accepted_feet==4&&r.base_height>.48&&r.base_height<.56&&r.planar_speed<1e-8);
        assert(!r.control_authorized&&!r.hardware_qualified&&!r.reported_ddq_used&&!r.common_mode_slip_observable);
        // Duplicates do NOT renew freshness.
        o.Update(s,1495,1.8);const auto stale=o.Snapshot(1.8);assert(!stale.fresh&&!stale.stop_candidate_for_review);
    }
    for(double load:{0.,-100.})
    {Observer o(m);const auto r=Run(o,Standing(m,load),0,500);assert(!r.initialized&&!r.stop_candidate_for_review);}
    {
        auto bad=s;bad.specific_force.z()=-9.81;Observer o(m);const auto r=Run(o,bad,0,300);assert(!r.initialized);
    }
    for(int mode=0;mode<6;++mode)
    {
        Observer o(m);Run(o,s,0,300);Sample bad=s;
        if(mode==0)bad.q[0]=2.;if(mode==1)bad.tau[0]=NaN;if(mode==2)bad.mode[2]=0;
        if(mode==3)bad.lost[4]=1;if(mode==4)bad.quaternion=Eigen::Quaterniond(0,0,0,0);
        if(mode==5)bad.specific_force.x()=100.;
        const auto r=Run(o,bad,300,310);assert(!r.estimate_valid_for_review&&!r.stop_candidate_for_review);
    }
    for(int leg=0;leg<4;++leg)
    {
        Observer o(m);Run(o,s,0,300);auto missing=s;const auto air=Standing(m,0.);
        for(int j=0;j<3;++j)missing.tau[3*leg+j]=air.tau[3*leg+j];
        const auto r=Run(o,missing,300,400);assert(!r.load_candidate[leg]&&r.support_count==3&&!r.stop_candidate_for_review);
    }
    {
        Observer o(m);Run(o,s,0,300);o.Update(s,3,1.5);const auto r=o.Snapshot(1.5);assert(!r.initialized&&!r.stop_candidate_for_review);
        assert(Run(o,s,301,600).initialized);
    }
    {
        Observer o(m);Run(o,s,0,300);o.Update(s,1600,1.8);assert(!o.Snapshot(1.8).initialized);
        o.Update(s,1605,1.6);assert(!o.Snapshot(1.6).initialized);
    }
    {
        Observer o(m);auto reported=s;reported.reported_ddq.fill(NaN);
        assert(Run(o,reported,0,300).stop_candidate_for_review); // deliberately not used
    }
    {
        Observer o(m);auto biased=s;biased.gyro=V3(.001,-.0085,.002);biased.specific_force+=V3(.02,-.03,-.025);
        const auto r=Run(o,biased,0,500);assert(r.estimate_valid_for_review&&r.planar_speed<.003);
        assert((r.gyro_bias-biased.gyro).norm()<1e-8);
    }
    {
        // Fundamental ambiguity: constant common-mode slip can have the
        // same IMU/encoders as zero velocity. We expose this limitation and
        // never claim that a low covariance proves no slip or grants control.
        Observer o(m);const auto r=Run(o,s,0,300);
        assert(!r.common_mode_slip_observable&&!r.control_authorized);
    }
    std::cout<<"PASS: quiet initialization, unloaded/negative loads, IMU sign, invalid motors, four individual unloadings, duplicate/reset/gap/clock, unused ddq, bias correction, no authorization or common-slip claim\n";
}
