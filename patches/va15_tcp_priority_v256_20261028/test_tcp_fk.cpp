// SDK-free model geometry test. Not physical tool calibration.
#include "production/tcp_fk.hpp"
#include <mujoco/mujoco.h>
#include <cassert>
#include <iostream>
#include <limits>
#include <random>

int main() {
    char error[2048]{};
    auto* model=mj_loadXML("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",nullptr,error,sizeof(error));
    assert(model);auto* data=mj_makeData(model);assert(data);
    const int site=mj_name2id(model,mjOBJ_SITE,"ee_site");assert(site>=0);
    std::array<int,6> ids;
    for(int i=0;i<6;++i){ids[i]=mj_name2id(model,mjOBJ_JOINT,("joint"+std::to_string(i+1)).c_str());assert(ids[i]>=0);}
    std::mt19937 rng(249);std::uniform_real_distribution<double> uniform(0.,1.);
    double max_error=0.;
    for(int sample=0;sample<1000;++sample) {
        std::vector<double> policy(18,0.);
        for(int i=0;i<6;++i) {
            const double lower=model->jnt_range[2*ids[i]],upper=model->jnt_range[2*ids[i]+1];
            const double value=lower+uniform(rng)*(upper-lower);
            policy[va15_tcp::MeasuredTCPArmIndices[i]]=value;
            data->qpos[model->jnt_qposadr[ids[i]]]=value;
        }
        const Eigen::Vector3d p(uniform(rng),uniform(rng),.52);
        const Eigen::Quaterniond q(Eigen::AngleAxisd(uniform(rng),Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(.2*uniform(rng),Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(.2*uniform(rng),Eigen::Vector3d::UnitX()));
        for(int i=0;i<3;++i)data->qpos[i]=p[i];
        data->qpos[3]=q.w();data->qpos[4]=q.x();data->qpos[5]=q.y();data->qpos[6]=q.z();
        mj_forward(model,data);
        Eigen::Vector3d measured;
        assert(va15_tcp::MeasuredTCPWorld(p,q,policy,measured));
        const Eigen::Vector3d expected(data->site_xpos[3*site],data->site_xpos[3*site+1],data->site_xpos[3*site+2]);
        max_error=std::max(max_error,(expected-measured).norm());
        policy[8]=std::numeric_limits<double>::quiet_NaN();const auto unchanged=measured;
        assert(!va15_tcp::MeasuredTCPBase(policy,measured)&&(measured-unchanged).norm()==0.);
    }
    assert(max_error<1.e-10);mj_deleteData(data);mj_deleteModel(model);
    std::cout<<"PASS 1000 measured virtual welding-TCP FK poses; model error="<<max_error<<"m; not hardware calibration\n";
}
