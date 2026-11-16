#include "production/precision_tcp_control.hpp"
#include "production/foot_seam_geometry.hpp"
#include "production/independent_model_io.hpp"
#include <mujoco/mujoco.h>
#include <cassert>
#include <iostream>
#include <random>
#include <filesystem>

int main() {
    char error[2048]{};
    auto* m=mj_loadXML("/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml",nullptr,error,sizeof(error));assert(m);
    auto* d=mj_makeData(m);assert(d);
    for(int i=0;i<3;++i)d->qpos[i]=0.;
    d->qpos[3]=1.;d->qpos[4]=d->qpos[5]=d->qpos[6]=0.;
    auto model=LoadIndependentModel((std::filesystem::path(__FILE__).parent_path()/"production/b2_independent_model.json").string());
    const int site=mj_name2id(m,mjOBJ_SITE,"ee_site");
    std::array<int,6> joint;
    for(int i=0;i<6;++i)joint[i]=mj_name2id(m,mjOBJ_JOINT,("joint"+std::to_string(i+1)).c_str());
    std::mt19937 rng(257);std::uniform_real_distribution<double> random(-1.,1.);
    double max_jacobian=0.,max_foot=0.,max_ik=0.;
    const char* names[]={"FR","FL","RR","RL"};const char* joints[]={"hip","thigh","calf"};
    for(int sample=0;sample<300;++sample) {
        va15_precision::Joints q{{.2*random(rng),2.18+.2*random(rng),-1.6+.2*random(rng),-.9+.2*random(rng),.2*random(rng),.2*random(rng)}};
        for(int i=0;i<6;++i)d->qpos[m->jnt_qposadr[joint[i]]]=q[i];
        std::vector<float> policy(18,0.);
        for(int l=0;l<4;++l)for(int j=0;j<3;++j) {
            const int id=mj_name2id(m,mjOBJ_JOINT,(std::string(names[l])+"_"+joints[j]+"_joint").c_str());
            const double value=j==0?.2*random(rng):j==1?.8+.2*random(rng):-1.5+.3*random(rng);
            policy[va15_foot_seam::PolicyDDS[3*l+j]]=float(value);
            d->qpos[m->jnt_qposadr[id]]=policy[va15_foot_seam::PolicyDDS[3*l+j]];
        }
        mj_forward(m,d);std::vector<double> jac(3*m->nv);
        mj_jacSite(m,d,jac.data(),nullptr,site);
        va15_tcp::ArmGeometry geometry;assert(va15_tcp::ArmTCPGeometry(q,geometry));
        for(int i=0;i<6;++i)for(int j=0;j<3;++j)
            max_jacobian=std::max(max_jacobian,std::fabs(geometry.jacobian(j,i)-jac[j*m->nv+m->jnt_dofadr[joint[i]]]));
        std::array<Eigen::Vector3d,4> feet;assert(va15_foot_seam::Centers(model,policy,feet));
        for(int l=0;l<4;++l) {
            const int id=mj_name2id(m,mjOBJ_GEOM,(std::string(names[l])+"_foot_collision").c_str());
            const Eigen::Vector3d expected(d->geom_xpos[3*id],d->geom_xpos[3*id+1],d->geom_xpos[3*id+2]);
            max_foot=std::max(max_foot,(feet[l]-expected).norm());
        }
        auto seed=q;for(auto& value:seed)value+=.02*random(rng);
        const auto solution=va15_precision::Solve(geometry.position,seed,q);assert(solution.valid);
        max_ik=std::max(max_ik,solution.residual);
    }
    std::cerr<<"geometry maxima: jac="<<max_jacobian<<" foot="<<max_foot<<" IK="<<max_ik<<'\n';
    // The archived URDF foot frame and MuJoCo foot sphere differ by at
    // most 174 um (small lateral fixed foot offsets), not a controller error.
    assert(max_jacobian<1.e-9&&max_foot<.00018&&max_ik<.001);
    const va15_precision::Joints q{{0.,2.18,-1.6,-.9,0.,0.}};
    const auto unreachable=va15_precision::Solve(Eigen::Vector3d(5,5,5),q,q);
    assert(unreachable.valid&&unreachable.residual>1.);
    va15_precision::Controller controller;std::vector<float> measured(18,0.),output(18,.123f);
    for(int i=0;i<6;++i)measured[va15_tcp::MeasuredTCPArmIndices[i]]=float(q[i]);
    va15_tcp::ArmGeometry k;assert(va15_tcp::ArmTCPGeometry(q,k));
    assert(controller.Update(measured,measured,k.position+Eigen::Vector3d(.01,0,0),.005,output));
    for(int i:va15_foot_seam::PolicyDDS)assert(output[i]==.123f);
    const auto held=output;controller.Hold(output);assert(held==output);
    assert(!controller.Update(measured,measured,k.position,NAN,output)&&output==held);
    mj_deleteData(d);mj_deleteModel(m);
    std::cout<<"PASS independent MuJoCo Jacobian="<<max_jacobian<<" feet="<<max_foot<<" IK="<<max_ik<<"; hold/invalid input/leg isolation\n";
}
