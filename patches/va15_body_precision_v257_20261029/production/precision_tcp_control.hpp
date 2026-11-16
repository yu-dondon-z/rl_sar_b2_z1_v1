#pragma once
#include "tcp_fk.hpp"
#include <Eigen/Cholesky>
#include <algorithm>

namespace va15_precision {
using V3=Eigen::Vector3d;
using Joints=std::array<double,6>;

inline bool Extract(const std::vector<float>& policy,Joints& q) {
    if(policy.size()!=18)return false;
    for(int i=0;i<6;++i)q[i]=policy[va15_tcp::MeasuredTCPArmIndices[i]];
    va15_tcp::ArmGeometry check;return va15_tcp::ArmTCPGeometry(q,check);
}

struct SolveResult {Joints q{};double residual=0.;bool valid=false,limited=false;};
inline SolveResult Solve(const V3& target,const Joints& seed,const Joints& posture) {
    SolveResult out;out.q=seed;
    if(!target.allFinite())return out;
    for(int iteration=0;iteration<18;++iteration) {
        va15_tcp::ArmGeometry k;if(!va15_tcp::ArmTCPGeometry(out.q,k))return out;
        const V3 error=target-k.position;
        const Eigen::Matrix3d metric=k.jacobian*k.jacobian.transpose()+.0001*Eigen::Matrix3d::Identity();
        const Eigen::Matrix<double,6,3> inverse=k.jacobian.transpose()*metric.ldlt().solve(Eigen::Matrix3d::Identity());
        Eigen::Matrix<double,6,1> prior;
        for(int i=0;i<6;++i)prior[i]=posture[i]-out.q[i];
        Eigen::Matrix<double,6,1> delta=inverse*error+
            (Eigen::Matrix<double,6,6>::Identity()-inverse*k.jacobian)*prior*.04;
        const double scale=std::max(1.,delta.cwiseAbs().maxCoeff()/.06);
        for(int i=0;i<6;++i) {
            const double raw=out.q[i]+delta[i]/scale;
            out.q[i]=std::clamp(raw,va15_tcp::MeasuredTCPArmLower[i]+.015,va15_tcp::MeasuredTCPArmUpper[i]-.015);
            out.limited|=std::fabs(out.q[i]-raw)>1.e-8;
        }
        if(error.norm()<.00015&&delta.cwiseAbs().maxCoeff()<.0003)break;
    }
    va15_tcp::ArmGeometry final;
    out.valid=va15_tcp::ArmTCPGeometry(out.q,final);
    if(out.valid)out.residual=(target-final.position).norm();
    return out;
}

// Cartesian residual applied ONLY to Z1 position commands after the actor/FSM.
// Measured state and the original VA15 actor, gains, contacts and frame ages
// are unchanged. Command continuity is retained across ordinary visual stops.
class Controller {
public:
    Joints command{};
    V3 integral=V3::Zero(),error=V3::Zero(),goal=V3::Zero();
    double ik_residual=0.,max_joint_step=0.;
    bool engaged=false,tracking=false,limited=false;
    unsigned precision_samples=0;
    void Reset(){*this=Controller();}
    bool Update(const std::vector<float>& measured,const std::vector<float>& actor,
                const V3& actual_goal,double dt,std::vector<float>& output) {
        Joints q,prior;
        if(!Extract(measured,q)||!Extract(actor,prior)||output.size()!=18||
           !actual_goal.allFinite()||!std::isfinite(dt)||dt<=0.||dt>.02)return false;
        va15_tcp::ArmGeometry geometry;if(!va15_tcp::ArmTCPGeometry(q,geometry))return false;
        if(!engaged) {command=prior;integral.setZero();engaged=true;}
        goal=actual_goal;error=goal-geometry.position;
        // Remove quasi-static servo load error using actual encoder feedback.
        // Integrate only near the physical target and without saturation.
        if(error.norm()<.05&&!limited) {
            integral+=2.*dt*error;
            if(integral.norm()>.04)integral*=.04/integral.norm();
        }
        V3 feedback=.30*error;
        if(feedback.norm()>.018)feedback*=.018/feedback.norm();
        const auto solution=Solve(goal+feedback+integral,command,prior);
        if(!solution.valid)return false;
        ik_residual=solution.residual;limited=solution.limited||ik_residual>.003;
        max_joint_step=0.;
        for(int i=0;i<6;++i) {
            const double delta=solution.q[i]-command[i];
            const double step=std::clamp(delta,-.30*dt,.30*dt);
            limited|=std::fabs(delta-step)>1.e-8;
            command[i]+=step;max_joint_step=std::max(max_joint_step,std::fabs(step));
            output[va15_tcp::MeasuredTCPArmIndices[i]]=float(command[i]);
        }
        precision_samples=error.norm()<=.01?precision_samples+1:0;
        tracking=precision_samples>=100; // 0.5 s of actual <=10 mm feedback
        return true;
    }
    void Hold(std::vector<float>& output)const {
        if(engaged&&output.size()==18)
            for(int i=0;i<6;++i)output[va15_tcp::MeasuredTCPArmIndices[i]]=float(command[i]);
    }
};
}
