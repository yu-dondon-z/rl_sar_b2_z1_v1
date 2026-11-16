#include "independent_model_io.hpp"
#include <cassert>
#include <iostream>
using namespace va15_independent;
int main(int argc,char** argv)
{
    if(argc!=3)throw std::runtime_error("Expected model and goldens");
    const auto m=LoadIndependentModel(argv[1]);Dynamics d(m);const auto root=YAML::LoadFile(argv[2]);
    assert(root["source_sha256"].as<std::string>()==m.source_sha256);
    double torque_error=0.,force_error=0.;int count=0;
    auto vec=[](const YAML::Node& n){const auto v=n.as<std::vector<double>>();return V3(v[0],v[1],v[2]);};
    for(const auto& item:root["cases"])
    {
        Sample s;std::array<double,12> qdd;
        for(int i=0;i<12;++i){s.q[i]=item["q"][i].as<double>();s.dq[i]=item["dq"][i].as<double>();s.tau[i]=item["tau"][i].as<double>();qdd[i]=item["qdd"][i].as<double>();}
        const auto q=item["quaternion"].as<std::vector<double>>();s.quaternion=Eigen::Quaterniond(q[0],q[1],q[2],q[3]);
        s.gyro=vec(item["gyro"]);s.specific_force=vec(item["specific_force"]);
        for(int leg=0;leg<4;++leg)
        {
            const auto f=d.Evaluate(leg,s,qdd,vec(item["angular_acceleration"]));
            torque_error=std::max(torque_error,(f.dynamics_torque-vec(item["expected_inverse_torque"][leg])).cwiseAbs().maxCoeff());
            force_error=std::max(force_error,(f.force_world-vec(item["expected_force"][leg])).cwiseAbs().maxCoeff());
        }
        ++count;
    }
    std::cout<<"Independent MuJoCo inverse-dynamics cases="<<count<<" max_tau_error="<<torque_error<<" max_force_error="<<force_error<<'\n';
    assert(count>=64&&torque_error<1e-8&&force_error<1e-8);
}
