#pragma once
#include "va15_independent_state.hpp"
#include <yaml-cpp/yaml.h>
#include <vector>

inline va15_independent::Model LoadIndependentModel(const std::string& path)
{
    using namespace va15_independent;
    const auto root=YAML::LoadFile(path);
    if(root["schema"].as<std::string>()!="va15_independent_state_v1"||
       root["dds_order"].as<std::vector<std::string>>()!=std::vector<std::string>{"FR","FL","RR","RL"}||
       root["control_authorized"].as<bool>()||root["hardware_qualified"].as<bool>())
        throw std::invalid_argument("Model must be unqualified, native DDS order");
    auto vec=[](const YAML::Node& n){const auto v=n.as<std::vector<double>>();
        if(v.size()!=3)throw std::invalid_argument("Expected three-vector");return V3(v[0],v[1],v[2]);};
    Model m;m.source_sha256=root["source_urdf_sha256"].as<std::string>();m.imu_position=vec(root["imu_position_base"]);
    if(root["legs"].size()!=4)throw std::invalid_argument("Expected four legs");
    for(int i=0;i<4;++i)
    {
        const auto leg=root["legs"][i];auto& l=m.legs[i];
        if(leg["joints"].size()!=3||leg["bodies"].size()!=4)throw std::invalid_argument("Unexpected leg topology");
        for(int j=0;j<3;++j)
        {
            const auto joint=leg["joints"][j];auto& v=l.joints[j];v.origin=vec(joint["origin"]);
            v.rpy=vec(joint["rpy"]);v.axis=vec(joint["axis"]);v.lower=joint["lower"].as<double>();v.upper=joint["upper"].as<double>();
        }
        l.tip=vec(leg["tip"]);l.radius=leg["radius"].as<double>();
        for(int j=0;j<4;++j)
        {
            const auto b=leg["bodies"][j];l.bodies[j].mass=b["mass"].as<double>();l.bodies[j].com=vec(b["com"]);
            if(b["inertia_com_link"].size()!=3)throw std::invalid_argument("Expected inertia matrix");
            for(int k=0;k<3;++k)l.bodies[j].inertia.row(k)=vec(b["inertia_com_link"][k]).transpose();
        }
    }
    Dynamics validate(m);return m;
}
