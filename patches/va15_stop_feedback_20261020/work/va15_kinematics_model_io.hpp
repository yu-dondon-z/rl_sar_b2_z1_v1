#pragma once
#include "va15_kinematics_shadow.hpp"
#include <yaml-cpp/yaml.h>
#include <vector>

inline va15_shadow::Model LoadVA15ShadowModel(const std::string& path)
{
    const auto root = YAML::LoadFile(path);
    if (root["schema"].as<std::string>() != "va15_leg_kinematics_v1" ||
        root["dds_order"].as<std::vector<std::string>>() != std::vector<std::string>{"FR","FL","RR","RL"})
        throw std::invalid_argument("Unexpected leg model schema/order");
    auto vec = [](const YAML::Node& node) {
        const auto v = node.as<std::vector<double>>();
        if (v.size() != 3) throw std::invalid_argument("Invalid kinematic vector size");
        return va15_shadow::Vec(v[0],v[1],v[2]);
    };
    va15_shadow::Model model;
    model.source_sha256 = root["source_urdf_sha256"].as<std::string>();
    const auto legs = root["legs"];
    if (legs.size() != 4) throw std::invalid_argument("Expected four leg chains");
    for (int i = 0; i < 4; ++i)
    {
        const auto joints = legs[i]["joints"];
        if (joints.size() != 3) throw std::invalid_argument("Expected three joints per leg");
        for (int j = 0; j < 3; ++j)
        {
            auto& joint = model.legs[i].joints[j];
            joint.origin = vec(joints[j]["origin"]);
            joint.rpy = vec(joints[j]["rpy"]);
            joint.axis = vec(joints[j]["axis"]);
            joint.lower = joints[j]["lower"].as<double>();
            joint.upper = joints[j]["upper"].as<double>();
        }
        model.legs[i].tip = vec(legs[i]["tip"]);
        model.legs[i].radius = legs[i]["radius"].as<double>();
    }
    va15_shadow::Estimator validate(model);
    return model;
}
