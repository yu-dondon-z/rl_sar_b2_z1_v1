#include "production/tcp_fk.hpp"
#include <cassert>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace {
struct TestPose {
    std::uint64_t ns = 1;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    bool valid = true;
};

// Deliberately tiny model-only CLI: six measured Z1 angles, then base x y z
// and quaternion w x y z, one sample per line. No robot SDK is linked.
int CLI()
{
    std::string line;
    std::cout << std::setprecision(17);
    while (std::getline(std::cin, line))
    {
        std::istringstream row(line);
        std::array<double, 13> values;
        bool parsed = true;
        for (auto& value : values)
            if (!(row >> value)) { parsed = false; break; }
        std::string extra;
        if (row >> extra) parsed = false;
        if (!parsed) { std::cout << "INVALID\n"; continue; }
        std::vector<double> policy_q(18, 0.);
        for (int i = 0; i < 6; ++i)
            policy_q[va15_tcp::MeasuredTCPArmIndices[i]] = values[i];
        const Eigen::Vector3d position(values[6], values[7], values[8]);
        const Eigen::Quaterniond quaternion(values[9], values[10], values[11], values[12]);
        Eigen::Vector3d base, world;
        if (!va15_tcp::MeasuredTCPBase(policy_q, base) ||
            !va15_tcp::MeasuredTCPWorld(position, quaternion, policy_q, world))
        { std::cout << "INVALID\n"; continue; }
        std::cout << base.transpose() << ' ' << world.transpose() << '\n';
    }
    return 0;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc == 2 && std::string(argv[1]) == "--cli") return CLI();
    if (argc != 1) return 1;
    std::vector<double> policy_q(18, 0.);
    Eigen::Vector3d base, world;
    assert(va15_tcp::MeasuredTCPBase(policy_q, base));
    assert((base - Eigen::Vector3d(-.0922, 0., -.5695)).norm() < 1.e-12);
    TestPose pose;
    pose.p = Eigen::Vector3d(3., -2., .52);
    pose.q = Eigen::Quaterniond(Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(-.2, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(.15, Eigen::Vector3d::UnitX()));
    assert(va15_tcp::MeasuredTCPWorld(pose, policy_q, world));
    assert((world - pose.p - pose.q * base).norm() < 1.e-12);
    const Eigen::Vector3d unchanged = world;
    auto invalid = policy_q;
    invalid[8] = std::numeric_limits<double>::quiet_NaN();
    assert(!va15_tcp::MeasuredTCPBase(invalid, world));
    assert(world == unchanged);
    invalid = policy_q; invalid[0] = std::numeric_limits<double>::infinity();
    assert(!va15_tcp::MeasuredTCPWorld(pose, invalid, world));
    invalid = policy_q; invalid[13] = -0.1;
    assert(!va15_tcp::MeasuredTCPBase(invalid, world));
    invalid = policy_q; invalid.pop_back();
    assert(!va15_tcp::MeasuredTCPBase(invalid, world));
    pose.q.coeffs() *= 2.;
    assert(!va15_tcp::MeasuredTCPWorld(pose, policy_q, world));
    pose.q = Eigen::Quaterniond::Identity(); pose.valid = false;
    assert(!va15_tcp::MeasuredTCPWorld(pose, policy_q, world));
    pose.valid = true; pose.ns = 0;
    assert(!va15_tcp::MeasuredTCPWorld(pose, policy_q, world));
    assert(world == unchanged);
    std::cout << "PASS measured 18D policy-order virtual TCP FK, full pose transform, invalid input rejection; no SDK/robot initialized\n";
    return 0;
}
