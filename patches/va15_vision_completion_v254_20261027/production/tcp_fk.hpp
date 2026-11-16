#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <vector>

// Exact forward kinematics of the archived VA15 rear-mounted Z1 model.
// This is a virtual welding TCP when no gun is physically installed. It is
// not a tool calibration, odometry source, or contact/support measurement.
namespace va15_tcp {

inline constexpr std::array<int, 6> MeasuredTCPArmIndices{{8, 13, 14, 15, 16, 17}};
inline constexpr std::array<double, 6> MeasuredTCPArmLower{{
    -2.6179939, 0., -2.8797933, -1.5184364, -1.3439035, -2.7925268}};
inline constexpr std::array<double, 6> MeasuredTCPArmUpper{{
    2.6179939, 2.9670597, 0., 1.5184364, 1.3439035, 2.7925268}};

// Inputs are the measured 18-joint POLICY-order state, never actor targets.
// Failure leaves output unchanged. A small tolerance accommodates rounded
// model limit constants and feedback at the configured physical joint limit.
template<class Scalar>
inline bool MeasuredTCPBase(const std::vector<Scalar>& policy_q,
                            Eigen::Vector3d& output)
{
    if (policy_q.size() != 18) return false;
    for (const auto value : policy_q)
        if (!std::isfinite(static_cast<double>(value))) return false;

    std::array<double, 6> q;
    for (std::size_t i = 0; i < q.size(); ++i)
    {
        q[i] = static_cast<double>(policy_q[MeasuredTCPArmIndices[i]]);
        if (q[i] < MeasuredTCPArmLower[i] - 1.e-4 ||
            q[i] > MeasuredTCPArmUpper[i] + 1.e-4) return false;
    }

    // MuJoCo body poses: link00 p=(0,0,.09), quaternion wxyz=(0,0,0,1).
    // Joint axes are in each child's local frame. Apply each child offset
    // BEFORE its joint rotation; reversing that order is not this model.
    Eigen::Vector3d position(0., 0., .09);
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation(0, 0) = rotation(1, 1) = -1.;
    const std::array<Eigen::Vector3d, 6> offset{{
        {0., 0., .0585}, {0., 0., .045}, {-.35, 0., 0.},
        {.218, 0., .057}, {.07, 0., 0.}, {.0492, 0., 0.}}};
    const std::array<Eigen::Vector3d, 6> axis{{
        Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()}};
    for (std::size_t i = 0; i < q.size(); ++i)
    {
        position += rotation * offset[i];
        rotation = (rotation * Eigen::AngleAxisd(q[i], axis[i])).eval();
    }

    // link06 -> ee_site == link06 -> link_mounter -> link_gun -> dummy_target.
    // The gun's fixed Rz(pi/2) converts (0,-.03,-.62) to (.03,0,-.62),
    // giving net (.075+.03, 0, -.20-.62), NOT merely the flange position.
    position += rotation * Eigen::Vector3d(.105, 0., -.820);
    if (!position.allFinite()) return false;
    output = position;
    return true;
}

// The local-world pose must be the same valid generation used for the cached
// crack endpoint. Quaternion argument is explicitly Eigen's w,x,y,z type.
// This transform does not assert the pose is fresh; the caller owns that gate.
template<class Scalar>
inline bool MeasuredTCPWorld(const Eigen::Vector3d& base_position,
                             const Eigen::Quaterniond& base_quaternion,
                             const std::vector<Scalar>& policy_q,
                             Eigen::Vector3d& output)
{
    if (!base_position.allFinite() || !base_quaternion.coeffs().allFinite() ||
        std::fabs(base_quaternion.squaredNorm() - 1.) > .01) return false;
    Eigen::Vector3d base_tcp;
    if (!MeasuredTCPBase(policy_q, base_tcp)) return false;
    const Eigen::Vector3d result =
        base_position + base_quaternion.normalized() * base_tcp;
    if (!result.allFinite()) return false;
    output = result;
    return true;
}

// Convenience overload for va15_tcp::Pose without coupling this FK-only
// header to camera packets or visual guidance. Validity/freshness/sequence
// bounds remain enforced at the caller before using it for completion.
template<class PoseType, class Scalar>
inline bool MeasuredTCPWorld(const PoseType& pose,
                             const std::vector<Scalar>& policy_q,
                             Eigen::Vector3d& output)
{
    if (!pose.valid || !pose.ns) return false;
    return MeasuredTCPWorld(pose.p, pose.q, policy_q, output);
}

} // namespace va15_tcp
