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

struct ArmGeometry {
    Eigen::Vector3d position=Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
    Eigen::Matrix<double,3,6> jacobian=Eigen::Matrix<double,3,6>::Zero();
};

// Analytic spatial position Jacobian at the SAME virtual gun tip as FK.
// There is no flange/tool-frame substitution in the Cartesian controller.
inline bool ArmTCPGeometry(const std::array<double,6>& q,ArmGeometry& result) {
    for(int i=0;i<6;++i)
        if(!std::isfinite(q[i])||q[i]<MeasuredTCPArmLower[i]-1.e-4||
           q[i]>MeasuredTCPArmUpper[i]+1.e-4)return false;
    Eigen::Vector3d position(0.,0.,.09);
    Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
    rotation(0,0)=rotation(1,1)=-1.;
    const std::array<Eigen::Vector3d,6> offset{{
        {0.,0.,.0585},{0.,0.,.045},{-.35,0.,0.},
        {.218,0.,.057},{.07,0.,0.},{.0492,0.,0.}}};
    const std::array<Eigen::Vector3d,6> axis{{
        Eigen::Vector3d::UnitZ(),Eigen::Vector3d::UnitY(),Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitY(),Eigen::Vector3d::UnitZ(),Eigen::Vector3d::UnitX()}};
    std::array<Eigen::Vector3d,6> origin,base_axis;
    for(int i=0;i<6;++i) {
        position+=rotation*offset[i];origin[i]=position;base_axis[i]=rotation*axis[i];
        rotation=(rotation*Eigen::AngleAxisd(q[i],axis[i])).eval();
    }
    position+=rotation*Eigen::Vector3d(.105,0.,-.820);
    ArmGeometry candidate;candidate.position=position;candidate.rotation=rotation;
    for(int i=0;i<6;++i)candidate.jacobian.col(i)=base_axis[i].cross(position-origin[i]);
    if(!candidate.position.allFinite()||!candidate.jacobian.allFinite())return false;
    result=candidate;return true;
}

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

    ArmGeometry geometry;
    if(!ArmTCPGeometry(q,geometry))return false;
    output = geometry.position;
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
