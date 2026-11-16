#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

// Flat-ground, rigid-foot/no-slip HYPOTHESES, not a contact detector or EKF.
// A robot suspended in air at the same joint pose produces the same geometry.
// This module must never create foot contact or permission to move/lower.
namespace va15_shadow
{
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;

struct Joint
{
    Vec origin = Vec::Zero(), rpy = Vec::Zero(), axis = Vec::UnitX();
    double lower = 0., upper = 0.;
};
struct Leg
{
    std::array<Joint, 3> joints;
    Vec tip = Vec::Zero(); // last moving-joint frame to spherical foot center
    double radius = 0.;
};
struct Model
{
    std::array<Leg, 4> legs; // FR FL RR RL, matching native DDS motor triples
    std::string source_sha256;
};
struct Foot
{
    Vec position_base = Vec::Zero();
    Mat jacobian_base = Mat::Zero();
    Vec no_slip_velocity_world = Vec::Zero();
    double flat_ground_height = std::numeric_limits<double>::quiet_NaN();
};
struct Sample
{
    std::array<double, 12> q{}, dq{};
    std::array<double, 4> quaternion{{1., 0., 0., 0.}}; // world <- base, wxyz
    std::array<double, 3> gyro{}; // body axes, rad/s
    // Supplied only by an independently validated contact source, never FK.
    bool contact_evidence_valid = false;
    std::array<bool, 4> contact_dds{};
};
struct Result
{
    bool input_valid = false, fresh = false, sequence_valid = false;
    bool hypothesis_consistent = false, conditional_stance_valid = false;
    // Deliberately immutable authorization boundary, independent of YAML.
    static constexpr bool control_authorized = false;
    std::array<Foot, 4> feet;
    double hypothesis_height = std::numeric_limits<double>::quiet_NaN();
    Vec hypothesis_velocity_world = Vec::Constant(std::numeric_limits<double>::quiet_NaN());
    double height_spread = std::numeric_limits<double>::quiet_NaN();
    double velocity_spread = std::numeric_limits<double>::quiet_NaN();
    double conditional_height = std::numeric_limits<double>::quiet_NaN();
    Vec conditional_velocity_world = Vec::Constant(std::numeric_limits<double>::quiet_NaN());
    int accepted_contacts = 0;
    const char* reason = "no input";
};

inline Mat Rpy(const Vec& angles)
{
    return (Eigen::AngleAxisd(angles[2], Vec::UnitZ()) *
            Eigen::AngleAxisd(angles[1], Vec::UnitY()) *
            Eigen::AngleAxisd(angles[0], Vec::UnitX())).toRotationMatrix();
}

class Estimator
{
public:
    explicit Estimator(const Model& model) : model_(model)
    {
        if (model_.source_sha256.size() != 64) throw std::invalid_argument("Missing model provenance hash");
        for (const auto& leg : model_.legs)
        {
            if (!leg.tip.allFinite() || !std::isfinite(leg.radius) || leg.radius <= 0. || leg.radius > .1)
                throw std::invalid_argument("Invalid spherical foot geometry");
            for (const auto& joint : leg.joints)
                if (!joint.origin.allFinite() || !joint.rpy.allFinite() || !joint.axis.allFinite() ||
                    std::fabs(joint.axis.norm() - 1.) > 1.e-6 || !std::isfinite(joint.lower) ||
                    !std::isfinite(joint.upper) || joint.lower >= joint.upper)
                    throw std::invalid_argument("Invalid leg joint geometry/limits");
        }
    }

    Foot Forward(int leg_index, const std::array<double, 3>& q) const
    {
        const auto& leg = model_.legs.at(leg_index);
        Mat rotation = Mat::Identity();
        Vec position = Vec::Zero();
        std::array<Vec, 3> origins, axes;
        for (int i = 0; i < 3; ++i)
        {
            const auto& joint = leg.joints[i];
            if (!std::isfinite(q[i]) || q[i] < joint.lower - 1.e-5 || q[i] > joint.upper + 1.e-5)
                throw std::invalid_argument("Joint pose outside model limits");
            position += rotation * joint.origin;
            rotation *= Rpy(joint.rpy);
            origins[i] = position;
            axes[i] = rotation * joint.axis;
            rotation *= Eigen::AngleAxisd(q[i], joint.axis).toRotationMatrix();
        }
        Foot foot;
        foot.position_base = position + rotation * leg.tip;
        for (int i = 0; i < 3; ++i)
            foot.jacobian_base.col(i) = axes[i].cross(foot.position_base - origins[i]);
        return foot;
    }

    Result Evaluate(const Sample& sample) const
    {
        Result result;
        for (double value : sample.dq)
            if (!std::isfinite(value)) { result.reason = "invalid joint velocity"; return result; }
        for (double value : sample.quaternion)
            if (!std::isfinite(value)) { result.reason = "invalid IMU orientation"; return result; }
        const Vec gyro(sample.gyro[0], sample.gyro[1], sample.gyro[2]);
        Eigen::Quaterniond quaternion(sample.quaternion[0], sample.quaternion[1],
                                     sample.quaternion[2], sample.quaternion[3]);
        if (!gyro.allFinite() || quaternion.squaredNorm() < .9 || quaternion.squaredNorm() > 1.1)
        { result.reason = "invalid IMU norm/gyro"; return result; }
        quaternion.normalize();
        const Mat world_from_base = quaternion.toRotationMatrix();
        try
        {
            for (int i = 0; i < 4; ++i)
            {
                auto& foot = result.feet[i];
                foot = Forward(i, {{sample.q[i*3], sample.q[i*3+1], sample.q[i*3+2]}});
                const Vec dq(sample.dq[i*3], sample.dq[i*3+1], sample.dq[i*3+2]);
                // v_base^W = -R_WB (J(q) dq + omega_B cross p_foot^B).
                // Assumes the foot-center contact is stationary: rolling,
                // deformation, slip or unknown contact invalidate authority.
                foot.no_slip_velocity_world = -world_from_base *
                    (foot.jacobian_base * dq + gyro.cross(foot.position_base));
                foot.flat_ground_height = model_.legs[i].radius -
                    (world_from_base * foot.position_base)[2];
                if (!foot.position_base.allFinite() || !foot.jacobian_base.allFinite() ||
                    !foot.no_slip_velocity_world.allFinite() || !std::isfinite(foot.flat_ground_height))
                { result.reason = "nonfinite forward kinematics"; return result; }
            }
        }
        catch (const std::invalid_argument&)
        { result.reason = "joint pose outside model limits"; return result; }
        result.input_valid = true;
        std::array<int, 4> indices{{0,1,2,3}};
        Aggregate(result, indices, 4, result.hypothesis_height, result.hypothesis_velocity_world,
                  result.height_spread, result.velocity_spread);
        result.hypothesis_consistent = result.height_spread <= .030 && result.velocity_spread <= .150;
        if (!sample.contact_evidence_valid)
        { result.reason = "contact UNKNOWN; geometry hypotheses only"; return result; }
        int count = 0;
        for (int i = 0; i < 4; ++i) if (sample.contact_dds[i]) indices[count++] = i;
        result.accepted_contacts = count;
        if (count < 2) { result.reason = "fewer than two validated stance feet"; return result; }
        double spread_height, spread_velocity;
        Aggregate(result, indices, count, result.conditional_height, result.conditional_velocity_world,
                  spread_height, spread_velocity);
        result.conditional_stance_valid = spread_height <= .030 && spread_velocity <= .150;
        result.reason = result.conditional_stance_valid
            ? "conditional no-slip estimate only; NOT motion authorization"
            : "inconsistent stance feet: slope/slip/model/contact ambiguity";
        return result;
    }

private:
    static double Median(std::array<double, 4> values, int count)
    {
        std::sort(values.begin(), values.begin()+count);
        return count % 2 ? values[count/2] : .5*(values[count/2-1]+values[count/2]);
    }
    static void Aggregate(const Result& result, const std::array<int, 4>& indices, int count,
                          double& height, Vec& velocity, double& height_spread, double& velocity_spread)
    {
        std::array<double, 4> values{};
        for (int j = 0; j < count; ++j) values[j] = result.feet[indices[j]].flat_ground_height;
        height = Median(values, count);
        auto bounds = std::minmax_element(values.begin(), values.begin()+count);
        height_spread = *bounds.second - *bounds.first;
        for (int axis = 0; axis < 3; ++axis)
        {
            for (int j = 0; j < count; ++j) values[j] = result.feet[indices[j]].no_slip_velocity_world[axis];
            velocity[axis] = Median(values, count);
        }
        velocity_spread = 0.;
        for (int j = 0; j < count; ++j)
            velocity_spread = std::max(velocity_spread,
                (result.feet[indices[j]].no_slip_velocity_world - velocity).norm());
    }
    Model model_;
};

class Tracker
{
public:
    explicit Tracker(const Model& model) : estimator_(model) {}
    void Update(const Sample& sample, std::uint32_t tick, double now)
    {
        if (!std::isfinite(now)) { result_ = {}; have_ = false; return; }
        if (have_ && now < received_) { result_ = {}; have_ = false; return; }
        if (have_ && tick == tick_) return;
        const auto delta = static_cast<std::uint32_t>(tick - tick_);
        const bool continuous = have_ && delta > 0 && delta <= 50 && now - received_ <= .100;
        result_ = estimator_.Evaluate(sample);
        result_.sequence_valid = continuous;
        tick_ = tick; received_ = now; have_ = true;
    }
    Result Snapshot(double now) const
    {
        auto result = result_;
        result.fresh = have_ && std::isfinite(now) && now >= received_ && now - received_ <= .100;
        if (!result.fresh || !result.sequence_valid)
        {
            result.conditional_stance_valid = false;
            result.reason = "missing/stale/reset lowstate sequence";
        }
        return result;
    }
private:
    Estimator estimator_;
    Result result_;
    bool have_ = false;
    std::uint32_t tick_ = 0;
    double received_ = 0.;
};
} // namespace va15_shadow
