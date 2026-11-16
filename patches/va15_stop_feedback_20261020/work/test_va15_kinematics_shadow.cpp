#include "va15_kinematics_model_io.hpp"
#include <cassert>
#include <iostream>

int main(int argc, char** argv)
{
    if (argc != 3) throw std::runtime_error("model and MuJoCo golden paths required");
    auto model = LoadVA15ShadowModel(argv[1]);
    const auto golden = YAML::LoadFile(argv[2]);
    assert(model.source_sha256 == golden["source_urdf_sha256"].as<std::string>());
    va15_shadow::Estimator estimator(model);
    double position_error = 0., jacobian_error = 0., velocity_error = 0., height_error = 0.;
    for (const auto& item : golden["cases"])
    {
        va15_shadow::Sample sample;
        const auto copy = [](const YAML::Node& node, auto& out) {
            const auto values = node.as<std::vector<double>>();
            assert(values.size() == out.size());
            std::copy(values.begin(), values.end(), out.begin());
        };
        copy(item["q"], sample.q); copy(item["dq"], sample.dq);
        copy(item["quaternion"], sample.quaternion); copy(item["gyro"], sample.gyro);
        auto result = estimator.Evaluate(sample);
        assert(result.input_valid && !result.control_authorized && !result.conditional_stance_valid);
        for (int leg = 0; leg < 4; ++leg)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                position_error = std::max(position_error, std::fabs(result.feet[leg].position_base[axis] -
                    item["foot_position_base"][leg][axis].as<double>()));
                velocity_error = std::max(velocity_error, std::fabs(result.feet[leg].no_slip_velocity_world[axis] -
                    item["no_slip_velocity_world"][leg][axis].as<double>()));
                for (int j = 0; j < 3; ++j)
                    jacobian_error = std::max(jacobian_error, std::fabs(result.feet[leg].jacobian_base(axis,j) -
                        item["jacobian_base"][leg][axis][j].as<double>()));
            }
            height_error = std::max(height_error, std::fabs(result.feet[leg].flat_ground_height -
                item["flat_ground_height"][leg].as<double>()));
        }
        // Independent central finite difference of each FK joint.
        for (int leg = 0; leg < 4; ++leg) for (int j = 0; j < 3; ++j)
        {
            std::array<double, 3> q{{sample.q[leg*3],sample.q[leg*3+1],sample.q[leg*3+2]}};
            q[j] += 1.e-6; const auto plus = estimator.Forward(leg,q).position_base;
            q[j] -= 2.e-6; const auto minus = estimator.Forward(leg,q).position_base;
            assert(((plus-minus)/2.e-6-result.feet[leg].jacobian_base.col(j)).norm() < 1.e-8);
        }
    }
    assert(position_error < 1.e-11 && jacobian_error < 1.e-11 && velocity_error < 1.e-11 && height_error < 1.e-11);
    va15_shadow::Sample sample;
    for (int i = 0; i < 4; ++i) { sample.q[i*3+1]=.8; sample.q[i*3+2]=-1.6; }
    auto result = estimator.Evaluate(sample);
    assert(result.input_valid && result.hypothesis_consistent && !result.conditional_stance_valid);
    assert(!result.control_authorized && result.accepted_contacts == 0);
    assert(std::fabs(result.hypothesis_height - (.7*std::cos(.8)+.032)) < 1.e-10);
    // Identical suspended geometry cannot certify ground contact.
    sample.contact_dds.fill(true);
    assert(!estimator.Evaluate(sample).conditional_stance_valid);
    sample.contact_evidence_valid = true;
    result = estimator.Evaluate(sample);
    assert(result.conditional_stance_valid && result.accepted_contacts == 4 && !result.control_authorized);
    sample.dq[0] = 2.;
    assert(!estimator.Evaluate(sample).conditional_stance_valid);
    sample.dq[0] = 0.; sample.q[1] = 1.4;
    assert(!estimator.Evaluate(sample).conditional_stance_valid);
    sample.contact_dds = {{false,true,true,true}};
    assert(estimator.Evaluate(sample).conditional_stance_valid);
    sample.contact_dds = {{false,true,false,false}};
    assert(!estimator.Evaluate(sample).conditional_stance_valid);
    sample.contact_evidence_valid = false;
    sample.q[1]=.8;
    va15_shadow::Tracker tracker(model);
    tracker.Update(sample,100,1.);
    assert(!tracker.Snapshot(1.).sequence_valid);
    tracker.Update(sample,105,1.005);
    assert(tracker.Snapshot(1.005).fresh && tracker.Snapshot(1.005).sequence_valid);
    assert(!tracker.Snapshot(1.2).fresh);
    tracker.Update(sample,105,1.2);
    assert(!tracker.Snapshot(1.2).fresh); // duplicates cannot refresh age
    tracker.Update(sample,3,1.205);
    assert(!tracker.Snapshot(1.205).sequence_valid);
    tracker.Update(sample,8,1.210);
    assert(tracker.Snapshot(1.210).sequence_valid);
    auto invalid=sample;invalid.quaternion.fill(0);
    assert(!estimator.Evaluate(invalid).input_valid);
    invalid=sample;invalid.dq[0]=std::numeric_limits<double>::quiet_NaN();
    assert(!estimator.Evaluate(invalid).input_valid);
    invalid=sample;invalid.q[2]=0;
    assert(!estimator.Evaluate(invalid).input_valid);
    invalid=sample;invalid.quaternion[0]=2;
    assert(!estimator.Evaluate(invalid).input_valid);
    std::cout << "PASS: " << golden["cases"].size() << " independent MuJoCo cases; max errors: position="
              << position_error << " Jacobian=" << jacobian_error << " velocity=" << velocity_error
              << " height=" << height_error << "; finite differences, missing contact, "
                 "suspended ambiguity, slip, unknown/stale/reset/NaN checks. Control authorization is always false.\n";
}
