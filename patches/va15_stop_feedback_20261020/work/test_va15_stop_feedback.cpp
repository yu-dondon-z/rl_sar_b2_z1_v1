#include "va15_stop_feedback.hpp"
#include <cassert>
#include <iostream>

int main()
{
    // Synthetic values only; these are deliberately NOT a B2 calibration.
    VA15StopFeedbackTracker::Calibration calibration;
    calibration.foot_validated = calibration.sport_validated = true;
    calibration.id = "SYNTHETIC_TEST_ONLY";
    calibration.zero = {{100, 200, 300, 400}};
    calibration.direction = {{1, -1, -1, 1}};
    calibration.on.fill(20); calibration.off.fill(10);
    const std::array<float, 4> unloaded{{100, 200, 300, 400}};
    const std::array<float, 4> loaded{{130, 170, 270, 430}};
    VA15StopFeedbackTracker tracker;
    tracker.Configure(calibration);
    auto feed = [&](const std::array<float, 4>& raw, int start, int n) {
        for (int i = start; i < start + n; ++i) {
            tracker.UpdateLow(raw, 100 + i * 5, 1. + i * .005);
            tracker.UpdateSport(.52f, {{0, 0, 0}}, 1000000 + i * 5000000ull, 1. + i * .005, true);
        }
    };
    feed(unloaded, 0, 20);
    assert(VA15StopFeedbackValid(tracker.Snapshot(1.095)));
    assert(!VA15RealStopSettled(tracker.Snapshot(1.095), .01f, .01f, .01f, .48f, .15f));
    feed(loaded, 20, 20);
    const auto good = tracker.Snapshot(1.195);
    assert(VA15RealStopSettled(good, .01f, .01f, .01f, .48f, .15f));
    assert(!VA15RealStopSettled(good, .01f, .01f, .01f, 0.f, .15f));
    assert(!VA15RealStopSettled(good, .2f, .01f, .01f, .48f, .15f));
    assert(!VA15RealStopSettled(good, .01f, .2f, .01f, .48f, .15f));
    assert(!VA15RealStopSettled(good, .01f, .01f, .6f, .48f, .15f));
    auto invalid = good; invalid.planar_speed = .1f;
    assert(!VA15RealStopSettled(invalid, 0, 0, 0, .48f, .15f));
    invalid = good; invalid.base_height = .3f;
    assert(!VA15RealStopSettled(invalid, 0, 0, 0, .48f, .15f));
    invalid = good; invalid.planar_speed = std::numeric_limits<float>::quiet_NaN();
    assert(!VA15StopFeedbackValid(invalid));
    assert(!VA15StopFeedbackValid(VA15StopFeedback{}));
    assert(!VA15StopFeedbackValid(tracker.Snapshot(1.5)));
    // Replaying a packet cannot refresh either sensor.
    tracker.UpdateLow(loaded, 295, 1.5);
    tracker.UpdateSport(.52f, {{0, 0, 0}}, 196000000ull, 1.5, true);
    assert(!VA15StopFeedbackValid(tracker.Snapshot(1.5)));
    // FR release maps to policy contact index 1, not FL/RL/RR.
    feed(loaded, 110, 20);
    auto released = loaded; released[0] = unloaded[0];
    feed(released, 130, 20);
    const auto one_up = tracker.Snapshot(1.745);
    assert((one_up.contacts == std::array<bool, 4>{{true, false, true, true}}));
    assert(!VA15RealStopSettled(one_up, 0, 0, 0, .48f, .15f));
    // A contradictory RL signal stays negative; never abs() or flip direction.
    auto contradiction = loaded; contradiction[3] = 370;
    feed(contradiction, 150, 20);
    assert(!tracker.Snapshot(1.845).contacts[2]);
    tracker.UpdateSport(0, {{0, 0, 0}}, 999999999ull, 1.85, true);
    assert(!tracker.Snapshot(1.85).height_valid);
    tracker.UpdateSport(.52f, {{0, 0, 0}}, 1000000000ull, 1.855, false);
    assert(!tracker.Snapshot(1.855).velocity_valid);
    tracker.UpdateSport(.52f, {{0, 0, 0}}, 3, 1.86, true);
    assert(!tracker.Snapshot(1.86).velocity_valid);
    calibration.foot_validated = false;
    tracker.Configure(calibration);
    feed(loaded, 0, 30);
    assert(!tracker.Snapshot(1.145).contacts_valid);
    assert(std::string(tracker.Snapshot(1.145).unavailable_reason) == "foot calibration not validated");
    calibration.foot_validated = true; calibration.sport_validated = false;
    tracker.Configure(calibration); feed(loaded, 0, 30);
    assert(tracker.Snapshot(1.145).contacts_valid);
    assert(!tracker.Snapshot(1.145).velocity_valid);
    calibration.sport_validated = true; calibration.direction[0] = 0;
    bool rejected = false;
    try { tracker.Configure(calibration); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    std::cout << "PASS: real stop feedback validity, signed contact mapping, debounce, "
                 "duplicates/staleness, contradictory RL, invalid/reset sport state, no calibration fallback\n";
}
