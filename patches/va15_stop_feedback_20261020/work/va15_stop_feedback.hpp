#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

// Deployment diagnostics only. None of these fields is an actor input.
// Raw DDS bridge counts are NOT Newtons or binary contacts. No archived
// calibration is selected implicitly, and missing data is never a zero speed.
struct VA15StopFeedback
{
    bool lowstate_fresh = false;
    bool sport_fresh = false;
    bool contacts_valid = false;
    bool height_valid = false;
    bool velocity_valid = false;
    float base_height = std::numeric_limits<float>::quiet_NaN();
    float planar_speed = std::numeric_limits<float>::quiet_NaN();
    std::array<float, 4> raw_dds{}; // FR FL RR RL
    std::array<bool, 4> contacts{}; // FL FR RL RR
    const char* unavailable_reason = "no real support feedback";
};

class VA15StopFeedbackTracker
{
public:
    struct Calibration
    {
        bool foot_validated = false;
        bool sport_validated = false;
        std::string id;
        std::array<float, 4> zero{}, direction{}, on{}, off{};
    };

    void Configure(const Calibration& value)
    {
        if (value.foot_validated)
        {
            if (value.id.empty()) throw std::invalid_argument("Missing foot calibration ID");
            for (int i = 0; i < 4; ++i)
                if (!std::isfinite(value.zero[i]) ||
                    !std::isfinite(value.direction[i]) ||
                    std::fabs(std::fabs(value.direction[i]) - 1.f) > 1.e-6f ||
                    !std::isfinite(value.on[i]) || !std::isfinite(value.off[i]) ||
                    value.off[i] <= 0.f || value.on[i] <= value.off[i])
                    throw std::invalid_argument("Invalid signed foot bridge calibration");
        }
        *this = VA15StopFeedbackTracker();
        calibration_ = value;
    }

    void UpdateLow(const std::array<float, 4>& raw, std::uint32_t tick, double now)
    {
        if (!std::isfinite(now)) { ResetContacts(); have_low_ = false; return; }
        if (have_low_ && (now < low_time_ || now - low_time_ > .100)) ResetContacts();
        // Repeated DDS packets do not refresh sensor age or debounce windows.
        if (have_low_ && tick == tick_) return;
        if (have_low_ && static_cast<std::uint32_t>(tick - tick_) > 1000u) ResetContacts();
        have_low_ = true;
        low_time_ = now;
        tick_ = tick;
        raw_ = raw;
        for (float v : raw)
            if (!std::isfinite(v)) { ResetContacts(); have_low_ = false; return; }
        if (!calibration_.foot_validated) return;
        for (int i = 0; i < 4; ++i) samples_[i][next_] = raw[i];
        next_ = (next_ + 1) % 5;
        count_ = std::min(count_ + 1, 5);
        if (count_ < 5) return;
        contacts_valid_ = true;
        for (int i = 0; i < 4; ++i)
        {
            auto sorted = samples_[i];
            std::sort(sorted.begin(), sorted.end());
            const float signal = calibration_.direction[i] * (sorted[2] - calibration_.zero[i]);
            const bool change = contact_dds_[i]
                ? signal < calibration_.off[i] : signal >= calibration_.on[i];
            if (!change) { change_time_[i] = -1.; continue; }
            if (change_time_[i] < 0) change_time_[i] = now;
            if (now - change_time_[i] + 1.e-12 >= (contact_dds_[i] ? .030 : .020))
            {
                contact_dds_[i] = !contact_dds_[i];
                change_time_[i] = -1.;
            }
        }
    }

    void UpdateSport(float height, const std::array<float, 3>& velocity,
                     std::uint64_t stamp, double now, bool error_free)
    {
        if (!std::isfinite(now) || stamp == 0 || !error_free)
        { sport_values_valid_ = false; return; }
        if (have_sport_ && stamp == sport_stamp_) return;
        const bool forward = !have_sport_ || stamp > sport_stamp_;
        have_sport_ = true;
        sport_stamp_ = stamp;
        sport_time_ = now;
        height_ = height;
        speed_ = std::hypot(velocity[0], velocity[1]);
        sport_values_valid_ = forward && std::isfinite(height) && height > .05f && height < 1.5f;
        for (float v : velocity) sport_values_valid_ = sport_values_valid_ && std::isfinite(v);
    }

    VA15StopFeedback Snapshot(double now) const
    {
        VA15StopFeedback value;
        value.raw_dds = raw_;
        value.lowstate_fresh = std::isfinite(now) && have_low_ && now >= low_time_ && now - low_time_ <= .100;
        value.sport_fresh = std::isfinite(now) && have_sport_ && now >= sport_time_ && now - sport_time_ <= .150;
        value.base_height = height_;
        value.planar_speed = speed_;
        value.contacts_valid = value.lowstate_fresh && calibration_.foot_validated && contacts_valid_;
        value.height_valid = value.sport_fresh && calibration_.sport_validated && sport_values_valid_;
        value.velocity_valid = value.height_valid;
        if (value.contacts_valid)
            value.contacts = {{contact_dds_[1], contact_dds_[0], contact_dds_[3], contact_dds_[2]}};
        if (!value.lowstate_fresh) value.unavailable_reason = "lowstate missing/stale";
        else if (!calibration_.foot_validated) value.unavailable_reason = "foot calibration not validated";
        else if (!value.contacts_valid) value.unavailable_reason = "foot contact filter not ready";
        else if (!value.sport_fresh) value.unavailable_reason = "sport state missing/stale";
        else if (!calibration_.sport_validated) value.unavailable_reason = "sport height/velocity not validated in low-level mode";
        else if (!value.height_valid) value.unavailable_reason = "invalid/reset sport height/velocity";
        else value.unavailable_reason = "none";
        return value;
    }

private:
    void ResetContacts()
    {
        contacts_valid_ = false;
        contact_dds_.fill(false);
        change_time_.fill(-1.);
        count_ = next_ = 0;
    }
    Calibration calibration_;
    bool have_low_ = false, have_sport_ = false, contacts_valid_ = false, sport_values_valid_ = false;
    std::uint32_t tick_ = 0;
    std::uint64_t sport_stamp_ = 0;
    double low_time_ = 0., sport_time_ = 0.;
    float height_ = std::numeric_limits<float>::quiet_NaN();
    float speed_ = std::numeric_limits<float>::quiet_NaN();
    int count_ = 0, next_ = 0;
    std::array<float, 4> raw_{};
    std::array<std::array<float, 5>, 4> samples_{};
    std::array<bool, 4> contact_dds_{};
    std::array<double, 4> change_time_{{-1., -1., -1., -1.}};
};

inline bool VA15StopFeedbackValid(const VA15StopFeedback& value)
{
    return value.lowstate_fresh && value.sport_fresh && value.contacts_valid &&
        value.height_valid && value.velocity_valid && std::isfinite(value.base_height) &&
        std::isfinite(value.planar_speed) && value.base_height > 0.f && value.planar_speed >= 0.f;
}

inline bool VA15RealStopSettled(const VA15StopFeedback& value, float tilt,
                               float gyro, float joint_speed, float min_height, float max_tilt)
{
    return VA15StopFeedbackValid(value) &&
        std::all_of(value.contacts.begin(), value.contacts.end(), [](bool v) { return v; }) &&
        std::isfinite(tilt) && std::isfinite(gyro) && std::isfinite(joint_speed) &&
        std::isfinite(min_height) && min_height > 0.f && std::isfinite(max_tilt) && max_tilt > 0.f &&
        tilt >= 0.f && gyro >= 0.f && joint_speed >= 0.f &&
        value.base_height >= min_height && tilt <= max_tilt &&
        value.planar_speed < .04f && gyro < .18f && joint_speed < .55f;
}
