#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

// Offline contract model for the SDK behavior audited in
// outputs/startup_analysis/Z1_GAIN_ORDER_AUDIT.md. This deliberately does
// not link to, initialize, or call the real Z1 SDK.
class FakeLowCmd {
public:
    std::array<double, 6> kp{};
    std::array<double, 6> kd{};

    void setControlGain(const std::array<double, 6>& next_kp,
                        const std::array<double, 6>& next_kd) {
        kp = next_kp;
        kd = next_kd;
    }

    void setControlGain() {
        kp = {20., 30., 30., 20., 15., 10.};
        kd = {2000., 2000., 2000., 2000., 2000., 2000.};
    }
};

class FakeArm {
public:
    enum class Fsm { Passive, LowCmd };

    bool setFsm(Fsm next) {
        // Correct the old fake's incomplete branch: the actual audited SDK
        // refuses LOWCMD unless the RECEIVED state is PASSIVE, and requests
        // PASSIVE without pretending that its reply already arrived.
        if (next == Fsm::LowCmd && state_ != Fsm::Passive) {
            requested = Fsm::Passive;
            return false;
        }
        if (next == Fsm::LowCmd && state_ == Fsm::Passive) {
            // Matches the audited LOWCMD transition side effect.
            lowcmd.setControlGain();
        }
        requested = state_ = next; // explicit immediate fake ACK on success
        return true;
    }

    FakeLowCmd lowcmd;
    Fsm requested = Fsm::Passive;
    Fsm received() const { return state_; }

private:
    Fsm state_ = Fsm::Passive;
};

static bool Same(const std::array<double, 6>& a,
                 const std::array<double, 6>& b) {
    for (std::size_t i = 0; i < a.size(); ++i)
        if (!std::isfinite(a[i]) || !std::isfinite(b[i]) ||
            std::fabs(a[i] - b[i]) > 1.e-12) return false;
    return true;
}

int main() {
    const std::array<double, 6> scaled_kp = {7., 10.5, 10.5, 7., 5.25, 3.5};
    const std::array<double, 6> sdk_default_kp = {20., 30., 30., 20., 15., 10.};
    const std::array<double, 6> sdk_default_kd = {2000., 2000., 2000., 2000., 2000., 2000.};

    // V251 scales Kp only and leaves Kd unchanged. The fake defaults are
    // audited SDK buffer constants, not physical motor gain units.
    FakeArm historical;
    for (int cycle = 0; cycle < 3; ++cycle) {
        assert(historical.setFsm(FakeArm::Fsm::Passive));
        historical.lowcmd.setControlGain(scaled_kp, sdk_default_kd);
        assert(historical.setFsm(FakeArm::Fsm::LowCmd));
        assert(Same(historical.lowcmd.kp, sdk_default_kp));
        assert(Same(historical.lowcmd.kd, sdk_default_kd));
    }

    // A separate post-transition write is the only operation that makes the
    // requested values persistent in this contract model. This test records
    // the requirement; it does not authorize changing the real robot order.
    FakeArm post_transition_example;
    for (int cycle = 0; cycle < 3; ++cycle) {
        assert(post_transition_example.setFsm(FakeArm::Fsm::Passive));
        assert(post_transition_example.setFsm(FakeArm::Fsm::LowCmd));
        post_transition_example.lowcmd.setControlGain(scaled_kp, sdk_default_kd);
        assert(Same(post_transition_example.lowcmd.kp, scaled_kp));
        assert(Same(post_transition_example.lowcmd.kd, sdk_default_kd));
    }

    FakeArm already_low;
    assert(already_low.setFsm(FakeArm::Fsm::LowCmd));
    assert(!already_low.setFsm(FakeArm::Fsm::LowCmd));
    assert(already_low.requested == FakeArm::Fsm::Passive);
    assert(already_low.received() == FakeArm::Fsm::LowCmd);
    assert(already_low.setFsm(FakeArm::Fsm::Passive));
    assert(already_low.setFsm(FakeArm::Fsm::LowCmd));

    std::cout << "PASS offline Z1 LOWCMD gain-order contract; no SDK or robot I/O\n";
}
