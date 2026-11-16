#pragma once
#include "near_visual_guide.hpp"

namespace va15_tcp_priority {
// V256's live-vision contract. The tolerant guide may reuse a previous frame
// for other deployments; an explicit rejection must stop THIS whole session.
// No timestamps, support evidence, gains or target positions are altered here.
inline va15_visual::Output Tick(va15_visual::Guide& guide,
                               const va15_visual::Packet& incoming,
                               std::uint64_t now, double dt,
                               bool healthy, bool stop, bool centered,
                               bool local_pose_valid, bool policy_ready,
                               bool& active, bool& pending_start) {
    if (active) {
        if (!va15_visual::Valid(incoming, now)) {
            guide.Stop("visual packet rejected or expired; live TCP session stopped");
            // Six new associated frames are required after a rejected frame.
            // The retained capture is never relabelled as a fresh observation.
            guide.qualified = 0;
        } else if (!local_pose_valid) {
            guide.Stop("local pose stale/unreliable; live TCP session stopped");
        } else if (!policy_ready) {
            guide.Stop("policy standby lost; live TCP session stopped");
        }
    }
    const auto output = guide.Tick(now, dt, healthy, stop, centered);
    if (active && (output.stop || guide.stage != va15_visual::Guide::Stage::Running)) {
        active = false;
        pending_start = false;
        return {0., 0., true};
    }
    return output;
}
} // namespace va15_tcp_priority
