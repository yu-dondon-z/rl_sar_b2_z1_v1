# RGB endpoint evidence (offline candidate, 2026-10-25)

This is per-frame evidence, not an endpoint completion decision or robot safety
qualification. No SSH, camera RPC, DDS, Z1 or B2 control was run for these tests.

## Feature contract

`LongTracker.detect()` adds these finite JSON values without changing any
existing near/look/long-path validity gate:

| Field | Meaning |
|---|---|
| `endpoint_candidate` | This frame contains a conservative visible cap candidate on the same supported track. Not a confirmed terminal. |
| `endpoint_u`, `endpoint_v` | Existing observed far-tip point in calibrated native 1920x1080 pixel coordinates. Both zero for false candidates. |
| `endpoint_confidence` | Minimum cap neural support / photometric floor quality, in [0,1]. Zero for false candidates. Not an accuracy probability. |
| `endpoint_clear_pixels` | Independently observed local clear-floor row distance beyond the cap, in native pixels. Not metric remaining distance. |
| `endpoint_reason` | Explicit acceptance/rejection reason for diagnostic logs. |

Endpoint geometry is taken from the original observed path, whose endpoints are
preserved in the resampled transmitted polyline. Tangent projections are used
only to find contradictory RGB continuation, never to add path points or infer
a hidden tail.

## Required per-frame evidence

- An already-valid supported long path, including unchanged original near/look checks.
- Far tip inside the existing tracking ROI, with at least 72 native pixels to its top boundary and adequate side margin.
- Broad cap support: median neural confidence >=0.95 and coverage >=0.60 at confidence >=0.90.
- Observed RGB cap within one tracker row step, not just the last available neural point.
- A clear, similarly illuminated floor corridor extending 144 native pixels beyond the cap. Dark objects, glare-like bright covers, patch boundaries and neural ambiguity reject the candidate.
- A further RGB-only contradiction search extends up to 432 native pixels beyond the cap to reject longer interior gaps even when the neural network misses the continuation. This is veto-only and does not select distant lines.

## Verification

Command:

```bash
/home/dong/anaconda3/envs/crack_vision/bin/python -m unittest discover -p 'test*.py' -v
```

54/54 Python tests passed, including 15 new endpoint cases: a real-looking cap,
continuous RGB line, missing neural support, glare gap, long interior gap,
bright/dark synthetic occluders, ROI and side cutoffs, low cap confidence,
separate side/distant cap, short-path rejection and ambiguous neural floor.

300 identical-frame calls of `visible_endpoint` on the local development CPU:
p50 1.73 ms, p95 2.92 ms, max 3.15 ms. This excludes neural inference and camera
fetch and is not a board latency qualification.

## Known limitations and required downstream behavior

- RGB alone cannot distinguish a genuine cap from an exactly floor-coloured
  covering or a gap whose continuation is completely outside the visible
  inspection region. A confidence value is not proof of topology.
- Actual cracked-ground segmentation, shadows, perspective variation, moving
  view accuracy and real-world endpoint repeatability are not qualified by
  synthetic tests. Conservative false rejection on textured/reflective floors
  is expected.
- The controller must independently confirm multiple new frames in a bounded
  locally metric pose, preserve track/session identity and stop on unhealthy
  pose. Single-frame evidence must not enable blind continuation or mark a
  task completed. Ordinary stale/invalid vision is not a terminal.
- Once a terminal is independently confirmed and its finite observed tail is
  cached, the existing path may later leave the camera view as the rear TCP
  approaches it. Whether that tail may be consumed and when the TCP finishes
  is exclusively the controller's bounded endpoint logic, not this module.

Only `endpoint_vision.py`, `test_endpoint_vision.py`, `long_tracker.py` and this
report belong to the frontend change. No camera launch/production controller,
policy weight, contact threshold, gain, or robot command was changed here.
