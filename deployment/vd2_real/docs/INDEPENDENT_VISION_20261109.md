# Independent Front/Rear Publication (2026-11-09)

## Evidence and Change

The recorded run `20261109_093859_501584` stopped on a front capture age
of 1.0137 seconds. Fifteen recorded processing cycles averaged 463.71 ms
(front 251.68 ms, rear 194.47 ms, preparation 17.56 ms). Serial processing
delayed both results, and delivery/receipt jitter consumed the remaining margin.

Front and rear now have separate latest-only processing workers. Network
inference shares a lock and one model, while geometric processing is independent.
Either worker can publish immediately after finishing; publication precedes
diagnostic storage and GUI copies. Front projection uses preceding IMU gravity
and configured height, never the rear ground plane in this mode.

`publication_sequence` identifies a publication, not an image. Each stream keeps
its original capture generation, sequence and host receipt time. A publication
may carry an unchanged result from the other stream. This never renews its age,
rear continuity anchor, front stable-frame count or recovery evidence. Start
qualification requires distinct evidence from both cameras.

Both streams must still be within the original one-second age limit. Stop
latching and explicit R1+Y restart remain unchanged. Legacy synchronized mode
retains its same-sequence and 40-ms receipt-skew checks. No motor gains, actor
weights, native IO, contact signals or hardware calibration are changed.

## Steering Correction Included

The previous real run commanded right steering while the nearest visible path
was left of the robot. The front estimate now chooses its target within the
nearest observed fitting span. It does not extrapolate into the unobserved near
region. Low-confidence estimates fall back to visible-segment guidance even
after startup handover. This corrects the recorded command sign, not a measured
claim that real lateral drift is eliminated.

## Validation and Operation

Unit tests cover independent capture expiry, unchanged-stream identity, blocked
rear processing with continuing front publication, and distinct-frame startup.
Saved-image benchmarks measure compute only: they do not validate live-camera
delivery, physical actuator response or real tracking accuracy.

The same board, recorded `latest_valid_` images and 40-iteration synthetic
30-Hz capture / 50-Hz actor load gave these preliminary timings:

| Pipeline | Both tasks done p50 | p95 | Maximum | Valid results |
| --- | --- | --- | --- | --- |
| Serial baseline | 297.73 ms | 322.44 ms | 326.18 ms | 40/40 |
| Independent geometry / locked network | 256.55 ms | 308.91 ms | 345.64 ms | 40/40 |

Median pair completion improved about 14%, but the worst measured cycle did
not improve. This short offline benchmark is not proof that stale stops are
eliminated. The functional benefit is that each task can publish at completion
instead of waiting for its partner. Only the blocked-worker regression tests
exercise that publication property; the timing tool waits for both tasks to
measure compute. Live acquisition, SDK IO, IMU alignment and frame freshness
still need a new supervised run. Old images are never published to control.

Install with the hashed patch installer. Original changed files and manifest
are retained under `patch_backups/independent_vision_<timestamp>/`; run logs stay
untouched. No C++ rebuild is required. Offline verification does not open robot
or camera IO:

```bash
cd /home/unitree/rl_sar_b2_z1_VD2
source .venv/bin/activate
python run.py --offline-check
```

Only when the operator is ready, start manually in the board desktop terminal:

```bash
bash start_real.sh --interface eth0
```

The launcher retains its existing front/rear display default. SSH without a
desktop cannot display those windows. A/Y/R1+Y/X/B behavior is unchanged.
