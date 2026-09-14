# B2 + Z1 real built-in-camera trajectory deployment

The policy still consumes IMU orientation/angular velocity in its normal
observation. Crack positions are **not** converted into accelerometer values.
They use this timestamped transform chain:

```text
B2 front JPEG -> RHACrackNet pixels -> calibrated local road plane
               -> B2 base_link at capture time -> odom (cached history)
               -> current B2 base_link -> velocity and rear TCP target
```

`rt/sportmodestate` supplies `odom -> base_link`. IMU acceleration alone is
not integrated because its translation drift is too large for the front
camera-to-rear-tool delay.

## 1. Build

```bash
cd /home/dong/rl_sar_b2_z1
cmake --build cmake_build --target rl_real_b2_z1_no_gun -j2
```

The built-in stream uses Unitree's B2 `FrontVideoClient`, not a D405. Verify
the checked-out Unitree Python SDK can be imported:

```bash
PYTHONPATH=/home/dong/unitree_sdk2_python /home/dong/anaconda3/bin/python -c \
  'from unitree_sdk2py.b2.front_video.front_video_client import FrontVideoClient; print("B2 camera OK")'
```

On this workstation use `/home/dong/anaconda3/bin/python` for the bridge. It
already contains both `cyclonedds` and the RHACrackNet runtime. The
`crack_vision` environment currently lacks `cyclonedds`, so it cannot connect
to the built-in B2 stream.

## 2. Calibrate the fixed camera transform

Edit `config/b2_builtin_front_camera.yaml`. Supply the built-in camera's
intrinsics, optical-frame-to-`base_link` transform, and measured standing base
height. Leave `calibrated: false` until calibration has been checked against
several known floor points. The launcher intentionally refuses an unapproved
identity transform.

The checked-in B2 SDK exposes compressed front/rear RGB clients but no aligned
depth-image client. This first real interface therefore intersects calibrated
camera rays with the local road plane. The separate RealSense script remains
available only for an explicitly attached RGB-D camera; it is not used by the
B2 built-in-camera launcher.

## 3. Read-only preflight

```bash
cd /home/dong/rl_sar_b2_z1
./scripts/run_b2_z1_real_vision.sh enp5s0 --preflight
```

Replace `enp5s0` with the Ethernet interface connected to B2. This checks B2,
Z1, and fresh `rt/sportmodestate` without publishing low-level motor commands.

## 4. Supported real run

Before enabling vision, validate the corrected deployment stand by itself:

```bash
cd /home/dong/rl_sar_b2_z1
./scripts/run_b2_z1_real.sh enp5s0 --run --z1-ip 192.168.123.110
```

Enter `RUN`, press only `0`, and do not press `1`. The controller now uses the
stock symmetric B2 leg stance for GetUp, independently of the asymmetric joint
reference required by the trained actor. It will print
`max_leg_target_error` and block policy handoff until every leg is close to the
stand target. Use `P` or the remote emergency stop if a leg moves in the wrong
direction.

After that stand-only check succeeds, run the visual launcher:

```bash
cd /home/dong/rl_sar_b2_z1
./scripts/run_b2_z1_real_vision.sh enp5s0 --run \
  --z1-ip 192.168.123.110
```

Use a support frame for the first trial. Enter `RUN` only after the camera
window stays `VALID`. Then press `0`, wait for standing to settle, press `1`,
and finally press `T`. `Space` stops visual following and `P` requests passive
mode.

The controller refuses or exits visual mode when:

- camera and odometry timestamps differ by more than 0.15 s;
- odometry is older than 0.25 s or jumps more than 0.75 m;
- the projected path is shorter than 0.30 m or becomes stale;
- RHACrackNet confidence is below the configured threshold.

Tune thresholds only from logs and recorded RGB-D data. Do not relax timestamp
or odometry checks to make an unavailable positioning source appear valid.
