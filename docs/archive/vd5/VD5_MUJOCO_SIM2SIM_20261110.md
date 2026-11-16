> 历史归档：内容和命令对应原记录日期，不是当前VD13操作指令。当前状态见[文档入口](../../README.md)。

# VD5 MuJoCo Visual Sim2Sim

## Ahead-observation candidate (2026-11-11)

The separate `bash scripts/debug_vd5_rear_preview.sh` uses up to 0.6 s of previously observed rear path, propagated by sent base commands. This is explicitly not measured odometry or simulator truth. In one 60 s run, combined active control reached 98.1% (82.9% used history), with eight braking episodes. TCP projection P95 was 2.63 cm and maximum 6.72 cm; continuity improved while P95 was worse than the repeated hold baseline. The previous entry remains available. Full comparison and limitations: [VD5_REAR_PREVIEW_20261111.md](/home/dong/rl_sar_b2_z1/docs/reports/vd5/VD5_REAR_PREVIEW_20261111.md).

## Rear intermittency update (2026-11-11)

The dedicated `debug_vd5_work_distance.sh` now explicitly enables observed-mask cross-section centres and a labelled 0.15 s body-relative target hold. The main runner's defaults remain unchanged. In the 60 s candidate, fresh-measurement control was 72.0%, held-target control 18.7%, and combined active control 90.7%; it advanced 12.27 m at a post-startup average of 0.209 m/s. TCP projection P95 was 2.17 cm, but the maximum was 7.72 cm, and some braking episodes remained. Full conditions and limitations: [VD5_REAR_STABILITY_20261111.md](/home/dong/rl_sar_b2_z1/docs/reports/vd5/VD5_REAR_STABILITY_20261111.md).

## Work-distance comparison (2026-11-11)

The measured-camera comparison is recorded in [VD5_WORK_DISTANCE_20261111.md](/home/dong/rl_sar_b2_z1/docs/reports/vd5/VD5_WORK_DISTANCE_20261111.md). A new, explicit simulation candidate uses a 0.80 m trailing target with at most 5 cm of local visible-line extrapolation. It advanced 7.17 m in 60 s, but still suffered intermittent rejected frames and averaged only 0.121 m/s after startup; TCP projection error reached 5.58 cm. This is an improvement, not a completed continuous-tracking validation. The baseline remains 0.70 m with the new far-end estimate disabled by default. Launch the separate candidate with `bash scripts/debug_vd5_work_distance.sh`.

## Measured-camera and gun-edge update (2026-11-11)

The renderer now uses [`config/vd5_measured_cameras.yaml`](/home/dong/rl_sar_b2_z1/config/vd5_measured_cameras.yaml). The rear transform is the measured matrix

```text
[ 0,  0.770435, -0.637519, -0.434510 ]
[ 1,  0,         0,          0       ]
[ 0, -0.637519, -0.770435,  0.026751 ]
```

and the recorded D435i rectified color intrinsics are `fx=607.6588`, `fy=607.8262`, `cx=323.6682`, `cy=246.7432` at 640x480. The front uses the recorded 2026-09-15 mount and intrinsics, scaled from the board's 1920x1080 rectified stream to the 640x480 MuJoCo render. The same values are passed to the renderer and frame metadata; the old 45-degree, `fx=fy=320` VD3 camera is no longer used by VD5.

Every rear capture carries six arm encoder angles. A fixed URDF gun mesh is forward projected into the measured camera and dilated by 8 pixels. The rear centerline is extracted only outside that exclusion mask, and the mask is applied again after morphology so closing cannot reconnect a gun edge to the seam. A blocked seam remains invalid. The overlay draws the excluded gun outline in blue and a rejected visible fragment in amber. This does not use simulator world pose, the simulated seam, or a TCP truth measurement.

The rear endpoint tangent now uses a 15 cm measured neighbourhood instead of the last two pixels. This fixes the seq-1123-style pixel hook that could falsely report missing support. Rejected-packet reasons and `latest_packet_capture_age_s` now distinguish fresh but rejected images from the age of the last accepted measurement (`capture_age_s`).

Validation session: `/home/dong/rl_sar_b2_z1/outputs/VD5_MUJOCO_DEBUG/vd5_20261111_092334_7hoz4cg4` (60 s, 3000 actor steps, no runtime error/fall). Of 548 processed pairs, 51 were valid; 490 failed the rear geometric-length check and 7 failed pixel-centerline length. The last frame was still fresh (0.117 s old, 93.5 ms processing). Only 263/3000 actor ticks had valid visual commands, so this is **not a successful continuous-walking result**. Mean forward speed 0.014 m/s is dominated by the stop. With the measured mount and current gun pose, the gun physically hides most of the centreline; the 10 s diagnostic's final visible fragment was only 6.05 cm, `x=[-0.691,-0.633] m`, insufficient to observe the trained 0.70 m point reliably. No validity threshold, gun geometry, actor, actuator gain, or frame age was relaxed to hide that occlusion. Camera/gun relative visibility remains the next issue; low stationary TCP errors from this run must not be presented as walking precision.

Regression checks cover the measured mount, actual rendered off-centre projection, encoder FK versus the articulated model, a gun edge with erroneous floor depth, preservation of an adjacent visible seam, stale encoders, and the endpoint-hook case. Existing visual front/rear/guard/path tests also pass. No board files were synchronized and no robot control was started.

## Run

From `/home/dong/rl_sar_b2_z1`:

```bash
bash scripts/debug_vd5_mujoco.sh
```

This opens the MuJoCo viewer and a vision window showing front RGB, rear RGB with detected seam, and rear aligned depth. The run is capped at 60 simulated seconds; close either viewer or press Ctrl+C to stop. Headless smoke test:

```bash
bash scripts/debug_vd5_mujoco.sh --headless --duration 10
```

Use `--vision-mode classical` only to diagnose a dark tape rendering. RHANet is the default and the only result comparable to the deployed perception model. `--no-vision-window` hides the OpenCV view but keeps the saved overlay.

## Command Data Path

```text
MuJoCo front camera RGB ──> RHANet centerline ──> calibrated ground points ──> robust local fit ──> constant vx + visual wz
MuJoCo rear camera RGB-D ─> RHANet centerline + measured depth plane ────────> 0.70 m TCP target in capture-base coordinates
```

The 640x480 rectified render uses the measured camera calibration described above. Paired images carry capture sequence, generation, and monotonic timestamps; the runner validates the pair and stops accepting frames older than the configured 1.0 s limit. The front command is constant `vx=0.20 m/s`; only `wz` is estimated from the observed front centerline. The rear target comes from the rear RGB-D centerline and depth-derived ground plane.

The rear camera cannot see the seam immediately beneath/behind the chassis. The controller bridges only the near-field interval between the base ground projection and the visible centerline, and permits at most 0.25 m of endpoint-tangent extension to reach the 0.70 m target. Longer missing support is rejected. The inferred target must also stay within a broad +/-35 cm x/y envelope around the trained 0.70 m trailing point, so a far-side arm/tool silhouette cannot silently become a valid TCP target. This is a bounded geometric treatment of camera/body occlusion, not a hidden simulator path lookup. Front curvature uses the existing robust local quadratic fit to avoid reacting to pixel-scale centerline hooks.

The initial TCP pose is calculated from arm encoder values and the source URDF forward kinematics. MuJoCo world pose and the known seam are used only by the separate evaluation logger; neither is returned to the command generator or actor. The actor remains the reviewed VD5 Startup `model_4498` export: 70 observations to 18 actions, same B2+Z1 VD3 zero-added-mass-gun model, 50 Hz actor, 200 Hz joint control.

## Simulation Limits

This is camera-rendered perception, not the previous ideal point-to-command path. It includes perspective, occlusion, scene lighting, RGB segmentation, aligned metric depth, and the same RHANet/geometry bridge used by the deployment workflow. MuJoCo does not model the real camera's lens distortion, D435i depth quantization/holes, sensor noise, USB timing, or board inference performance. Those still need real-camera validation.

The rendered tape has no collision or height. Supporting-foot distance to the visual tape is evaluation-only. A successful MuJoCo run does not establish hardware safety or real-world tracking accuracy.

## Previous RHANet Smoke Result (before the measured-camera update)

Session: `/home/dong/rl_sar_b2_z1/outputs/VD5_MUJOCO_DEBUG/vd5_20261110_171751_cosw4j2n`.

- 60.0 simulated seconds, 3000 actor steps, no fall guard or runtime error.
- 1582/3000 actor steps used valid visual commands (52.7%). Rejections: 1105 rear support too short, 158 targets outside the 0.70 m envelope, 150 waiting for a valid pair, and 5 rear paths not behind the base tangent.
- Commanded speed was 0.20 m/s; measured forward speed averaged 0.112 m/s because rear invalidity repeatedly brakes/holds the command.
- Base cross-track error P95 was 10.0 cm; TCP ground-projection error P95 was 5.13 cm (max 7.86 cm).
- Maximum body tilt was 2.92 degrees.

Interpretation: the realistic image interface runs end to end and exposes the current failure, but tracking is not yet acceptable. Rear segmentation occasionally follows arm/tool edges and frequently lacks enough observed support to cover the work point. The working-envelope check rejects many false targets rather than sending large TCP jumps to the policy. The base consequently slows and accumulates lateral error. The next issue to address is rear RGB/RGB-D centerline validity/near-field coverage, then front steering bias; this result does not justify deploying the current controller on hardware. The earlier 60-second ideal-geometry result does not describe this RGB/RGB-D run. Logs include `vision_bridge.log`, `dual_vision_packets.jsonl`, `latest_dual_overlay.png`, camera frame slots, `actor_trace.jsonl.gz`, `evaluation_truth.jsonl.gz`, `runtime_contract.json`, and `summary.json`.

## Files

- [play_vd5_mujoco.py](/home/dong/rl_sar_b2_z1/scripts/play_vd5_mujoco.py): renders and publishes paired camera frames, runs the VD5 actor, and saves session artifacts.
- [vd5_visual_control.py](/home/dong/rl_sar_b2_z1/scripts/vd5_visual_control.py): validates image provenance and converts observed centerlines to actor commands.
- [vd5_mujoco_control.py](/home/dong/rl_sar_b2_z1/scripts/vd5_mujoco_control.py): shared pose math and startup utilities.
- [debug_vd5_mujoco.sh](/home/dong/rl_sar_b2_z1/scripts/archive/vd5/debug_vd5_mujoco.sh): reproducible GUI/headless run command.
- [export_vd5_policy.py](/home/dong/rl_sar_b2_z1/scripts/export_vd5_policy.py): reviewed checkpoint export.
