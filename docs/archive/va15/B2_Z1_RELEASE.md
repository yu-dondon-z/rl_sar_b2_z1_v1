# B2+Z1 deployment v2 — visual trajectory following

> 历史发布说明，原文保留。当前本地入口见 [文档索引](../../README.md)，
> 板载操作见 [板载编译与运行](../../../deployment/vd2_real/README_板载编译与运行.md)。

This is the complete B2+Z1 deployment. RHACrackNet extracts a crack trajectory
from the built-in front RGB-D camera, the trajectory controller generates
forward/yaw commands, and the rear-facing Z1 tracks cached crack points.
Ordinary proprioceptive velocity control remains available through
`scene_flat`.

## Run

```bash
cd /path/to/rl_sar_b2_z1_v1
./scripts/run_b2_z1_vision_trajectory.sh
```

Keyboard sequence:

1. Press `0` and wait for the robot to finish standing.
2. Press `1` and wait for policy warm-up.
3. Press `T` to start visual crack following.
4. Press `Space` to stop and `R` to reset.

The deployed policy is the TorchScript export of
`rear_deployment_finetune_from7999/model_9400.pt`. Its rear TCP observation
matches training at `z=-0.35 m`. The controller uses a local one-metre
trajectory chord and bounded cross-track feedback. If the vision bridge
becomes stale, autonomous motion is stopped instead of continuing blindly.

## No-vision mode

```bash
./scripts/run_b2_z1_no_vision.sh
```

Then use `0`, `1`, and `W/S/A/D/Q/E` as in v1.5.
