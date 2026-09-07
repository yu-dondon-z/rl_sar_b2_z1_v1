# B2+Z1 deployment v1.5 — no vision

This release runs the B2+Z1 whole-body AMP policy from proprioceptive state
and operator velocity commands. It does not start or require RHACrackNet,
RGB-D cameras, or the visual trajectory bridge.

## Run

```bash
cd /path/to/rl_sar_b2_z1_v1
./scripts/run_b2_z1_no_vision.sh
```

Keyboard sequence:

1. Press `0` and wait for the robot to finish standing.
2. Press `1` and wait for policy warm-up.
3. Use `W/S/A/D/Q/E` for velocity commands.
4. Press `Space` to stop and `R` to reset.

The deployed policy is the TorchScript export of
`rear_deployment_finetune_from7999/model_9400.pt`.
