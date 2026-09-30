# V19 contact-cycle MuJoCo profile

This profile is isolated from the existing `v4` deployment baseline.

- Source checkpoint: `model_11449.pt`
- Training run: `2026-09-21_08-48-36_proprio_forward_v19_contact_cycle_from11350`
- Observation contract: 70-D, unnormalized
- Action contract: 18-D joint position residual, scale `0.25`
- Exported actor validation: 256 random samples, maximum error `0`
- Checkpoint SHA256: `f85351e8ca9c04e9ef91f6d55916f0d1e159c8f6bf91ed31d498e0a62964b82f`

## Controls

- `0`: stand up and deploy Z1 to the installation pose
- `1`: confirm tool installation and hand control to the policy
- `T`: toggle trajectory/visual tracking
- `Space`: stop trajectory motion
- `9`: stop smoothly, verify symmetric four-foot support, detach the simulated
  tool, fold Z1, and lie down; pressing `Space` first is not required
- `Ctrl+C`: exit

## Validated smoke tests

- Proprioceptive walk: 20 simulated seconds at `0.25 m/s`, no fall.
- Visual trajectory: 25 simulated seconds at `0.25 m/s`, no fall; visual path and TCP target became active.
- Direct `9` during proprioceptive and visual tracking: both runs passed through
  `stopping -> settling -> getting_down -> idle` without a fall. Final tilt was
  `0.162 deg` and `0.134 deg`, respectively.
