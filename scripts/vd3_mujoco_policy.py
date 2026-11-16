"""Pinned, reviewed actor for local VD3 MuJoCo simulation only."""
from pathlib import Path


VD3_POLICY = (Path(__file__).resolve().parents[1] / "policy/b2_z1_no_gun"
              / "mujoco_vd3_mirror_2000/policy.pt")
VD3_POLICY_SHA256 = "51eda90b2b77204ef66d949523486250e88c7062aeb1e86f0dc5478924291c78"
