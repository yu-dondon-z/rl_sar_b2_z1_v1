"""Pinned local MuJoCo export of VD5 Startup model_4498."""

from pathlib import Path


POLICY_DIR = (Path(__file__).resolve().parents[1] / "policy/b2_z1_no_gun"
              / "mujoco_vd5_startup_4498")
VD5_POLICY = POLICY_DIR / "policy.pt"
VD5_MANIFEST = POLICY_DIR / "manifest.json"
VD5_CHECKPOINT_SHA256 = "9af8e2ca67382d3186bb9ea145930c25797b06e6189963bb1d996fee91f3fedf"
VD5_POLICY_SHA256 = "989895d8077841855b1aa6791aca24e3cf3eb4a7457bb3d902eec19fc6494075"
