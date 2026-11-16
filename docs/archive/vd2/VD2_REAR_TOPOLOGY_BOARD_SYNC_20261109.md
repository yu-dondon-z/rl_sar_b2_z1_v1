# Rear topology board sync, 2026-11-09

Installed `rear_rgb_topology_20261109` on `unitree@192.168.123.164`, directory `/home/unitree/rl_sar_b2_z1_VD2`.

Changed files: `python/realtime_perception.py`, `tests/test_rear_topology.py`, and `docs/REAR_RGB_TOPOLOGY_20261107.md`. The candidate-status sentence in the packaged development note describes its original pre-sync state; this record supersedes that status.

Board isolated validation: `/home/unitree/vd2_rear_topology_check_XPoRCd/tests.log`, 133 tests passed, including native offline check. Installation hashes verified. Config, assets and retained runs were unchanged. No C++ rebuild required. No controllers or robot motion started.

Original files and manifest backup: `/home/unitree/rl_sar_b2_z1_VD2/patch_backups/rear_rgb_topology_20261109_093416_207019`.

Local patch scripts and baseline manifest: `outputs/VD2_REAR_TOPOLOGY_SYNC_20261109_hcNKFo`. Board archive: `/home/unitree/vd2_rear_rgb_topology_20261109.zip`.

This preserves RGB seam topology across unknown-depth pixels without treating them as depth measurements. Direct measured-depth requirements, freshness checks and stopping conditions remain unchanged. Offline validation does not establish successful real tracking; genuine depth gaps may still stop tracking.
