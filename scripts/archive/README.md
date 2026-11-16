# 历史启动脚本

2026-11-15：10个旧独立shell按版本归档，原`scripts/`路径保留相对软链接；旧路径和归档路径都定位原仓库。任务参数保持原样，仅增加历史入口提示。

| 子目录 | 旧入口 |
| --- | --- |
| `vd1/` | `debug_vd1_flat_walk.sh`、`debug_vd1_truth_curve.sh`、`debug_vd1_dual_vision.sh` |
| `vd3/` | `debug_vd3_rear_vision.sh`、`play_vd3_mujoco.sh`、`play_vd3_front_path.sh` |
| `vd5/` | `debug_vd5_gait_compare.sh`、`debug_vd5_mujoco.sh`、`debug_vd5_work_distance.sh`、`debug_vd5_rear_preview.sh` |

当前VD13动态／双视觉验收未完成；这些旧命令不适用于直接验收75cm新安装。现有Python中包含后续视觉、仿真和测试仍在导入的公共模块，保留原路径。

当前工具见 [脚本索引](../README.md)，备份、清单与恢复见 [维护记录](../../docs/maintenance/README.md)。
