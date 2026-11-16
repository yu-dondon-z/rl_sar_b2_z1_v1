# 独立诊断与报告工具

这些脚本从 `scripts/` 移入此目录。主仿真入口与其共用模块仍保留原路径。以下命令从项目根目录执行，并使用原有 `crack_vision` 或对应 Isaac Python 环境。

| 工具 | 用途 |
| --- | --- |
| `review_vd14_motion_transport.py` | 纯离线核对捕获→控制的传感器补偿误差、同图像平面高度、工作点／四元数／关节目标跳变、200Hz枪尖高频及原生枪体接触力；世界位姿仅用于离线评估，绝不回馈控制 |
| `replay_vd14_rear_evidence.py` | 保留RGB-D帧按同一RHANet输出对照旧组件与暗脊证据选择，不推进物理、不使用世界位姿；暗缝假设，不是任意实景识别验收 |
| `check_rear_mount_candidate.py` | 从各版实际URDF核对75cm夹持／后视前瞻、惯量、静态IK／启动采样、网格遮挡与渲染；不加载actor、不推进物理，[结果与边界](../../docs/reports/vd12/REAR_MOUNT_CANDIDATE_20261115.md) |
| `diagnose_vd2_legacy_getup.py`、`diagnose_vd2_stand_recovery.py` | 历史 VD2 起立/站立候选复现；输入 bundle 由参数指定 |
| `analyze_vd5_work_distance.py` | 已记录的工作距离对照 |
| `plot_vd5_gait_compare.py` | 步态对照绘图 |
| `analyze_vd6_joint_oscillation.py`、`analyze_vd6_joint_response.py` | 关节频带与落脚事件统计 |
| `probe_vd6_effort_readback.py` | 离线执行器读回诊断 |
| `replay_vd6_fixed_arm.py` | 固定底座机械臂重放；需要仿真环境 |
| `analyze_vd6_actor_feedback.py`、`report_vd6_actor_feedback.py` | actor 输入归因与汇总 |
| `analyze_vd6_roll_pulse.py` | 成对滚转脉冲会话对照 |
| `review_vd6_results.py`、`summarize_vd6_actuator_diagnosis.py` | 汇总已有评估数据 |

```bash
python scripts/diagnostics/analyze_vd6_actor_feedback.py --help
python scripts/diagnostics/analyze_vd6_joint_oscillation.py --help
```

历史报告中的命令已更新到新位置。JSON/日志中的旧路径保留为原始证据，手动重放时按 [移动清单](../../docs/maintenance/organization_20261112.json) 查找对应新路径。
