# 仿真与部署脚本

## 当前工作

2026-11-16：VD14最终5999的Isaac名义／mixed和MuJoCo真值回放完成30秒，姿态改善；双视觉修正传感器时间对齐、暗缝证据／工作点跳变拒绝及四元数翻号后，曲线与直线均完成30秒，明显挥杆减轻。[实际结果与启动命令](../docs/reports/vd14/VD14_PLAY_MUJOCO_20261116.md)。目标仍主要为有界估计，尚非已验收视觉部署；默认actor和实机未改动。

| 用途 | 入口 | 状态 |
| --- | --- | --- |
| 当前VD14双相机闭环 | [deploy_vd14_vision.sh](deploy_vd14_vision.sh)、[实现](deploy_vd14_vision.py) | RHANet暗缝证据＋传感器补偿＋姿态符号连续；曲线／直线30秒样本完成，尚未验收 |
| 当前VD14动作回放 | [play_vd14_mujoco_truth.sh](play_vd14_mujoco_truth.sh) | 最终5999，已跑30秒；不含视觉，不等同部署 |
| 当前VD14 Isaac Play | [play_vd14.sh](../../b2_z1_VD2/scripts/play_vd14.sh) | 名义／mixed均完成30秒；有真实视频／200Hz日志 |
| 当前VD13双相机视觉闭环 | [deploy_vd13_vision.sh](deploy_vd13_vision.sh)、[实现](deploy_vd13_vision.py) | 最终6199／75cm／0.95m；已接入RHANet，但连续视觉未通过 |
| 当前VD13动作回放 | [play_vd13_mujoco_truth.sh](play_vd13_mujoco_truth.sh) | 已跑30秒；真值命令诊断，不含视觉，不等同部署 |
| 当前VD13 Isaac Play | [play_vd13.sh](../../b2_z1_VD2/scripts/play_vd13.sh) | 使用训练工作区和Isaac环境；有真实视频／200Hz日志 |
| 75cm安装／相机前瞻静态核对 | [check_rear_mount_candidate.py](diagnostics/check_rear_mount_candidate.py) | [已有结果](../docs/reports/vd13/VD13_MOUNT75_DECISION_20261115.md)，不需要为整理重复评估 |
| 整枪几何契约静态核对 | [verify_vd12_geometry_contract.py](diagnostics/verify_vd12_geometry_contract.py)、[共用adapter](vd12_geometry_guard.py) | 不接actor，不代表动态验收 |
| VD12真实双相机闭环 | [deploy_vd12_vision.sh](deploy_vd12_vision.sh)、[实现](deploy_vd12_vision.py) | 历史对照；[连续视觉未通过](../docs/reports/vd12/VD12_VISION_DEPLOYMENT_20261114.md) |
| VD12真值诊断回放 | [play_vd12_mujoco.sh](play_vd12_mujoco.sh)、[实现](play_vd12_mujoco.py) | 不等于视觉部署；不能直接用于VD13 |
| VD11穿模诊断 | [diagnose_vd11_gait.py](diagnose_vd11_gait.py) | 失败候选的受保护回放，不是部署推荐 |
| 步态与策略公共工具 | [diagnose_vd5_gait.py](diagnose_vd5_gait.py)、[export_vd5_policy.py](export_vd5_policy.py) | 后续版本仍有复用，保留原路径 |
| 历代独立启动入口 | [archive/README.md](archive/README.md) | 归档shell＋原路径兼容软链接 |
| 独立诊断／离线分析 | [diagnostics/README.md](diagnostics/README.md) | 与日常入口分开 |
| 本地目录维护 | [maintenance/README.md](maintenance/README.md) | 不启动仿真或硬件 |

## 不按名称删除的共用模块

VD14日常观看直接运行`bash scripts/deploy_vd14_vision.sh`：默认同时打开MuJoCo和前后视觉窗口、渐变天空、不限时，关闭MuJoCo窗口或Ctrl+C正常退出。后视外推支持不足现在只下发零行走命令、保持最后TCP命令并等待，工作点恢复且连续后继续，不再因此结束仿真；窗口标注`Control: WAIT`，不把旧目标当新测量。其余视觉／相机时效故障及物理保护仍终止。60秒样本中8次等待均恢复，仍不代表全程视觉有效或完整验收。`--duration 秒数`是可选计时；`--headless`只关MuJoCo窗口，`--no-vision-window`只关识别窗口。速度、曲率等仍可覆盖。独立观看模型远裁剪250m；机器人相机仍6m、不渲染天空。VD12／VD13保留旧行为；详见[运行记录](../docs/reports/vd14/VD14_PLAY_MUJOCO_20261116.md)。

VD14与VD13物理安装不变，因此`vd14_mujoco_contract.py`复用VD13物理backend；策略身份及姿态奖励另行严格校验，不是回退VD13 actor。表中的VD13入口作为保留对照。

`vd1_*`、`vd3_*`、`vd5_*` 仍包含后续入口使用的场景、相机几何、策略元数据、视觉控制、模型契约和步态指标；`run_vd1_mujoco.py` 等也仍被入口／测试导入。保留这些Python文件和上游构建／下载工具。

[板载包说明](../deployment/vd2_real/README_板载编译与运行.md) 保留原路径，不在本次整理或当前仿真授权范围内。回归测试见 [tests/README.md](../tests/README.md)。新增一次性分析放入 `scripts/diagnostics/`，输出放入 `outputs/`。
