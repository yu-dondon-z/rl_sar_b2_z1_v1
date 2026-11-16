# B2+Z1 仿真与部署文档

2026-11-16：当前候选为 **VD14最终5999（75cm夹持点／0.95m工作距离）**。Isaac名义／mixed及MuJoCo真值回放均完成30秒，后部下沉／前脚过窄明显改善；双视觉修正拍摄→控制转换、灰色区域误选及四元数翻号后，曲线／直线均完成30秒，挥杆明显减轻，尚未完成部署验收。

| 用途 | 入口 |
| --- | --- |
| 当前VD14：Play、MuJoCo、双视觉命令与真实结果 | [VD14实跑记录](reports/vd14/VD14_PLAY_MUJOCO_20261116.md) |
| VD14：姿态奖励、训练与恢复 | [训练说明](../../b2_z1_VD2/docs/VD14_POSTURE_TRAINING_20261116.md) |
| 当前VD13：Play、MuJoCo启动与实际结果 | [Play／MuJoCo记录](reports/vd13/VD13_PLAY_MUJOCO_20261116.md)，明确区分动作回放和视觉闭环 |
| 当前VD13：模型验证、训练决定与证据 | [VD13候选报告](reports/vd13/VD13_MOUNT75_DECISION_20261115.md) |
| 当前VD13：训练、TensorBoard、中止恢复 | [训练说明](../../b2_z1_VD2/docs/VD13_MOUNT75_TRAINING_20261115.md) |
| 项目简要流程 | [项目方案](B2_Z1_PROJECT_PLAN_20261113.md) |
| VD12历史对照：最终模型Play | [Isaac与MuJoCo记录](reports/vd12/VD12_FINAL_PLAY_MUJOCO_20261114.md) |
| VD12历史对照：真实双相机闭环 | [视觉部署记录](reports/vd12/VD12_VISION_DEPLOYMENT_20261114.md)，后视支持不足停止，连续视觉未通过 |
| 历代效果／穿模／落脚／枪尖抖动证据 | [阶段评估](reports/README.md) |
| 旧版部署说明与历史交接 | [历史归档](archive/README.md) |
| 当前工具、共用模块与测试 | [脚本索引](../scripts/README.md)、[测试说明](../tests/README.md) |
| 板载包与旧补丁（保留，不在本次操作范围） | [板载说明](../deployment/vd2_real/README_板载编译与运行.md)、[补丁索引](../patches/README.md) |
| 两仓整理、备份与恢复 | [维护记录](maintenance/README.md) |

识别与转换已补像素证据、工作点连续性拒绝和有符号四元数传输，参考训练的转换而不接训练真值；目标仍主要依赖有界历史／估计。后视支持不足现零行走等待／恢复，60秒样本完成，不代表全程视觉有效或实机验收。源码、依赖与模型清单见 [VD14 GitHub快照](VD14_GITHUB_SYNC_20261116.md)。不追加短训，不替换默认actor或发布实机。以下VD13条目保留作历史对照。

日常说明放在 `docs/`；阶段评估放在 `docs/reports/<版本>/`；历史方案放在 `docs/archive/<版本>/`。历史记录中的命令不视为当前推荐。
