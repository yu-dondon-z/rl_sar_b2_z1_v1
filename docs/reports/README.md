# 阶段评估报告

按版本集中存放评估结果，包含通过情况、未解决问题和原始数据链接。报告存在不表示该候选已发布或通过实机验收。操作入口见 [文档索引](../README.md)。

## 当前VD14

[VD14_PLAY_MUJOCO_20261116.md](vd14/VD14_PLAY_MUJOCO_20261116.md)：最终5999，Isaac／MuJoCo30秒；姿态明显改善，落脚非全面改善。双视觉修正传感器时间对齐、暗缝证据与四元数翻号后，曲线／直线30秒样本完成、挥杆减轻；目标主要为有界估计，尚未发布。旧失败日志保留。

## VD13与安装几何对照

| 文档 | 内容 |
| --- | --- |
| [VD13_PLAY_MUJOCO_20261116.md](vd13/VD13_PLAY_MUJOCO_20261116.md) | 最终6199启动命令、实际Isaac视频／mixed、MuJoCo真值30秒；双视觉2.24秒支撑不足停止，未发布 |
| [VD13_MOUNT75_DECISION_20261115.md](vd13/VD13_MOUNT75_DECISION_20261115.md) | 75cm夹持／0.95m工作点、新惯量／TCP、静态与旧策略试跑证据；原训练决定 |
| [REAR_MOUNT_CANDIDATE_20261115.md](vd12/REAR_MOUNT_CANDIDATE_20261115.md) | 安装候选与后视可见性分析；最终法兰偏移解释及选定值以VD13报告为准 |

以下为各旧版本阶段记录；旧结果不自动适用于新安装。

## VD5

| 文档 | 内容 |
| --- | --- |
| [VD5_GAIT_DIAGNOSIS_20261111.md](vd5/VD5_GAIT_DIAGNOSIS_20261111.md) | VD5 落脚与枪尖抖动对照 |
| [VD5_REAR_PREVIEW_20261111.md](vd5/VD5_REAR_PREVIEW_20261111.md) | VD5 后视提前观察与路径缓存试验，2026-11-11 |
| [VD5_REAR_STABILITY_20261111.md](vd5/VD5_REAR_STABILITY_20261111.md) | VD5 后视间歇失效改进，2026-11-11 |
| [VD5_WORK_DISTANCE_20261111.md](vd5/VD5_WORK_DISTANCE_20261111.md) | VD5 末端工作距离比较，2026-11-11 |

## VD6

| 文档 | 内容 |
| --- | --- |
| [VD6_ACTOR_FEEDBACK_20261111.md](vd6/VD6_ACTOR_FEEDBACK_20261111.md) | VD6 actor 闭环输入与 joint1/6 振荡归因 |
| [VD6_ACTUATOR_DIAGNOSIS_20261111.md](vd6/VD6_ACTUATOR_DIAGNOSIS_20261111.md) | VD6 关节振荡、落脚关联与执行器诊断 |
| [VD6_ROLL_PULSE_REVIEW_20261111.md](vd6/VD6_ROLL_PULSE_REVIEW_20261111.md) | VD6 / VD5 MuJoCo 机身滚转脉冲闭环对照 |

## VD7

| 文档 | 内容 |
| --- | --- |
| [VD7_SHORT_TRAINING_REVIEW_20261111.md](vd7/VD7_SHORT_TRAINING_REVIEW_20261111.md) | VD7 100 轮短训筛选：未通过，停止追加 |

## VD8

| 文档 | 内容 |
| --- | --- |
| [VD8_ROLL_LANDING_REVIEW_20261111.md](vd8/VD8_ROLL_LANDING_REVIEW_20261111.md) | VD8 600 轮阶段评估：没有形成可发布候选 |

## VD9

| 文档 | 内容 |
| --- | --- |
| [VD9_TRAINING_REVIEW_20261112.md](vd9/VD9_TRAINING_REVIEW_20261112.md) | VD9 训练与 MuJoCo 步态复盘 |
| [VD9_WARM_START_500_REVIEW_20261112.md](vd9/VD9_WARM_START_500_REVIEW_20261112.md) | VD9 `model_1500` 热启动后 500 轮复盘（2026-11-12） |

## VD11

| 文档 | 内容 |
| --- | --- |
| [VD11_FINAL_REVIEW_20261114.md](vd11/VD11_FINAL_REVIEW_20261114.md) | 6000轮长训、新枪Isaac／MuJoCo运动质量和隔离回放入口；未替换默认actor |
| [VD11_PLAY_GEOMETRY_REVIEW_20261114.md](vd11/VD11_PLAY_GEOMETRY_REVIEW_20261114.md) | 两端复核发现枪体自相交／高度失真；失败证据与受保护Play，不是部署推荐 |

## VD12

| 文档 | 内容 |
| --- | --- |
| [VD12_GEOMETRY_GUARD_20261114.md](vd12/VD12_GEOMETRY_GUARD_20261114.md) | 整枪几何禁区、净隙奖励和终止的实现阶段记录；含可达性／静态双端核对，后续正式长训见2000轮评估 |
| [VD12_2000_PLAY_REVIEW_20261114.md](vd12/VD12_2000_PLAY_REVIEW_20261114.md) | 正式长训2000轮趋势、名义／mixed真实Isaac 30秒Play与运动质量；原6000计划继续，未部署 |
| [VD12_FINAL_PLAY_MUJOCO_20261114.md](vd12/VD12_FINAL_PLAY_MUJOCO_20261114.md) | 6000轮最终5999，实际Isaac视频、mixed及MuJoCo隔离仿真，直接查看命令和剩余限制 |
| [VD12_VISION_DEPLOYMENT_20261114.md](vd12/VD12_VISION_DEPLOYMENT_20261114.md) | 真正前RGB／后RGB-D图像→RHANet→VD12控制；与真值诊断回放分开记录 |

