# VD13安装候选：决定先训练

后续更新（2026-11-16）：最终6199的Play和MuJoCo脚本已就绪，两端动作样本完成30秒；双视觉2.24秒停止，未验收发布。最新入口见[Play／MuJoCo记录](VD13_PLAY_MUJOCO_20261116.md)，以下保留原建模／训练决定。

2026-11-15。**新安装建模完成，需要VD13长训练；不直接部署旧VD12策略。** 目录整理时已观察到正式`vd13_mount75_long_20261115`训练运行（256环境、6000轮）；本次整理没有重启或修改训练。默认actor和实机均未改动。

## 做了什么

- 新增独立75cm夹持点URDF，重算完整合并惯量、质心及TCP；两端读取同一URDF，旧VD10/11/12不替换。
- 枪长1100mm、外径56／内径52mm、质量2.93kg不变。夹环到枪尖750mm，尾段350mm，夹环相对法兰局部+Z上偏100mm。
- 法兰中心在link06中有75mm偏移，因此新TCP=(0.825,0,0.1)m。修正最初静态诊断混用link06原点和法兰中心的解释，旧输出仍保留。
- 工作点由底盘后方0.80m改为0.95m；相机安装、奖励、增益、速度限制、70维actor输入与18维位置动作不改。保留原有整枪200Hz几何终止和枪体接地保护。

## 为什么要训练

新姿态在名义水平底盘下几何可达，最小限位余量33.6°，整枪保守净隙下界40.5mm。当前近似相机外参下，计入管身遮挡／排除边缘后，朝机体方向的连续直线前瞻由15.4cm增加到30.3cm；近枪尖仍有约3.4cm盲区，并非直接看到全部工作点。

沿用实际重置关节，按原启动逻辑取3个地面课程比例、每个21个时刻，共63个静态IK样本全部可达；最小保守净隙下界38.83mm。这是固定水平底盘的离散几何检查，不是连续运动／完整自碰撞／动态工作包络保证。

将**未更新参数的VD12最终5999 actor**用于新VD13模型，4环境、15秒、满地面课程、名义重置的Isaac试跑结果：

| 指标 | 结果 |
| --- | --- |
| 实际reset／终止 | 0 |
| 最后5秒三轴TCP位置误差的每环境均值 | 18.84–18.89cm |
| 最后5秒枪尖高度 | 约17.0cm |
| 同期目标高度 | 约1.65cm |
| 最后5秒枪轴角误差 | 约16.5–16.6° |
| 最小保守整枪净隙下界 | 36.45mm |

它没有倒地，但没有跟上新工作点。5秒试跑的偏差延续到15秒；不是仅凭一小段训练奖励判断收敛。TCP与负载分布／工作距离都变了，旧模型不能直接作为新安装部署。

## 训练入口（正式训练已启动，勿重复运行）

```bash
cd /home/dong/b2_z1_VD2
bash scripts/train_vd13.sh --run_name vd13_mount75_long_20261115
```

默认256环境、固定配置6000轮，从VD12最终5999迁移actor／critic／AMP；重置优化器、轮数和课程，让新安装重新经历启动到地面的过程。每100轮保存，保留全部检查点。6000是预算，不是收敛保证；未收敛时按相同配置续训，再根据完整回放判断，不反复短训改奖励。

TensorBoard：

```bash
/home/dong/anaconda3/envs/env_isaaclab/bin/tensorboard \
  --logdir /home/dong/b2_z1_VD2/logs/rsl_rl/b2_z1_vd13_mount75_amp \
  --port 6006
```

打开`http://localhost:6006`。完整中止续训命令见[训练说明](/home/dong/b2_z1_VD2/docs/VD13_MOUNT75_TRAINING_20261115.md)。不要从`*smoke*`或试跑目录开始正式训练。

## 验证证据

- [静态几何／相机／63点启动结果](/home/dong/rl_sar_b2_z1/outputs/rear_mount_candidate/20261115_164546_ylipuszf/summary.json)；[后视渲染](/home/dong/rl_sar_b2_z1/outputs/rear_mount_candidate/20261115_164546_ylipuszf/clamp_tip_750mm_work_950mm.png)。绿点是几何测试投影，不是识别结果。
- [15秒策略试跑及终止统计](/home/dong/b2_z1_VD2/logs/rsl_rl/b2_z1_vd13_mount75_amp/2026-11-15_16-44-46_vd13_transfer_probe_15s/vd10_policy_probe.json)；[实际200Hz轨迹](/home/dong/b2_z1_VD2/logs/rsl_rl/b2_z1_vd13_mount75_amp/2026-11-15_16-44-46_vd13_transfer_probe_15s/vd10_200hz_trace.json)。文件名沿用共用诊断格式，模型哈希／task为VD13，加载actor来源为VD12。
- [Isaac实际质量／COM／惯量／TCP检查](/home/dong/b2_z1_VD2/logs/rsl_rl/b2_z1_vd13_mount75_amp/2026-11-15_16-42-11_vd13_plumbing_smoke_20261115/vd10_runtime_asset.json)。
- [2轮保存检查](/home/dong/b2_z1_VD2/outputs/vd13_plumbing_smoke_console_20261115.log)与[1轮恢复检查](/home/dong/b2_z1_VD2/outputs/vd13_resume_smoke_console_20261115.log)：优化器Adam步数40→60、课程累计48→72步、地面比例0.0024→0.0036。仅验证训练管线，不作学习效果结论。底层沿用RSL-RL的轮号语义，续训第一个标签可能与载入标签相同，实际优化器／课程确实推进。

训练仍是几何教师命令，不是相机图像直接输入actor。最终应先检查VD13的Isaac Play和动态净隙，再接入同安装／0.95m命令的MuJoCo双视觉闭环，独立检查枪尖摆动、落脚冲击和跟踪。VD12既有视觉失败记录不被本次静态前瞻结果覆盖；未通过前不发布默认actor，更不操作机器人。
