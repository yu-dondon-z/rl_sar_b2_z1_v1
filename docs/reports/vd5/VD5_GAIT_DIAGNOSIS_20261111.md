# VD5 落脚与枪尖抖动对照

结论：同一策略在无视觉、固定目标下，两种引擎中仍有快速落脚与末端高频运动。建议从 VD5 定向续训，新增落脚减速/冲击和机械臂动作连续性奖励；同时保留对部署差异的检查。当前结果不能证明 MuJoCo 接触或执行器模型完全等同于 PhysX，也不是实机验证。

## 对照条件

- 策略：VD5 Startup model_4498 的同一 TorchScript 导出，SHA256 `989895d8077841855b1aa6791aca24e3cf3eb4a7457bb3d902eec19fc6494075`。
- 0.2 m/s 固定直行指令，vy=wz=0；两秒五次曲线启动渐变；之后末端固定在机身坐标 `[-0.80,0,-0.54] m`，姿态与当前部署一致。
- 同一初始关节姿态，base 初始高度 0.50 m，零附加枪质量模型，平地，PD 160/5，50 Hz actor、200 Hz 物理/PD。
- MuJoCo 保留当前部署的隐式位置项数值换算和质量度量速度约束；PhysX 使用源资产的显式腿部 PD、隐式机械臂执行器及源场景求解设置。两端不调增益、不滤波策略动作。
- 相机关闭，无视觉帧失效、轨迹记忆切换或视觉停车。actor 只接收规定的命令、IMU、关节编码器和上一次原始动作。根位置、TCP 真值和接触仅供单向记录/评价。
- 本次固定的是**机身相对目标**，不是地面上的固定点。用于隔离命令抖动，不能把本次 3D 目标误差直接当成视觉焊缝投影精度。

## 60 秒结果

统计去掉前 5 秒启动段。两端均完整结束，3000 actor 步、12000 物理步，无摔倒。

| 指标 | MuJoCo | Isaac Lab / PhysX |
| --- | ---: | ---: |
| 实际前向速度均值 | 0.231 m/s | 0.213 m/s |
| 最大倾角 | 1.68° | 3.74° |
| 机身高度 >3 Hz RMS | 0.78 mm | 0.93 mm |
| 枪尖世界高度 >3 Hz RMS | 2.32 mm | 1.22 mm |
| 枪尖机身相对 XYZ >3 Hz 向量 RMS | 5.81 mm | 6.19 mm |
| 固定机身目标 3D 位置误差 P95 | 21.5 mm | 30.7 mm |
| 机械臂目标每 20 ms 变化 P95 / 最大 | 0.043 / 0.122 rad | 0.051 / 0.371 rad |
| 腿部目标每 20 ms 变化 P95 / 最大 | 0.133 / 0.444 rad | 0.135 / 0.883 rad |

触地前足端下降速度中位数，FL/FR/RL/RR 顺序：

| | FL | FR | RL | RR |
| --- | ---: | ---: | ---: | ---: |
| MuJoCo | 1.09 | 0.87 | 1.34 | 1.66 m/s |
| PhysX | 1.35 | 0.77 | 1.46 | 1.31 m/s |

触地事件要求此前至少 60 ms 腾空、此后至少 10 ms 持续 >10 N 接触。速度取首次过阈值前一个 5 ms 样本的足部 link 原点竖直速度；同时统计随后 40 ms 内的力峰值。数据是 200 Hz 采样，不能当作连续时间真实冲击峰值。

RMS 通过三阶 3 Hz 高通双向滤波计算，并去除滤波边缘各一秒。机身相对 XYZ RMS 是向量均方根；它不同于世界 Z 单轴 RMS，也不同于静态目标偏差。

原始日志：

- `outputs/VD5_GAIT_COMPARE/mujoco_20261111_150418_3yj6vntg/`
- `outputs/VD5_GAIT_COMPARE/isaac_20261111_150433_zr4o8co1/`
- 图：`outputs/VD5_GAIT_COMPARE/vd5_gait_baseline_20261111.png`

各目录的 `manifest.json` 记录策略指纹和指令；`asset_audit.json` 验证模型质量、刚体与关节结构；`actor_trace.jsonl.gz` 为 50 Hz 输入/动作，`physics_truth.jsonl.gz` 为 200 Hz 独立评价数据；`summary.json` 汇总结果。

两个较早的 10 秒目录只供脚本开发检查，未用于上述对照：其中首次 MuJoCo 速度读取使用了刚体惯性局部轴，随后已改成 link 原点 Jacobian 速度并重跑 60 秒；首次 PhysX 在 IMU 初始化时退出，保留错误摘要供审计。

## 原训练工作距离复核

为排除只因部署目标伸到 0.80 m 而产生问题，又用 0.70 m、其余同样的固定指令，两端各运行 30 秒，均完整结束。PhysX 的实际速度均值为 0.200 m/s；触地前下降速度中位数 FL/FR/RL/RR 为 1.16/0.65/0.99/1.34 m/s，枪尖机身相对 XYZ 高频向量 RMS 为 5.20 mm。MuJoCo 对应为 0.229 m/s、0.91/1.18/1.63/1.65 m/s、4.61 mm。

因此，在训练原本的工作距离下也能复现，不能只归因于 0.80 m 伸臂。两引擎实际速度差仍在，本轮没有修复它；后续效果比较应同时报告速度，防止把走慢误当成落脚改善。

日志：`outputs/VD5_GAIT_COMPARE/mujoco_20261111_151745_80c0mzcj/` 与 `outputs/VD5_GAIT_COMPARE/isaac_20261111_151757_dtoaqbv8/`。

## 复现

```bash
cd /home/dong/rl_sar_b2_z1
bash scripts/debug_vd5_gait_compare.sh both --duration 60
```

也可将 `both` 改成 `mujoco` 或 `isaac`。这是无窗口的量化对照，会创建独立输出目录；不连接机器人。

训练原本的 0.70 m 目标可另外检查：

```bash
bash scripts/debug_vd5_gait_compare.sh both --duration 30 --work-distance 0.70
```

绘图脚本 `scripts/diagnostics/plot_vd5_gait_compare.py` 接收 MuJoCo、PhysX 两个目录和 `--output` 图片路径。奖励/统计相关测试已通过；部署侧 39 项 VD5 测试通过。

## 续训

新增 VD6 独立任务 `Unitree-b2z1-VD6-Velocity`，继承 VD5 Startup，原 VD5 部署及策略继续保留。

训练说明：`/home/dong/b2_z1_VD2/docs/archive/vd6/VD6_TRAINING_20261111.md`。

现在只完成接线和短程训练验证。正式训练后的策略必须重新比较步态、实际速度、末端投影误差和视觉闭环效果，才能认定改善。
