# VD6 / VD5 MuJoCo 机身滚转脉冲闭环对照

2026-11-11。按用户同意的单项本地实验，在 0.80 m、固定机身坐标枪尖目标、0.20 m/s 指令下，分别运行 VD5 4498 / VD6 4997 的无扰动和脉冲配对会话。每段 20 s，200 Hz 物理 / 50 Hz actor，原控制增益、限速求解及 actor 输入不变；没有板载或机器人 IO。部署仍为 VD5。

## 施加方式与校验

10.000–10.120 s，对 MuJoCo `base_link.xfrc_applied` 施加沿**即时机身 x 轴**的外部滚转力矩 8 Nm（24 个物理步，名义积分幅值 0.96 Nm·s）；无扰动会话为 0。它是世界坐标下逐步计算的真实物理外力矩，不是修改 IMU、编码器、位置状态或 actor 输入，也不是跨引擎执行器对齐实验。

[analysis.json](../../../outputs/VD6_ROLL_PULSE_20261111/analysis.json) 验证：每对在脉冲前的整行物理、200 Hz 关节及 50 Hz actor 日志严格一致；实际施加的力矩范数只在 24 个预定步为 8 Nm，其余为 0。四段均 `duration_complete`，各 4000 物理步 / 1000 actor 步。表中的数值均为**同一策略的脉冲会话减去其无扰动会话**，不是把 VD5 和 VD6 的原始轨迹直接相减。

| 脉冲后窗口 | 指标（配对差值 RMS） | VD5 | VD6 |
| --- | --- | ---: | ---: |
| 0–0.2 s | 机身滚转角 | 0.01130 rad | 0.00958 rad |
| 0–0.2 s | actor 采样陀螺仪 x | 0.1275 rad/s | 0.1268 rad/s |
| 0–0.2 s | joint1 实际角度 | 0.00179 rad | 0.00310 rad |
| 0–0.2 s | joint1 实际角速度 | 0.0604 rad/s | 0.1897 rad/s |
| 0–0.2 s | joint6 目标角度 | 0.00241 rad | 0.02006 rad |
| 0–0.2 s | 枪尖机身系 Y | 0.645 mm | 8.726 mm |
| 1.4–3 s | 机身滚转角 | 0.00276 rad | 0.02425 rad |
| 1.4–3 s | joint1 实际角度 | 0.00342 rad | 0.04308 rad |
| 1.4–3 s | joint6 目标角度 | 0.01147 rad | 0.06054 rad |
| 1.4–3 s | 枪尖机身系 Y | 4.261 mm | 22.114 mm |

因此在**这两条实际步行轨迹的各自配对**中，VD6 的初始滚转扰动并不更大，但 joint1 实际响应、joint6 目标响应和枪尖横向偏移更大；后续差异也扩大。这支持先前“步行闭环生成的臂目标序列参与横向抖动”的链路，不支持仅把问题解释成 MuJoCo 固定底座下的机械臂响应。这里测的是配对扰动位移 RMS，**不是** 3 Hz 高频枪尖 RMS，不能拿 8.726 mm 与正式 5.797/11.229 mm 直接比较。

关键限制：脉冲前 VD5 的 FR/RL 接触（约 344/293 N），FL/RR 无接触；VD6 四足均接触（FL/FR/RL/RR 约 214/189/151/206 N），初始滚转角也不同（0.00889 / 0.02561 rad）。相同机身轴力矩不等于相同步态相位或动力学条件，接触变化与 actor 反馈无法通过此单点实验分开。1.4–3 s 的轨迹差值还包含步态相位漂移，不能简单称为线性系统的恢复时间，更不能把 joint1 归因当作已证明根因。本实验没有新增训练或确认哪一处应改增益。

## 可复核数据与日志口径

四段完整会话（含逐物理步受力/关节、actor 观测/动作、足力）位于 `outputs/VD6_ROLL_PULSE_20261111/vd5_baseline`、`vd5_pulse`、`vd6_baseline`、`vd6_pulse`，精确目录及策略哈希均写入 `analysis.json`。运行入口 [diagnose_vd5_gait.py](../../../scripts/diagnose_vd5_gait.py) 新增默认关闭的 MuJoCo 脉冲选项；[analyze_vd6_roll_pulse.py](../../../scripts/diagnostics/analyze_vd6_roll_pulse.py) 会拒绝不匹配的配对轨迹/脉冲时间/强度。VD6 checkpoint 哈希 `bd705fbc190baa1dfc915179f0e0a8badf37ea5a10fbeedca2cdd3f661d8a739`；部署 VD5 TorchScript 仍为 `989895d8077841855b1aa6791aca24e3cf3eb4a7457bb3d902eec19fc6494075`。

原始四段日志的 `base_angular_velocity_b` 曾用不等于 actor IMU 的对象速度读取，**本分析完全不使用该字段**；用 `actor_trace.gyro_b` 记录 actor 实际采到的值。会话 `joint_response` 内的历史字段 `actor_inputs_changed=false` 仅表示没有**人为覆盖**输入，真实脉冲后的 IMU/编码器当然会变化。运行结束后已将后续脚本的角速度日志改为读取原传感器，并将字段更名为 `actor_inputs_overwritten=false`；原始会话未重写或删除。

这是一个单时间、单幅值、单步态相位的 MuJoCo 闭环对照，足以支持“不要发布 VD6/原样续训 VD7”，不足以独立决定下一版本应修改 actor、足部奖励还是仿真动力学。继续保持 VD5 基线；任何新训练配方均应另行明确，不从本实验自动推导。
