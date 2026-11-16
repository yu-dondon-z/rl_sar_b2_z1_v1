# VA15 V256 接管续办结果（2026-10-29）

> 最新：四次实机记录已定位重复开始遗漏站立混合状态的问题，本地旧/新 FSM 闭环对照分别失败/通过，33/33 回归通过。这次 FSM 与保留视觉证据的启动脚本**尚未同步板载**。先读 [四次日志及重复停车修复](VA15_FOUR_RUN_STOP_TRACKING_20261029.md)；下文保留此前已部署 V256 的历史阶段。

> 后续进度：用户明确授权同步后，已于 2026-10-29 完成板载更新、编译和离线自检。控制程序未启动。指纹、备份及证据见 [板载同步记录](VA15_V256_BOARD_SYNC_20261029.md)。下文离线复核结果保持有效，原“尚未同步”事项已完成。

工作目录 `/home/dong/rl_sar_b2_z1`。本轮完整阅读 `HANDOFF_VA15_VISUAL_TCP_20261029.md`，以及 V256/V255 本地报告，再按当前代码和原始数据复核。以下内容取代旧文档中的进度判断，历史文件和失败记录保留。

**本轮完成本地视觉停止修复、闭环验证和 TCP 误差链定位。TCP 精度问题仍未解决，不能把本次工作描述为整机故障全部修好。** 用户要求保持操作简单；未增加启动步骤、人工确认流程、增益调节或接触/帧时效放宽。

最初离线复核阶段只通过 SSH/SCP 读取既有日志、配置、源码和指纹；随后按用户“先同步板载吧”的明确授权完成文件同步、编译和离线检查。没有运行控制器、释放厂商控制或发起运动；实机控制/运动仍未获本轮授权。

## 当前代码和结果

候选仍位于 `patches/va15_tcp_priority_v256_20261028`，没有覆盖主程序或 VA15 权重。

- `production/tcp_priority_session.hpp`：同一控制周期结束丢视觉的整段会话。明确拒绝的包不再借用 tolerant guide 保存的旧有效帧继续走；没有新包时仍按原始采集时间的 500 ms 上限判断过期。
- `production/rl_real_b2_z1_no_gun.cpp`：接入上述逻辑；恢复原有局部位姿失效停止条件在 TCP 优先模式下的作用；停止时清除启动暂存请求、关闭前进推理许可，冻结 TCP 参考并清除 active。画面恢复不会自行重新启动。策略退出后同样关闭会话。
- R1+Y 重复报文忽略逻辑、Armed→Forward 的正向请求、X/B/Space 停止保持现有操作方式。B 的趴下流程沿用原 FSM。
- `test_tcp_priority_session.cpp`：覆盖有效持续运行、明确拒帧、过期、错误会话/时间、健康/位姿/策略失效、人工停止、缓存冻结和显式重新开始。
- native `--offline-check` 也新增拒帧结束 TCP 会话的内存回归，构造过程中不初始化机器人或网络。
- `simulate_sessions.cpp`：新增三个 TCP 会话故障场景及一个**仅供离线试验**的目标补偿场景。补偿没有接入 native 控制器。

最新验证目录为 `patches/va15_tcp_priority_v256_20261028/outputs/local_review_20261029`：

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 当前 native 编译及 CTest | 32/32 | `ctest_final.log`，包含当前 native 离线自检 |
| 持续有效视觉、明确拒帧、局部位姿中断闭环 | 3/3 | `sessions_final/summary.json` |
| 上述闭环中的人工重新开始和最终 B 趴下 | 3/3 | 同上；均到 Passive、无跌倒、无自动续走 |
| 实测 3064 帧 actor 回放 | 最大差 2.85795e-6 | `tcp_analysis/diagnosis.json` |
| 归档训练 URDF / 部署 FK，1000 姿态 | 最大位置差约 1.14e-7 m | `archived_geometry.json` |
| 原目标 / 小幅目标补偿，三种摩擦系数 | 6 组均完成启停，但补偿未消除精度误差 | `compensation_probe/summary.json` |

这里的闭环测试使用合成视觉、VA15 和原 FSM/独立估计器。三个停止用例覆盖会话和停车链路；补偿对照是固定机身系 TCP 目标实验，均不是实际相机/枪具的精度验收，也不是完整动态裂缝跟踪验证。

最新源文件、可执行文件与依赖的 SHA-256 见该目录的 `manifest.json`，源码快照见 `sources_tested.tar.gz`。初次编译时新 target 尚未重新配置、补偿探针初次编译缺少类型定义的失败日志也保留；最终结果只引用重编译后的文件。

## 真实日志说明了什么

原始日志已读取到本机：`outputs/diagnosis_v256_20261029_100719`。四个原始日志文件保持内容不变，额外保存当前板载配置、源文件副本和 `provenance.json`。当前板载指纹是读取时的状态，不应冒充该次运行时生成的构建清单。

1. 视觉状态在运行开始后约 38.636 s 进入 Stopped，监督器约 84.712 s 才退出 Forward；累计有 **46.076 s** 同时满足“视觉已停、监督器仍 Forward”。当前板载源码也确实缺少交接中最后那段本地停止补丁，不能把本地通过当作板载已经生效。
2. 前进请求是 0.20 m/s，但 `Supervisor::SpeedLimit` 和 `RunModel` 将实际策略速度限为 **0.15 m/s**。本轮没有提高这个上限。
3. `terminal_confirmed` 始终为 0。`tcp_rear_crossings` 表示观测缓存线段与后方工作区的交叉，**不是物理线尾到达证明**。此前交接中的“到达线尾区域”表述不准确。
4. 正确的 Z1 policy 索引是 **8、13、14、15、16、17**，不是 12..17；索引 12 是右后小腿。

跟踪阶段（`trial_stage=3 && tcp_active=1`，1791 个控制样本）的分解：

| 同一机身坐标系中的比较 | 平均三维距离 |
| --- | ---: |
| 实测关节 FK 相对真正 TCP 目标 | 9.30 cm |
| FSM 目标关节 FK 相对真正 TCP 目标 | 10.68 cm |
| actor 目标关节 FK 相对真正 TCP 目标 | 10.81 cm |
| 实测关节 FK 相对 FSM 目标关节 FK | 1.71 cm |

向量满足 `实测−目标 = (FSM FK−目标) + (实测−FSM FK)`，这些距离的标量不能相加减归因。actor 对齐使用控制周期开始前最近完成的推理，是诊断时间对齐，不是 Z1 发包或电机确认。

该阶段实际关节相对 FSM 目标的平均绝对误差约为 `[.00366,.01010,.00429,.00367,.00291,.00494] rad`。目标进入 actor 的位置观测在持续更新；actor 回放与记录一致；命令滤波没有造成同量级固定偏差；模型 FK 与原始遥测一致，且与 checkpoint 内的训练 URDF 一致。

因此，现有证据把主要误差定位到 **VA15 目标→actor 输出这一层的系统偏差**。它不支持把机械臂不响应或 Z1 增益被覆盖当成本次 8–11 cm 误差的主因。真实枪具外参、实际末端精度和完整动态响应尚无独立测量，不能据这些虚拟 TCP 结果宣称已标定。

V255/V256 的 SDK 运行路径已经由单一 I/O 循环写命令邮箱，直接设置 LOWCMD 请求字段；这里未再次调用会重置增益的 `setFsm(LOWCMD)`。本轮原有真实 SDK 无 I/O 测试也通过。实际 SDK 增益仍是现有 `kp=[20,30,30,20,15,10]`、`kd=[2000,...]` 的原始 SDK 数值，本轮没有调节，也没有将原始 SDK 数值冒充训练 PD 单位。

## 目标补偿试验：有改善，未作为修复交付

使用原 VA15、原关节控制、原始不变的 `[-.775,0,-.35]` 评分目标，仅在离线 harness 中试验缓慢积分的位置输入偏移。偏移速度最多 2 cm/s、总量最多 12 cm，策略输入仍限制在归档训练工作区。评分始终相对于原目标，没有把修改后的输入当作完成目标。

| 摩擦系数 | 原始平均误差 | 补偿平均误差 |
| --- | ---: | ---: |
| 0.5 | 10.17 cm | 8.15 cm |
| 0.8 | 10.13 cm | 8.20 cm |
| 1.0 | 10.11 cm | 8.22 cm |

评分窗口是第一次行走的 t=10..16 s。两次启停及最终 B 趴下均完成，但补偿改变了本体前进距离，且剩余误差仍大。因此仅保留可复现的离线实验，**没有把它合入 native 控制**，不能称作 TCP 精度修复。

## 下一步的真实未完成项

1. **已完成板载同步。** 新会话头文件与候选主源已一起部署，板载编译和启动脚本的离线检查通过。板载实际主源路径为 `src/rl_sar/src/rl_real_b2_z1_no_gun.cpp`；新头文件位于 `src/rl_sar/library/core/rl_sdk/`。本次未启动控制或运动，详细备份/指纹见同步记录。
2. TCP 精度仍须修复。已经排除这份部署 FK 与归档 URDF 不一致、目标没有进 actor、主因来自命令滤波或实际关节完全不跟随等解释。下一轮应针对原策略的末端跟踪能力做独立修正验证，或补充训练；不要继续用提高 Z1 增益、未确认线尾、或扩大到达阈值掩盖误差。小幅输入积分补偿已有反例，不应直接上线。
3. 当前 TCP 优先模式关闭了本体中心线转向，也绕过有限线尾完成器的 Observe/Tick。它是“直行本体 + 后方历史 TCP”候选，不能宣称完整沿曲线行走并在线尾完成。用户明确要求丢视觉停车，应保留这一语义；前视线尾离开画面后继续依靠缓存到后方的做法不能隐式恢复。
4. 本地 `hardware_qualified=false`、`precision_qualified=false`。旧目录的 HANDOFF_REPORT/VALIDATION/OFFLINE_REPORT 大量继承 V242/V254 内容，不是 V256 的新验收证据。本轮已更新本候选 `qualification.json`，不再引用那些历史结论冒充当前版本。

## 本机复现

```bash
cd /home/dong/rl_sar_b2_z1/patches/va15_tcp_priority_v256_20261028
cmake -S . -B build_v256
cmake --build build_v256 --target native_candidate_offline test_tcp_priority_session simulate_sessions -j1
ctest --test-dir build_v256 --output-on-failure
python3 verify_archived_tcp_geometry.py
```

日志分析：`python3 analyze_recorded_tcp.py /home/dong/rl_sar_b2_z1/outputs/diagnosis_v256_20261029_100719 <新的输出目录> --policy /home/dong/rl_sar_b2_z1/patches/va15_stop_feedback_20261020/work/b2_z1_no_gun/VA15/policy.pt`。

闭环复现：`build_v256/simulate_sessions tcp_invalid <新的输出.csv> 0.8`；其余场景是 `tcp_valid`、`tcp_pose_loss` 和仅离线的 `tcp_compensation_probe`。输出路径必须是新名字，避免覆盖历史证据。
