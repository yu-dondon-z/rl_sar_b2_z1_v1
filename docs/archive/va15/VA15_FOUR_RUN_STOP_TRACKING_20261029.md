# VA15 四次日志：重复停车半蹲与跟踪复核

2026-10-29，Asia/Shanghai。本轮只读下载既有板载日志，在本地完成修复和验证。**新 FSM 和启动脚本尚未同步板载，未启动控制或机器人运动。** 板载仍为此前 V256 二进制 `084ed170fd4aa95e33193ebc3e1e041f9cc3fc0b67390834d18512bdc92c8c2e`。

重复启停中的半蹲已有可复现的软件原因和本地修复；TCP 精度和视觉拒帧来源仍有未完成项，不能宣布实机跟踪全部修好。

## 四次原始记录

本机原始副本为 `outputs/diagnosis_four_runs_20261029/`，内容未修改。目录时间为板载时间。下表高度为原控制器的模型高度下界，不是独立实测的真实高度。

| 运行时间 | 两段前进请求持续时间 | 真正启用后方 TCP 跟踪 | 停车结果 |
| --- | --- | --- | --- |
| 16:43:22 | 49.40 s、10.63 s | 合计 29.64 s；首次前进约 30.39 s 后才激活 | 首次完成；第二次接管约 0.44 s 后 Blocked，最低约 0.398 m |
| 16:50:41 | 2.70 s、2.17 s | 0 s | 首次完成；第二次接管约 0.58 s 后 Blocked，最低约 0.453 m |
| 17:01:43 | 0 s | 0 s，actor 记录数 0 | GetUp 期间故障锁存并回到 Passive，现有稀疏遥测不足以确认具体触发源 |
| 17:03:56 | 9.13 s、0.77 s | 0 s | 首次完成；第二次接管约 0.45 s 后 Blocked，最低约 0.400 m |

三次行走记录均为首次停车成功、重新开始后的第二次停车降低机身并锁住未完成的恢复指令。16:43 第二次停车后，后小腿实际角度由约 `[-1.547,-1.538] rad` 变为 `[-1.987,-1.960] rad`，关节记录确实显示后腿弯曲。

16:43 的 commands/control 尾部分别有一行含 NUL 的损坏记录，summary 为空；16:50 的 summary 全为 NUL。分析只排除明确不完整或含 NUL 的行，不补造记录，也不推断文件损坏的具体原因。原文件指纹和排除计数全部保留。

## 半蹲原因与修复

`RearmVisualSession()` 原来清除了停止控制器，却没有把 `leg_offset_scale` 从上次行走遗留的 0 恢复为站立状态的 1。

native 顺序是视觉重新开始并 Rearm → FSM 运行 → 写入自主前进请求。因此重新开始的首个 FSM 周期仍为零速度。旧代码在这个周期保持位置目标，却把既有站立增益 450/675 降成接近策略增益 160；随后 `ObserveStanding()` 又把低支撑指令估计为下一次停车的站立力矩参考。下一次停止在约 0.3 s 内淡出动态力矩后转向这个过低参考，机身下降，触发原高度条件，进入 Blocked 并冻结半途恢复姿态。

真实 17:03 记录中，重新开始前右前小腿指令估计支撑力矩约 43 Nm，首次记录到的低增益帧约 12 Nm。遥测采样约 20 ms，未直接记录所有 5 ms 周期；中间的软件顺序由同一代码的离线复现验证。这是指令力矩估计，不是电机确认。

本地修复只在候选 `production/fsm_b2_z1_no_gun.hpp` 的 `RearmVisualSession()` 恢复 `leg_offset_scale=1.f`，让等待周期保持已完成的站立指令，再由原 torque-continuous handoff 接管新 actor 帧。没有调整增益常数、接触结果、高度阈值、帧时间或 VA15 权重。

新增 `test_restart_support.cpp` 覆盖上述零速度间隔、延迟 actor 和下一次停车。数据明确标注为离线合成负载状态。相同回归对旧 FSM 失败，对新 FSM 连续三次重新开始通过，交接瞬间最大指令力矩差约 `7.63e-6 Nm`。

此前仿真在 FSM 运行前就给前进请求，跳过了关键周期。此次新增 `tcp_native_order` 场景按 native 顺序运行，使用同一策略、原 FSM、独立估计器与合成视觉，对比旧/新 FSM：

| 离线条件 | 第二次停车最低物理高度 | 第二次停车完成 | 最终 B 趴下 |
| --- | ---: | --- | --- |
| 旧 FSM，摩擦 0.8 | 0.382611 m | 否 | 否 |
| 新 FSM，摩擦 0.8 | 0.492634 m | 是 | 是 |
| 新 FSM，摩擦 0.5 | 0.501073 m | 是 | 是 |
| 新 FSM，摩擦 1.0 | 0.500166 m | 是 | 是 |

当前测试重新编译后 **33/33 通过**，额外明确拒帧会话也完成重新开始、停车及 B→Passive。这里是故障机制的离线闭环复现，尚非实机验收。

![相同 native 顺序的仿真对照](../../../patches/va15_tcp_priority_v256_20261028/outputs/four_run_review_20261029/native_stop_comparison.png)

## 跟踪不正确及丢失

1. **两次没有进入后方跟踪阶段。** 16:50、17:03 在观测轨迹进入后方工作区前就停止，全程 `tcp_active=0`。机械臂仍收到预设目标，没有开始沿后方观测轨迹工作。16:43 首次前进也等待约 30.39 s；前视观测点必须随本体前进转到后方，当前没有当前位置后方的初始裂缝观测。
2. **真正跟踪时存在策略层偏差。** 16:43 有效阶段，实测关节 FK 相对输入 TCP 目标平均偏差约 9.33 cm，FSM 目标 FK 偏差约 10.72 cm，最近已完成 actor 目标 FK 偏差约 10.86 cm；实测 FK 相对 FSM 目标 FK 差约 1.83 cm。部署 FK 与日志数值同时校验一致。这继续支持目标到策略输出已含主要偏差，不支持把机械臂完全不响应或盲目加增益作为解释。距离范数不能相减分摊原因，虚拟枪尖模型也不等于实枪标定。
3. **本体没有沿曲线转向。** 当前 TCP 优先模式将本体 yaw 请求固定为 0，虽然视觉建议转向。有效跟踪阶段本体横向参考偏差最高约 0.131 m，观测枪尖目标横向位置最高约 0.293 m，超过原策略 ±0.15 m 工作区；约 11.94% 的跟踪样本发生工作区限制。这份候选不能作为完整曲线行走与线尾完成的实现。
4. **停止不符合旧有效帧超过 500 ms 的情况。** 六次停止时保留的最后有效帧年龄约 0.23–0.33 s，局部位姿仍有效，未见 B/X 标记；视觉资格清零并终止会话，与当前收到明确无效包即终止的路径一致。本次四份记录中，视觉 Stopped 同时仍 Forward 的持续重叠时间均为 0，旧版本绕行停止的问题没有继续出现。这里遥测的是 tolerant guide 保留的旧帧，不能将它冒充触发停止的新包。

旧启动脚本退出时删除整个临时交换目录，连 `guide_packets.jsonl`、拒绝原因和 before/after 图像一起丢失。四次目录只保留控制遥测。**现存数据无法确认具体视觉拒绝来自分割、关联、路径长度、处理超时还是其他原因。** 不把推测写成证实，也没有延长时效或将无效帧标为有效。

本地 `scripts/run_b2_z1_real_near_visual.sh` 已将交换目录放进每次输出目录并保留，另存 `camera.log`、`controller.log`。原有有界诊断线程和最多 20 次拒帧图像机制保持原样；下次可对照真实包与原图定位。此次没有修改视觉模型或拒帧标准。

## 证据、复现与未完成项

全部结果位于 `patches/va15_tcp_priority_v256_20261028/outputs/four_run_review_20261029/`：

- `before/`、`before.json`：原源文件；旧 V256 的离线及板载同步快照保持不动。
- `analysis_verified/analysis.json`、`recorded_stops.png`：四次实测分析、源指纹、损坏记录计数。
- `restart_regression_before_final.log`、`restart_regression_after_final.log`：同一测试的旧版失败与新版通过。
- `physics/summary.json`：旧版一组失败、新版四组通过，附完整命令、CSV 和日志。
- `ctest_final.log`：当前 33/33，含 native 离线检查。
- `board_readonly_provenance.json`：本轮读取时板载指纹，不冒充运行当时的构建清单。
- `manifest.json`：本轮源码、本地二进制及验证产物指纹，明确未同步、实机未验收。

```bash
cd /home/dong/rl_sar_b2_z1/patches/va15_tcp_priority_v256_20261028
cmake -S . -B build_v256
cmake --build build_v256 --target test_restart_support simulate_sessions native_candidate_offline -j1
ctest --test-dir build_v256 --output-on-failure
build_v256/simulate_sessions tcp_native_order /tmp/va15_restart_new_result.csv 0.8
/usr/bin/python3 -s analyze_four_runs.py /home/dong/rl_sar_b2_z1/outputs/diagnosis_four_runs_20261029 /tmp/va15_four_runs_new_analysis
```

结果路径必须尚不存在。旧版比较可通过 CMake 参数 `VA15_RESTART_BASELINE_FSM` 指定 `before/` 内的原 FSM，构建 `test_restart_support_before` / `simulate_sessions_before`；旧版不能作为部署候选。中间未完成测试夹具的失败、默认 Python 的 NumPy/Matplotlib 不兼容记录未作为最终结果，最终分析使用系统 Python 的 `-s` 环境。

下一步按用户新的同步授权更新板载 FSM、启动脚本、重编译及离线自检；运动仍按现场状态和授权处理。TCP 策略偏差、本体曲线跟踪尚未修复；感知具体拒绝来源须从新保留的原始包与图像确认。继续遵守不盲调增益、不伪造接触或帧时间的约束。
