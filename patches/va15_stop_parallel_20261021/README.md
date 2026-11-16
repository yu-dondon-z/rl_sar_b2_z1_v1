# 无枪/无软管并行趴下候选（2026-10-21）

## 范围与适用条件

用户明确确认：B2 已趴下、控制程序已退出；没有焊缝枪/软管，收臂路径无遮挡。
本版本只适用于该条件，程序**不能自动识别是否装枪、周围是否有障碍**。
保留旧版并可回滚。没有自动起立、行走、开启视觉、发送 Z1 控制或结束现有服务。

板载生产运行时仅改 `RLFSMStateGetDown`；主 .cpp 仅调整离线回归函数，
不改正常运行分支。manifest 仅更新对应 FSM SHA-256，不改变任何资格状态。
VA15 model_5000、配置、增益、70/18 布局、足端阈值、lost 拒绝、停止控制器、
起步门槛、Z1 有限退出路径均不变，动态实机运动继续受限。

## 新流程

原先收臂 2 秒后才开始下降 2 秒；现在机械臂和 B2 **同时**按 3 秒
smoothstep 过渡。峰值目标变化率不超过旧 2 秒线性过渡，而非简单加速下降。
首帧完整保持上一帧 q/dq/kp/kd/tau，随后平滑过渡，避免 PD 支撑力矩突变。

- 机械臂相对上条目标的跟踪落后 >0.20 rad 时暂停机身下降，机械臂继续收回。
- 两部分目标完成、实测机械臂误差 <0.12 rad、腿误差 <0.18 rad、各关节
  速度 <0.5 rad/s 连续 0.20 秒后，才切入 Passive。
- 无效输入或 10 秒未完成：保持最后有效命令并告警，不以计时强制 Passive。
- P/L1+X 紧急被动优先级不变。软件判定不能替代实际趴稳和操作员急停。
- B 前的动态停稳过程没有移植其他候选，也没有缩短四足停稳许可窗口。

## 离线验证

本机 CTest 4/4；原生 VA15、无工具物理仿真 30/30（25 正常、5 故障）。
报告：`outputs/parallel_full_final/summary.json`、`comparison.json`。
比较脚本还核对了：生产/仿真 GetDown 类一致，FSM 其他类逐字不变，主程序
正常运行代码逐字不变，30 场景进入 GetDown 前所有共同 CSV 字段逐字相同。

**范围区别：**30 项仿真沿用上一轮 v3 离线估计停止候选作为 GetDown 的
前置控制器；生产只移植并行 GetDown，没有把 v3 的减速/捕获几何移植到板载。
板载当前只准备静态 A/B 检查，不能据此放行真实动态行走停止。

无工具仿真对照：

| 指标 | 旧串行 | 新并行 |
| --- | ---: | ---: |
| 待机 B 至趴稳（含末尾 2 秒检查） | 6.015 s | 5.220 s |
| 25 正常场景最长 B 至趴稳 | 11.600 s | 10.805 s |
| 25 正常场景最大倾斜 | 2.754° | 4.233° |

新方案并非所有稳定性指标都改善；下降过程最大倾斜略增。
检查了活动机械臂几何与地面/B2 机身碰撞代理：无 >2 mm 穿透，最小间距
约 0.239 m / 0.086 m。没有覆盖实机外参误差、软管、工具、全部自碰撞或障碍物，
也没有把视觉网格与碰撞代理混为一谈。这些都不是实机碰撞/运动验收。

```bash
cd /home/dong/rl_sar_b2_z1/patches/va15_stop_parallel_20261021
ctest --test-dir build --output-on-failure
python3 analyze_results.py
```

## 板载同步及复核

暂存：`/home/unitree/rl_sar_b2_z1/diagnostics/parallel_getdown_20261021_mG85ez/`。
最终旧版本备份：
`/home/unitree/rl_sar_b2_z1/deployment_backups/parallel_getdown_20261021_4SqEFL/`。
首次尝试的备份 `parallel_getdown_20261021_vedaRZ` 也保留，未删除材料。

最终板载编译、launcher `--offline-check` 通过；native golden 最大误差
1.14441e-05。4 项板载内存测试通过：并行下降边界、真实启动/停止门槛回归、
既有独立停止控制器、既有增益渐变。无 DDS/UDP/机器人初始化。
日志在暂存目录 `install_checks_ePpXJf/`，本地副本见 `outputs/board_final/`。

首次还尝试了旧 `scripts/test_va15_stop_fsm.cpp`，在 line 105
`state.policy_running` 断言失败，更新脚本自动回滚了源码/清单/二进制。
随后使用**未修改生产旧版**重新编译复现同一断言失败；其 Ready fixture 缺少
有效 IMU 四元数输入。该旧测试未修改、不计为通过，也未修改生产来迎合它。
新增 `production/test_parallel_stop_gate.cpp` 用显式有效/无效内存输入重新检验
门槛、保持请求不能自动重启、失效动态停止不能强制下降、待机 B 及紧急优先。

最终生产来源：

```text
FSM    5ff38043ccab5ceeaeec09bf5828ac26da2b093a72a725d9cd43f132a0a91d94
main   6cff6770d520e95ef2cd395959bacfbcffacf6c19da185329787339f01666a6f
config 8faa51628b3704ce3e7e3a384f1c0eb8bb59b7dcf5a0069992069efed8aab8d1
actor  8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098
```

## 下一次只做静态 A/B，不做行走

操作者再次检查无枪/软管、路径空旷、遥控急停可用。在**板载终端**：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
./scripts/run_b2_z1_real.sh --offline-check
./scripts/run_b2_z1_real.sh eth0 --run
```

输入大写 RUN，A 起立并等待站稳；不按 Y/R1+Y，不推摇杆。B 应使收臂与
下降同步；若提示 arm tracking lag，观察暂停而不要强制推进。实际趴稳、
看到 Passive 后再 Ctrl+C。若动作异常，停止测试，按既有安全/急停流程处理。
若末端仍装枪/软管，不使用本候选作并行趴下测试。

`lost=5` 的语义依然未获厂商确认；独立的第二轮只读核查见
`../va15_lost_audit_v2_20261021/README.md`，不是已解决或允许行走的结论。

### 操作者后续反馈

2026-10-21 操作者反馈此前测试正常，记为并行趴下的人工静态反馈，
不扩大为动态行走/停止的硬件验收。随后只读复核未更改该版本；当前
许可链与需厂商确认的问题见 `../../docs/B2_MOTION_READINESS_20261021.md`。
