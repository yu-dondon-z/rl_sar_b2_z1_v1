# V2.4.1：启动误停与 B 退出卡住（2026-10-23）

## 本次实际记录

PC4 `outputs/va15_near_walk_epqdqdj6`，会话 bundle `ce1340a2bd0e5a8096048073e72d528ad5d3618cf209725c17946388038fbced`。原始记录保留，开发电脑副本在 V2.4 的 `outputs/review_epqdqdj6`。

- 胶带有效，Visual gate 显示 curve_ready/quiet/neutral/health 均为 1。第一次请求时 supervisor 尚未 ARMED；第二次确实开始，不能说遥控器没收到。
- 正向请求累计约 0.085 s；策略速度命令最大仅 0.0144 m/s，之后 `local pose stale/unreliable; TCP target frozen`，因此未能真正走起来。
- 停止捕获后进入 `stop_stage=5`（Blocked）；紧接的一条记录 estimate_valid=0，后续数据恢复仍永久 Blocked。多次 B 收到，但没有 GetDown→Passive。
- 这次终端最终记录 `2026-10-23 15:26:11+08:00 COMMAND_EXIT_CODE=0`，控制循环、Z1 PASSIVE 和 recorder 均完成。排查时已无控制/相机主进程，仅独立 Z1 服务仍在。B 趴下卡住和进程最终退出需要分别判断，不把 Ctrl+C 当安全趴下方法。

## 程序修复

1. **一致时间快照**：接收线程可能在控制周期开始后更新位姿。旧代码用 control_begin 检查更晚的源时间，`now >= source_time` 会失败。新 helper 在获取源互斥锁之后再读单调时钟；不容许未来时间、不扩大 100 ms 新鲜度或 250 ms 惯性预测窗口。图像包也在该快照时刻校验。dt 仍来自相邻控制周期开始时间，避免把锁等待重复计入。
2. **模型置信暂缺与真实故障分开**：捕获之后，若 DDS/原始姿态仍健康、只是独立足端模型没有足够测量更新，则保持捕获指令、停用 actor、重新累计安静时间，不降低、不捏造四足接触，不永久锁住 B。恢复后仍必须原四足/高度/姿态/速度条件全部合格才允许趴下。真实 stale、无效状态或异常倾斜仍锁止。
3. **明确诊断**：启动门新增 local_pose_valid、trial_stage、stop_stage；去掉“NOT metric TCP / TCP unchanged”等旧版误导性文字；控制退出和相机关闭分别提示。没有强杀控制器或 Z1 服务的逻辑。

这两个缺陷可在离线回归复现；本次未新增高速、增益或接触阈值放宽，没有更换 VA15 权重。原同步仿真没有覆盖“回调发生在 control_begin 与锁获取之间”的线程交错，本版新增确定性双线程测试，不能只靠之前的同步物理仿真判定实机通过。

## 新验证

本轮组件、物理回归和板载自检的完成状态、源码/权重指纹记录在 qualification.json。实机运动与精度仍未验收，不能用离线通过替代实机结果。

## 手动操作（仅本版通过最终自检后）

PC4 NoMachine 桌面终端，B2 已实际趴稳、旧程序退出、无枪和软管：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_tcp_v241_20261023/run_visual.sh --offline-check
bash diagnostics/va15_visual_tcp_v241_20261023/run_visual.sh eth0 --walk
```

输入大写 RUN → A 起立 → 实际站稳 → Y 待机 → 轨迹点确在胶带上且 PATH IN RANGE → R1+Y 开始。先短程验证 X 停止及 B 趴下；实际趴稳、Passive 后才 Ctrl+C。Ctrl+C 不是站立时的安全停止或自动趴下操作。

若提示 trial_stage=1，说明仍在准备中，等 READY/ARMED 后明确重按一次；不缓存提前请求。正常已准备后不需要窗口 R、额外四秒静态审查或程序重启。TCP 仍需走过前方已经观察到的路径，才会有后方可达历史点并 active=1。

旧 V2.4 目录和会话保留用于复查，但请勿再用旧命令开展行走。当前仍仅适用于平地、无枪无软管实验；单目无滑移估计、约 8 cm 离线完整 TCP 误差等限制未由本修复消除。
