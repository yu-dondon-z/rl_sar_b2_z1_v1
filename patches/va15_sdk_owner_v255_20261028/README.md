# V255：单一 Z1 I/O 所有者实机候选

V255 在 V254 后方 TCP 视觉离线候选基础上，针对实机 `R1+Y` 抖动和重复启停问题，重构了 Z1 运行时通信所有权；完整候选已在板载当前 VA15 工程中编译。只读预检入口也已允许 `--disable-z1`，并补齐板载版本指纹和独立状态模型。

## 关键变化

- 启动时只用 SDK 后台线程完成一次 PASSIVE 握手；
- 运行时停止 SDK 后台线程；
- `real_z1_io` 以 500 Hz 独占 `lowcmd`、`recvState` 与 UDP `sendRecv()`；
- 策略/控制循环只提交邮箱帧，不再直接调用 `setFsm()` 或 `setArmCmd()`；
- 退出先提交 PASSIVE，等待 100 ms，再关闭 Z1 所有者；
- 未改变 VA15 权重、观测、动作滤波、B2 增益、接触门槛或后方 TCP 到达判定。

## 检查结果

- CTest 31/31：`outputs/ctest_v255.log`
- Python 67/67：`outputs/python_v255.log`
- 后方 TCP 19/19：`outputs/rear_endpoint_v255/summary.json`
- 原生无通信：`build/native_candidate_offline --offline-check`
- 板载只读预检：`--forward-trial --blind-fixed-target --disable-z1` 通过，未发布 B2 lowcmd。
- 板载视觉入口：`scripts/run_b2_z1_real_near_visual.sh eth0`；相机先启动，控制器随后建立 phase context，终点等待后方 TCP 到达前视确认的线尾。

板载同步和备份记录见：
`../../docs/VA15_V255_SDK_OWNER_20261028.md`。

当前仍不是硬件放行版本。板载只完成源码备份、替换、编译和只读预检；没有启动控制或机器人运动。
