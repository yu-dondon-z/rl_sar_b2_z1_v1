# VA15 TCP-priority candidate V2.5.6

> 2026-10-29 更新：当前语义是明确拒帧/过期立即结束整个视觉会话，恢复视觉不会自行续走；下文早期缓存继续运行描述已被取代。请求速度 0.20 m/s，actor 实际上限仍为 0.15 m/s。最新本地修复、32 项检查、闭环测试及仍未解决的精度问题见 `../../docs/HANDOFF_VA15_V256_REVIEW_20261029.md` 和 `outputs/local_review_20261029/`。补偿仅为离线试验，当前修复未同步板载。

> 后续已获用户授权同步：当前修复已在板载完成编译和离线自检，未启动控制；见 `../../docs/VA15_V256_BOARD_SYNC_20261029.md`。`qualification.json` 继续保留为原离线快照，部署状态以本次同步记录为准。

## 目标

本候选把两条控制链路解耦：

- B2 不再根据裂缝中心线做横向和航向修正，只以 0.08 m/s 低速直行；
- R1+Y 建立会话后，前置视觉继续提供已观测路径；
- 后方路径进入 Z1 工作区后，`visual_tcp::Tracker` 持续更新 `target_commands`，由原 70 维 VA15 全身策略驱动末端；
- TCP 数据短暂丢失时保持最后目标并停止本体推进，不清空末端目标；
- T、X、B、P 或策略待机失效时结束 TCP 会话。

## 关键修复

旧链路中 NearTracker 失效会直接结束整个视觉会话，随后 `tcp_tracker_.Step()` 的 `moving` 参数变为 false，TCP 目标被冻结；而当前主程序还没有把 V255 的视觉候选接入默认 real binary。

V2.5.6 在候选源中增加 `tcp_priority_session_`：

1. R1+Y 成功后独立保持 TCP 会话；
2. TCP tracker 不再依赖 NearTracker 是否仍处于 Running 阶段；
3. 前置线的角度、横向和航向误差不再写入 B2 速度指令；
4. `SetRuntimeTargetCommands()` 持续写入 TCP 目标，保持原 70/18 策略接口；
5. 停止键只冻结当前 TCP 目标并停止本体，不伪造新目标。

## R1+Y 首次按键修复

旧实现会在策略刚进入待机、四脚支撑统计或视觉帧数量尚未满足时直接消耗
R1+Y 事件，因此常见现象是第一次、第二次没有动作，第三或第四次才启动。
V2.5.6 现在即使遥控输入层仍处于 Y 后的 `wait_center` 阶段，也会接收并把一次
启动请求暂存最多 5 秒；摇杆回中、策略待机、支撑状态和视觉帧满足后自动启动
视觉会话。日志会显示 `queued during neutral gate`、`start request latched`
和 `latched session started`。X、B、Space、P 以及策略状态退出仍会立即清除
暂存请求。

## 当前边界

仍使用单目平地投影，TCP 目标不是深度测量；实际 TCP 是否到达需要在仿真和实机分别验证。

## 离线验证

候选工程已在 `build_v256` 完整编译通过。以下检查通过：

- `native_candidate_offline --offline-check`
- `test_repeatable`
- `test_tcp`
- `test_rear_completion`

这些检查覆盖模型输入输出、连续会话重启、TCP目标更新、后方终点处理和 X/B/T 停止路径；它们不代表已经完成实机验收。

## Armed 状态死锁修复（2026-10-28）

板载日志确认 `raw_keys=2049`（R1+Y）已经收到，视觉会话计数也递增；但试验监督器长期停在 `Armed`，策略没有进入 `Forward`。原因是速度请求只在 `Forward` 条件成立后才给出，而 `Armed -> Forward` 正需要下一次 FSM 运行中的正向请求。

候选源现已在 `Armed` 和 `Forward` 两个阶段都保持 0.08 m/s 的 TCP-priority 正向请求，使监督器能够完成状态切换并启动策略。`test_trial`、`test_repeatable`、`test_rear_completion` 和完整离线编译均通过。板载同步等待 SSH 恢复；当前未启动控制。

实机日志又确认第二个问题：运行中再次收到 `raw_keys=2049` 会被误当成停止请求。现在 R1+Y 在视觉会话运行期间被忽略，停止统一使用 X、B 或 Space，避免遥控器重复报文中断行走。

随后日志显示控制确实接管，但 `requested_0=0.08` 时腿只做近似站立修正，实际没有形成有效平移。TCP-priority 本体速度已调整为 `0.20 m/s`；横向和转向仍固定为零，便于先验证前进和 TCP 接管。

## 板载同步记录（2026-10-28）

已同步到 `unitree@192.168.123.164:/home/unitree/rl_sar_b2_z1`，并先备份原文件到：
`deployment_backups/va15_tcp_priority_v256_20261028/rl_real_b2_z1_no_gun.cpp`。

Armed→Forward 修复前的源码另备份为：
`deployment_backups/va15_tcp_priority_v256_20261028/rl_real_b2_z1_no_gun_before_armed_fix.cpp`。

- 当前板载二进制 SHA-256：`8b38e9f6d9897e7e8245b227a0599d415f1b95c7ebfcc8599ff4c4e184b95e16`
- `--offline-check`：通过
- 同步和检查过程中：未启动控制、未释放 B2 factory controller、未发送运动命令

