# VA15 实机部署：本帐号暂停交接

更新时间：2026-10-27 15:55，Asia/Shanghai。用户明确要求暂停排查，切换回上一个帐号。本文件是最新接手入口；原始交接、代码、报告和日志均保留。

## 1. 接手时最重要的状态

**实机横摆、真实执行器响应不匹配和 FR 模型支撑卡住尚未被证明修复。V252 仍为本地未发布候选，未同步板载。不要将下面的单测或名义仿真当作本次实机故障修复验收。**

用户保留的限制：没有重新确认机器人当前状态并取得明确授权，不得同步板载、启动控制或机器人运动。本帐号没有进行 SSH、板载写入、DDS/UDP 初始化、相机启动或机器人控制。此前用户说没有支架、暂时无法现场测试；历史“已趴稳、程序退出”不能作为当前状态确认。

本次暂停没有新增控制补丁。最后进行的是本地 SDK 头文件/反汇编只读审计。原有子任务均已完成或中断；暂停时本机进程检查没有匹配到本轮模拟器、原生候选、Z1 测试、strace 或分析脚本的运行进程。本机检查不代表板载状态，板载未访问。

工作仓库是 `/home/dong/rl_sar_b2_z1`，不是当前可能显示的 `/home/dong/b2_z1`。磁盘约剩 6.7 GB，99% 使用率；不要复制大型依赖或清理旧记录。

## 2. 建议阅读顺序

1. 原交接：[HANDOFF_VA15_REAL_V252_20261027.md](HANDOFF_VA15_REAL_V252_20261027.md)。其中的 V251 故障链、资格限制和原报告仍有效。
2. 本帐号持续工作记录：[VA15_V252_OFFLINE_TAKEOVER_20261027.md](VA15_V252_OFFLINE_TAKEOVER_20261027.md)。包括测试、停止恢复对照、SDK 竞态及策略重放的详细说明。
3. V251 原始证据：`/home/dong/rl_sar_b2_z1/outputs/diagnosis_v251_yumjt3ry_20261027/DIAGNOSIS.md`，同目录 `control.csv`、`commands.csv`、`actor.csv`、终端/视觉记录。
4. V252 原验证：`patches/va15_stop_recovery_v252_20261027/VALIDATION.md`、`qualification.json`、`BOARD_SYNC_REPORT.md` 以及 `outputs/startup_analysis/` 下五份审计报告。
5. 最新策略/仿真比较：`outputs/va15_actor_mujoco_nominal_20261027/SPECTRAL_REVIEW.md`、`ACTOR_SENSITIVITY_REVIEW.md`。

以下路径如未写绝对路径，均相对 `/home/dong/rl_sar_b2_z1`。

## 3. 本帐号实际做了什么

### 3.1 停止恢复单测和诊断补充

候选目录：`patches/va15_stop_recovery_v252_20261027/`。

- `test_planted_hip_recovery.cpp`：补充实际使用的独立模型反馈分支，显式注入 1011 持续 16000 个 5 ms 周期，验证不得 Ready；恢复四脚仍需连续 0.5 秒，再次丢接触重置计时。这些是测试输入，不是真实足端接触。
- 同测试补充历史髋角/近似静态请求力矩反事实与首帧请求力矩连续断言；它不是物理动力学回放。
- `test_z1_gain_order.cpp`、`CMakeLists.txt`：增加假 SDK 缓冲契约模型及 CTest，表达 PASSIVE→LOWCMD 重置增益的行为。**此测试不链接真实 SDK。**
- `production/rl_real_b2_z1_no_gun.cpp`：仅增加 LOWCMD 后缓冲 Kp/Kd 快照，将日志 `kp_scale` 改为 `requested_kp_scale`；没有移动增益写入顺序，也没有改变实际增益数值。打印不代表电机确认，存在少量启动时序开销，仍未实机验收。

未改主目录默认程序、VA15 权重、70 维观测、raw-action 历史、EMA、接触门槛或视觉 500 ms 时效。

### 3.2 补充停止链路物理对照

离线模拟器 `simulate_recorded_stop.cpp` 增加/使用稳定站立及缓慢前髋目标偏移场景，未重写接触判定。结果保存在：

- `patches/va15_stop_recovery_v252_20261027/outputs/supported_stand_20261027_1330/`
- `patches/va15_stop_recovery_v252_20261027/outputs/supported_wide_large_compare_20261027_1400/`

普通稳定四脚起点：V251/V252 × truth/estimated 四组均可停止并趴下，结果相同，不能证明 V252 改善。

宽站姿对照：缓慢给前髋施加 +0.14/-0.14 rad 静态目标偏移，MuJoCo 物理四脚接触保持、最大倾角约 0.0079 rad；V251/V252 的 estimated 分支均 40 秒内未捕获、未下降、未趴下，而 truth 分支均通过。证明当前模型支撑门控确实可能阻止恢复，但不是实机 FR 真实接触证明，也不是修复通过。新髋参考只在有效捕获时冻结，不能解决捕获前就被拒绝的问题。

### 3.3 复核 V251 原始停止数据

`stop_stage=3` 的 3430 条记录约覆盖 80.547 秒，全部 `stop_gate_mask=4`、contacts=13，显示序 FL/FR/RL/RR 为 1011，缺 FR。其他姿态、速度、新鲜度等门控均未置失败位，quiet 计时始终为零。

原始日志没有独立足端力真值及完整原始加速度/估算受力诊断，不能区分 FR 真实卸载与受力模型偏差。不能因机身静止就伪造 FR=true。两版最终 Settled 门槛相同，原样读取这段稀疏日志都会等待，不构成 V252 闭环回放。

### 3.4 真实 VA15 actor 重放和敏感性分析

新增脚本：`patches/va15_stop_recovery_v252_20261027/analyze_actor_sensitivity.py`。

对 V251 `actor.csv` 的 545 帧原始 70 维观测重跑同一 VA15 TorchScript，raw-action 最大误差约 2.14e-6，RMSE 3.06e-7。输出与记录一致，不支持策略权重错配、观测列错位或 raw-action 串台的解释。原始 CSV 是约 47 Hz 采样，不是完整 200 Hz 写出流。

在启动后 2–10 秒，以原时间戳做 5.875 Hz 正余弦拟合（含常量和线性趋势、不插值）；单独从观测组中移除该频率成分，再做单步 actor 求值。仅移除 gyro_z 成分时，左右大腿 raw-action 对比幅值约从 1.680 降至 0.878，小腿从 0.580 降至 0.133；上一帧动作去掉该成分反而使大腿振幅上升。

这只是非线性策略的局部输入敏感性。反事实破坏输入相关性，不传播物理状态，也不保持闭环历史，不能证明 gyro_z 是根因或据此添加滤波。TCP 目标与速度命令基本没有对应频率成分；本次动态后方 TCP 从未激活。

### 3.5 名义 MuJoCo 对照

离线 `simulate_trial.cpp` 增加有符号横滚、gyro 三轴、左右大腿/小腿目标和实测角诊断列，未改控制逻辑。使用同一 VA15，0.15 m/s 请求约 4 秒后走停止/趴下流程。输出：

`outputs/va15_actor_mujoco_nominal_20261027/steady_015.csv` 及 JSON；诊断列版本是 `steady_015_spectral.csv` 及 JSON。

未跌倒，最大倾角约 0.02568 rad，请求结束时前进约 0.481 m、横向约 -0.103 m；hardware_qualified=false。相同 5.875 Hz 下仿真左右大腿目标仅 0.003368/0.003560 rad，实机约 0.0618/0.0602 rad，约相差 17–18 倍。名义仿真没有忠实复现实机故障，不能据此验收增益/滤波补丁。

真实 actor 目标与约 14 ms 对齐、完成关节映射的发送目标相关系数 0.980–0.997；实测关节同频幅值仅约为发送目标的 0.0016–0.108，且相位不一致。当前记录不能唯一分离执行器响应、测量时序、负载或策略-机体闭环原因。已有一阶拟合 0/12 关节通过留出验证，不可使用拟合参数调实机。

诊断版模拟器在 `/tmp/va15-spectral-build.XpIfNw/simulate_trial`，暂停时目录仍存在；临时路径不保证重启后保留，源码已保存在候选目录。正式 `build/simulate_trial` 不应被默认认定已包含最新诊断列。

## 4. 最新 Z1 审计：已确认范围与未做事项

### 已读代码与二进制

SDK：`src/rl_sar/library/thirdparty/z1_sdk/lib/libZ1_SDK_x86_64.so`，SHA-256：

`ecfbdd917016de721983ad2cc0770adb110832f506cc9ef67b8c37a9f5f3eafd`。

原交接的 `Z1_GAIN_ORDER_AUDIT.md` 已记录此前板载同哈希和库解析检查；本帐号只核验本地，不能恢复历史加载环境或证明当前板载进程状态。补充工作记录中较早的“板载实际加载仍未核验”应理解为本帐号未重新核验，勿覆盖原交接已有的只读审计结论。

V251/V252 应用逻辑先将 Kp 乘 0.35，再调 `setFsm(LOWCMD)`。实际 .so 中该函数在 PASSIVE→LOWCMD 时调用无参 `setControlGain()` 和 `setGripperGain()`，覆盖之前缩放。默认缓冲 Kp 为 [20,30,30,20,15,10]、Kd 为 2000 各轴；物理单位未经确认，不能等同训练 160/5。

SDK 后台 500 Hz `sendRecvThread` 与应用写入同一公开 lowcmd 向量，未见共享锁。SDK `setFsm` 会修改状态/增益并等待 recvState 匹配，等待期间不自己推进 sendRecv、未见超时。停掉后台线程再调用阻塞 setFsm 会失去握手推进；保持线程运行再后写增益也未解决共享缓冲并发。因此没有把增益写入简单挪到 LOWCMD 之后。

另发现再启用路径风险：应用停用分支和 `SendPassiveCommands` 只调 `lowcmd->setPassive()`，其作用是清 dq/tau/Kp、设被动 Kd，不请求 FSM 回到 PASSIVE。实际 SDK 的 `setFsm(LOWCMD)` 若 recvState 不为 PASSIVE，会把 sendCmd 设为 PASSIVE、打印 `Only state_passive could tranfer to state_lowcmd` 并返回 false。因此同进程内停用后再启用可能失败。它是独立的软件路径风险，未证明导致 B2 横摆。

关键应用位置：`production/rl_real_b2_z1_no_gun.cpp` 的 Z1 构造初始化、`SetCommand` 激活/停用分支（约 1030 行起）和 `SendPassiveCommands`（约 2445 行）。退出辅助在 `patches/va15_stop_feedback_20261020/work/va15_z1_exit.hpp`，尚需继续审查完整退出/超时语义。

### 中断点：真实 SDK 无 I/O 测试尚未实现

暂停前我告诉用户，准备“加载实际 SDK、用内存状态确认代替 UDP、禁止网络和线程”做更强验证。**这只是接下来的方案；测试文件未创建、CMake 未加入目标、程序未编译或运行，seccomp/strace 也未执行。不能说已经用真实 SDK 动态复现。**

目前真实 SDK 证据仍是头文件与 objdump 审计；现有 `test_z1_gain_order` 仍为假 SDK 契约测试。

已核对可行性：`CtrlComponents()` 默认构造创建 lowcmd/lowstate；需显式初始化 dt、recvState、armModel=nullptr、udp=nullptr。`unitreeArm(CtrlComponents*)` 创建 LoopFunc 对象，但应用调用 start 前不会启动收发线程。其析构释放 CtrlComponents，不能重复释放。不要调用带网络默认初始化的重载构造，不要构造 UDPPort，不要调用 sendRecv 或 start。

拟议方法供接手者重新审查：测试进程先安装禁止 socket/网络及 clone/fork 的 seccomp，链接哈希固定的实际 SDK；仅在测试可执行文件中拦截 SDK 等待所用 `nanosleep`，明确合成 FSM 状态确认，测试增益覆盖、setPassive 不改 FSM、LOWCMD 再启用拒绝、显式 PASSIVE→LOWCMD 多轮转换。用 strace 佐证无网络/线程。合成握手只能证明软件分支，不是机械臂确认，严禁编入生产程序。若无法确保无 I/O，不运行。

## 5. 已有验证及其边界

| 检查 | 结果/记录 |
| --- | --- |
| 本帐号 C++/native 离线回归 | 25/25，`patches/va15_stop_recovery_v252_20261027/outputs/ctest_takeover_20261027_110448.log`；14:59 又完整通过，`build/Testing/Temporary/LastTest.log` 会被下一次 CTest 覆盖 |
| 本帐号 Python 回归 | 62/62，`outputs/python_takeover_20261027_1115.log`（候选目录内） |
| 原生候选 --offline-check | 通过，golden 误差 0；不会初始化机器人接口 |
| 静态站立停止 | 4/4 通过，但新旧相同 |
| 宽站姿估计反馈 | 新旧均未捕获，未修复 |
| actor 545 帧重放 | 数值一致，见最新敏感性报告 |
| 实际 SDK 无通信动态测试 | 未实现、未运行 |
| 实机、板载同步 | 未做 |

暂停整理只做文件/状态核对，不重新跑控制或仿真。qualification.json 原样保留：simulation_pass、walk_release_allowed、blind_test_release_allowed、board_sync、hardware_qualified、live_robot_tested、real_failure_reproduced_in_physics、startup_oscillation_resolved 均为 false。文件内 24/24 是原历史报告，不要将其改标为最新实机验收。

## 6. 工作树与指纹

Git HEAD 仍按原交接的 main/68a9e979665197e3ef146ff5b0906ec402840986 管理。暂停时 `git status --short`：

```text
 M .vscode/settings.json
?? docs/
?? outputs/
?? patches/
```

大量工作是未跟踪文件，Git diff 不能完整展示。`.vscode/settings.json` 是原有修改未动。没有 commit/push、重命名仓库、覆盖旧版本或删除日志。

本次核对 SHA-256：

```text
9538cda2a4513f5b439beacd44ce6ca93255afcb5274e16fd6df467b22ac74ad  src/rl_sar/src/rl_real_b2_z1_no_gun.cpp
592d9bb9fb0a535a1382193dfd46ebc589c76ef2606ac0eaf0ffa837048861c8  V252/production/rl_real_b2_z1_no_gun.cpp
0cd044e97e26acecf72a801732dd63b97ffdb7f3887cc43e692a16092ef2a286  V252/production/va15_controlled_stop.hpp
214c654a220b59575f6d4f9d0cd28fa39e9b27d2c759625be3080bbed9f0ddcd  V252/production/va15_independent_state.hpp
9a0ff7fcbf422c95ebafd87934a1def0f70e74b16b4b30cac6e329faa4d052e1  V252/production/trial_telemetry.hpp
d04f9ca1ad0bfc2e31a86e558e44c7a15ac73fd4e2b844737d59a032d2ca6c17  V252/CMakeLists.txt
b5fe53515ece144cbd6b4d328f65104c5b7856de23e74f1ba5b2b6a4bcc7073d  V252/qualification.json
8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098  patches/va15_stop_feedback_20261020/work/b2_z1_no_gun/VA15/policy.pt
```

其中 V252 代指 `patches/va15_stop_recovery_v252_20261027`。这些是状态标识，不代表发布。

## 7. 恢复工作后建议第一步

先读本文件及原交接，核验工作树和指纹；不要因为用户此前要求“今天解决”就越过板载/运动限制。当前用户要求暂停，待其恢复任务后再继续实现。

软件方面优先完成上面的实际 SDK 无通信测试，明确复现范围，再设计单一命令所有者和有界 FSM 握手；不要先改物理增益。停止恢复继续区分“B 意图抢占”与“真实支撑后下降”，不能让按钮或超时绕过四脚门控。

真实横摆定因仍缺同一单调时钟下的高频最终命令 q/dq/Kp/Kd/tau、实测 q/dq/电流或力矩、IMU 和独立足端支撑证据，还需核验实际发布者/固件/加载库。原始日志不足时要说明限制，不用失败的一阶模型或名义 MuJoCo 通过替代实机验证。现场试验须用户重新确认状态、授权且具备防倾倒保护；没有支架的情况仍未解决。

视觉近线/长前视解耦尚未实现。维持原 `VISUAL_STOP_REVIEW.md` 的建议和边界，不伪造 valid、远点或时间戳，不允许旧视觉自动重启。

## 8. 可复制给接手帐号的话

> 请先完整阅读 `/home/dong/rl_sar_b2_z1/docs/archive/va15/HANDOFF_VA15_ACCOUNT_SWITCH_20261027.md`，再读它列出的原交接、补充工作记录及最新两份策略/仿真报告。继续 B2＋Z1 VA15 实机视觉部署。V252 仍是本地未发布候选；当前没有已验收的横摆或 FR 停止恢复修复。真实 SDK 的无通信测试只是方案，尚未编写和运行，已有 Z1 单测是假的契约模型。请保留所有代码和日志，不盲调增益，不伪造接触或帧时效；先补软件复现和有界握手设计。没有我重新确认状态和授权，不得同步板载、启动控制或机器人运动。先告诉我你读到的进度和第一步。
