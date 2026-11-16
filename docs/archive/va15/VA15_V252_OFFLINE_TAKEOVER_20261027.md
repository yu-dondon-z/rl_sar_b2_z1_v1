# VA15 / V252 接手离线补充记录

日期：2026-10-27。本记录补充 `HANDOFF_VA15_REAL_V252_20261027.md`，不替换原交接、诊断、验证报告或资格文件。

2026-10-27 15:55 用户要求暂停并切换帐号，最新交接入口为 [HANDOFF_VA15_ACCOUNT_SWITCH_20261027.md](HANDOFF_VA15_ACCOUNT_SWITCH_20261027.md)。其中明确区分了已完成审计与尚未实现的真实 SDK 无通信测试；本记录中的历史结论不代表发布或实机修复。

## 当前结论

V252 仍是本地未发布候选，未同步板载，未进行实机验证。高频抖动、真实执行器闭环响应和物理停止恢复尚未解决。以下单元测试通过不等于实机故障修复，也不授予行走、盲测、同步或运动权限。

本轮仅操作本机离线文件、构建及无通信测试；没有 SSH、板载同步、DDS/UDP 初始化、相机/Z1 控制或机器人运动。用户仍需重新确认状态并明确授权，才能进行任何板载操作。

## 已读与复核

- 完整交接文件及其列出的 V251 `DIAGNOSIS.md`、V252 `VALIDATION.md`、`qualification.json`、`BOARD_SYNC_REPORT.md`。
- `outputs/startup_analysis/` 中的 `REPORT.md`、`TRAINING_CONTRACT_AUDIT.md`、`Z1_GAIN_ORDER_AUDIT.md`、`PLANTED_HIP_REVIEW.md`、`VISUAL_STOP_REVIEW.md`。
- 停止恢复实现、原生离线入口、反馈来源及关节映射、Z1 启停增益顺序、相关测试与物理复现准备过程。
- 原工作树含 `.vscode/settings.json` 修改以及未跟踪的 `docs/`、`outputs/`、`patches/`，均保留；磁盘仅余约 6.6 GB，未清理旧目录、记录或依赖。

SDK 位于 `/home/dong/rl_sar_b2_z1/src/rl_sar/library/thirdparty/z1_sdk`，本地库与旧目录及原审计哈希一致。初次相对路径检查失误不代表 SDK 缺失；本轮原生构建已成功。板载实际加载的库仍未核验。

## 本轮变更

变更仅位于 `patches/va15_stop_recovery_v252_20261027/`，没有改 VA15 权重、观测、动作历史、增益数值、接触判据、视觉时间戳或最终趴下许可门槛。

1. `test_planted_hip_recovery.cpp` 补充实际候选使用的实验反馈来源分支：`sport_fresh=false`，来源标签明确。注入 `1011` 持续 16000 个 5 ms 周期，断言停在 Settle、诊断 contacts=13/mask=4、不得 Ready；恢复四脚后仍需连续 0.5 秒，期间再次丢接触会清零计时。这些是明确注入的测试输入，不是伪装的真实接触。
2. 同文件加入历史髋角和近似静态请求力矩的数值反事实：检查四髋恢复参考约为 `[0.112202,-0.083762,-0.018652,-0.008256]` rad，以及捕获时 PD+前馈请求连续。静态力矩来自降采样记录的近似值，不是未被记录的私有 `stand_tau`，不构成动力学回放。
3. 新增 `test_z1_gain_order.cpp` 并接入 CTest。假的 SDK 缓冲模型表达已审计的 `PASSIVE -> LOWCMD` 覆盖效应，覆盖三轮启停、仅 Kp 缩放和 Kd 不变。它不链接真实 SDK，不能监测真实 SDK 的未来变化或证明电机行为。
4. `production/rl_real_b2_z1_no_gun.cpp` 仅增加 LOWCMD 转换后的缓冲 Kp/Kd 打印，并把原日志 `kp_scale` 改为 `requested_kp_scale`。没有移动或新增增益写入。打印明确标识单位未验证、不是电机确认；尚未采集实机输出。新增打印有启动时序开销，不能当作已验收的运行版本。

`production/va15_controlled_stop.hpp`、`va15_independent_state.hpp`、`trial_telemetry.hpp` 未改。没有通过按钮、超时、模型接触替代物理支撑证据，也没有伪造帧有效性或刷新旧路径时效。

## 本轮验证与证据

路径均相对于本地 V252 候选目录。

| 检查 | 结果 | 新证据 |
| --- | --- | --- |
| C++/原生无通信测试 | 25/25 | `outputs/ctest_takeover_20261027_110448.log` |
| Python 回归 | 62/62 | `outputs/python_takeover_20261027_1115.log` |
| 原生候选编译 | 成功 | `build/native_candidate_offline` |
| 原生 `--offline-check` | 通过，golden 误差 0 | 上述 CTest 的 `native_candidate_no_robot` |
| 静态四脚站立停止链路 | 4/4 PASS，限于 MuJoCo 控制链路检查 | `patches/va15_stop_recovery_v252_20261027/outputs/supported_stand_20261027_1330/` |
| 原始 FR=1011 故障复现/验收 | 未完成 | 原 `recorded_*` 失败/不合格证据保留 |
| 实机运行 | 未进行 | 无实机验收结论 |

原 `ctest_v252.log`、`python_v252.log`、所有物理套件、V251 日志及原报告均保留。旧物理套件没有针对本轮新源码重跑，其历史通过结果不得改标为本轮接受证据。

`qualification.json` 原样保留；其中 24/24 引用的是原记录，不是本轮 25/25。`simulation_pass`、`walk_release_allowed`、`blind_test_release_allowed`、`board_sync`、`hardware_qualified`、`live_robot_tested`、`real_failure_reproduced_in_physics`、`startup_oscillation_resolved` 仍全部为 false。

## 静态站立停止链路补测

新增 `supported_stand` 场景从 MuJoCo 动力学稳定站立参考开始，随后保持原站姿 2 秒，不挪足、不改髋目标，再进入原停止路径。它不注入接触，不改传感器或观察器，也没有使用真实硬件。候选和 V251 基线各用真值反馈、独立模型反馈跑一次，4/4 均成功趴下：

| 控制器/反馈 | 起点 | 捕获 | 开始下降 | 趴下 | 最大倾角 | 下降期间物理支撑缺失 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| V252 / truth | 真实与模型四脚，倾角 0.0031 rad | 0.005 s | 2.505 s | 6.775 s | 见 CSV | 否 |
| V252 / estimated | 同上 | 0.005 s | 2.505 s | 6.775 s | 见 CSV | 否 |
| V251 / truth | 同上 | 0.005 s | 2.505 s | 6.775 s | 见 CSV | 否 |
| V251 / estimated | 同上 | 0.005 s | 2.505 s | 6.775 s | 见 CSV | 否 |

四组数值结果相同；这只说明普通四脚稳定起点下，两版都能走通试验停止链路，不显示 V252 相对基线有改善。报告中的 `hardware_qualified` 仍为 false。

随后以 `supported_wide_large` 做了同条件四组对照：缓慢给前髋施加 `+0.14/-0.14 rad` 静态目标偏移，不重写状态、不移动脚、不覆盖观察器输出。真实 MuJoCo 接触四脚保持 40 秒，最大倾角 `0.0079 rad`。在日志 `estimated_contacts_DDS_FR_FL_RR_RL` 顺序中，掩码 14 表示 FR 未被模型判为承重；它与实机日志 `contacts_FL_FR_RL_RR=1011` 的语义相同（FR 缺位），但比特顺序不同。

| 控制器/反馈 | 捕获/下降/趴下 | 物理四脚而模型部分支撑 | 结果 |
| --- | --- | ---: | --- |
| V252 / truth | 0.005 / 2.505 / 6.775 s | 0.08 s | PASS；真值反馈反事实 |
| V251 / truth | 0.005 / 2.505 / 6.775 s | 0.08 s | PASS；真值反馈反事实 |
| V252 / estimated | 40 s 内均未发生 | 9.59 s | FAIL；无支撑捕获，不强制下降 |
| V251 / estimated | 40 s 内均未发生 | 9.59 s | FAIL；无支撑捕获，不强制下降 |

这比之前的宽站姿结果更接近“接触估计阻止停止恢复”：同一个 MuJoCo 初始状态下，估计反馈拒绝捕获，而物理真值反馈会捕获。它验证了门控行为，不证明实机 FR 当时真实承重，也没有证明此固定髋目标偏移是自然步态中形成的。候选和基线在此试验中没有差异，不能用真值反馈的 PASS 作为可部署结论。

## 尚未解决及下一步

V251 原始停止门控观察已完成，详见下节。它确认已记录的 Settle 决策条件，但不是控制器时间连续重放，也不能复现停止控制对之后状态的动力学影响。若之后要建立物理 FR 接触结论，仍需独立足底力/接触观测。当前模拟中的 `+/-0.14 rad` 固定髋目标只是一项稳定反例，不应扩大成增益或阈值调参。

同时保留独立的执行器调查：历史 CSV 约 47.2 Hz，不是完整 200 Hz 写出流；5.91 Hz 响应明显但 12 个关节一阶拟合均不合格，不能据此调参。真实加载库、增益单位、命令发布者是否竞争、完整命令与测量时序尚缺证据。SDK 缓冲快照也不能替代电机或固件确认。

Z1 顺序问题只完成源码审计、契约模型和诊断准备，尚未修正真实写入顺序。把缩放增益移到 LOWCMD 之后会真正改变执行器命令，必须单独建立候选和验收，不能作为无风险日志修复顺带部署。

视觉问题仍按原审计分离近线与远路径；不得通过虚假远点、接触或时间戳消除停车。停止链路和真实响应未确认前，不进入反复实机行走试验。

## 本轮文件指纹

以下 SHA-256 用于区分本轮离线源码/构建与旧物理报告，不代表发布或硬件资格。

```text
592d9bb9fb0a535a1382193dfd46ebc589c76ef2606ac0eaf0ffa837048861c8  production/rl_real_b2_z1_no_gun.cpp
0cd044e97e26acecf72a801732dd63b97ffdb7f3887cc43e692a16092ef2a286  production/va15_controlled_stop.hpp
214c654a220b59575f6d4f9d0cd28fa39e9b27d2c759625be3080bbed9f0ddcd  production/va15_independent_state.hpp
9a0ff7fcbf422c95ebafd87934a1def0f70e74b16b4b30cac6e329faa4d052e1  production/trial_telemetry.hpp
bc6ba7b95471b5de5f8b2c280e40e18491bffa37e020b514f4811cc9b7925b53  test_z1_gain_order.cpp
3c69d174f0b4a452a624b5b4a04c583f100b87f415ab8ea8b3228242ed0c1f7d  test_planted_hip_recovery.cpp
d04f9ca1ad0bfc2e31a86e558e44c7a15ac73fd4e2b844737d59a032d2ca6c17  CMakeLists.txt
0b73cc22aed9d1c0cbdb39e1bd1dc9803e3e8fdf4f9c99dd08a4bb5cca1c7701  build/native_candidate_offline
b5fe53515ece144cbd6b4d328f65104c5b7856de23e74f1ba5b2b6a4bcc7073d  qualification.json
```

本轮修改前主程序 SHA-256 为 `aeb5f70cfa952854e43ce1f4ec3c10820f7121647b3859b2c77ad6cedb9b33d1`；原生二进制为 `ebf6f7101b0771211bf02c3aa85d0f73e0c46c1fac7271a46ee8c27d19ceb13d`。

## V251 原始停止门控观察

对 `outputs/diagnosis_v251_yumjt3ry_20261027/control.csv` 和 `commands.csv` 做了只读逐行核对，没有把稀疏样本补齐、插值或重新积分控制状态。

- `control.csv` 有 5127 条采样记录，其中 `stop_stage=3`（Settle）为 3430 条，覆盖约 80.547 秒。首末控制序号为 6786 和 20501；采样间隔中位数 23.500 ms，范围 20.079–24.777 ms（约 42.6 Hz）。`commands.csv` 对应 3797 条 Settle 发布采样，间隔中位数 21.199 ms（约 47.2 Hz）。这些都不是 200 Hz / 500 Hz 的完整状态或发布流。
- 每一条 Settle 观测的 `stop_gate_mask` 都是十进制 `4`，接触位图都是十进制 `13`，且 `stop_contacts_valid=1`、`stop_source_valid=1`。按 `stop_gate_diagnostics.hpp` 的枚举，mask `4` 仅表示 `PartialSupport`；位图按 `FL, FR, RL, RR` 排列是 `1011`，缺的是模型判定的 FR。Source、contacts unknown、height、speed、tilt、gyro、joint speed、joint drift、stale 等失败位均未置位。
- 同一批样本中，倾斜量为 `0.01798–0.01952`，角速度范数 `0.00452–0.01339 rad/s`，最大腿关节速度 `0.000231–0.001946 rad/s`，关节漂移 `0–0.0000768 rad`；`stop_quiet_s` 始终为 0。于是这是持续缺一脚估计导致的门控等待，不是姿态、运动速度或数据源有效性导致的等待。
- 对照 V251 与 V252 的 `VA15ControlledStop` 及反馈谓词，V252 的改动影响捕获时冻结的髋参考与捕获校验；最终 `Settle` 仍调用相同的四脚 `Settled` 谓词。把这段已记录观测条件原样喂给门槛只能得到两版同样不 Ready 的结果。静态样本不包含 V252 控制命令施加后可能改变脚力/估计的动态响应，不能用作完整 log-conditioned 控制回放，也不能评价这种响应是否会发生。

因此可确认：V251 卡住的直接软件门槛是 FR 模型承重位缺失；V252 在现有日志证据上没有改变最终放行条件。仍不能确认实机 FR 是否真实承重，也不能据此断言髋参考修改能否让实机估计恢复。此前 MuJoCo 的估计反馈 `supported_wide_large` 对照中 V251 和 V252 都未捕获，结论一致；四脚真值反馈成功属于不可部署的反事实。

## 2026-10-27 追加：策略重放与 Z1 发送竞态

为继续分离 5.91 Hz 左右摆动来源，用本地已确认的 V251 `policy.pt` 对原始 `actor.csv` 的 545 组 `model_executed=1` 记录逐帧重放。输入是日志里的 70 维实际 observation，输出是 18 维 raw action。策略 SHA-256 为 `8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098`；重放对日志 raw action 的最大绝对误差 `2.143e-6`、P99 `9.568e-7`、RMSE `3.063e-7`。故日志中的动作与该策略对所记录观测的确定性输出一致，排除了本地策略权重错配、观测列读取错位或 raw-action 记录串台作为这次 5.91 Hz 的解释。

结合 V251 诊断中 raw action、策略目标和最终发布目标均有约 5.9 Hz 成分，这使“仅 DDS/打包层凭空产生摆动”不符合证据。但 observation 本身含真实状态反馈，且实测关节明显跟不上目标；现有稀疏日志无法把策略-机器人闭环、真实执行器响应不足、负载/地面等因素拆开，也不能据此认定应滤波或更换策略。

复核 Z1 增益覆盖问题时发现，不应只把 `.35` Kp 写入移动到 `setFsm(LOWCMD)` 后面。当前 SDK 500 Hz `sendRecvThread` 与公开的 `lowcmd` 向量没有互斥；`setFsm(LOWCMD)` 自身会重置默认 Kp/Kd 并等待接收状态，发送线程在此期间仍运行。应用层在转换后再写增益，也可能与发送线程读取同一向量竞争；SDK 内部重置本身亦存在并发读取窗口。仓库只有该 SDK 的头文件和二进制，没有对应实现源码可加入共享同步。顺序调整不是已证明安全的修复；当前候选主程序和契约测试文件都已核对恢复为原 SHA-256，未留下生产增益写入改动。

本次离线验证：正确筛选名 `z1_lowcmd_gain_order_contract` 的单测通过；V252 原生 CTest 全套 `25/25` 通过。它们只验证现有软件合同/候选离线行为，不代表实机摆动、FR 支撑或停止恢复已修好。下一步若要改 Z1 命令所有权/增益，必须先取得与板载一致的 SDK 源码或官方同步 API；若要归因和验收机械响应，还需要在具备防倾倒支撑并经用户重新确认/授权后采集同一单调时钟下的高频最终命令、实际 q/dq、电流/扭矩和 IMU。当前用户无法现场测试，所以本记录不生成发布候选、不同步板载、不启动控制或运动。

## 2026-10-27 追加：名义 MuJoCo 闭环对照与 SDK 暂停线程审计

运行现有 `simulate_trial`，加载同一 V251 VA15 actor，在 MuJoCo 名义场景执行约 4.005 秒、请求速度 `0.15 m/s` 的无视觉闭环走行，随后走既有停止/趴下流程。CSV 保存在 `outputs/va15_actor_mujoco_nominal_20261027/steady_015.csv`。仿真未跌倒，最大倾角幅值 `0.02568 rad`，行走最低机身高度 `0.517 m`，请求结束时前进约 `0.481 m`、横向偏移约 `-0.103 m`；最终报告 `hardware_qualified=false`。

该结果只说明此名义仿真没有跌倒，不能证明实机左右倾斜或 5.91 Hz 摆动消失：本场景不是 V251 启动日志的物理闭环回放，CSV 仅输出倾角幅值而非有符号横滚角/陀螺频谱，且没有实机执行器状态和足底载荷。现有启动日志也缺完整机身平动初态、接触力与执行器内部状态；`simulate_recorded_stop` 会清零 MuJoCo 速度且不运行 actor。因此当前工具不能忠实重建该实机故障，名义走行通过不能用作故障修复验收。

补查发现 SDK `LoopFunc` 头文件公开 `shutdown()`/`start()`，但这并不足以安全地暂停并修改 LOWCMD 增益。随包 x86_64 SDK 二进制中，`unitreeArm::setFsm(LOWCMD)` 先调用默认 `LowlevelCmd::setControlGain()` 和 gripper gain，然后等待 `recvState` 的 FSM 字段匹配目标状态；公开 `sendRecvThread` 负责 500 Hz send/receive。停掉该线程后调用 FSM 切换，状态握手缺少收发推进；保持线程运行则 SDK 的默认增益写入与命令序列化并发。直接在 `setFsm()` 返回后写缩放值同样没有共享锁。故当前二进制 SDK 上不能仅凭 `shutdown()/start()` 或调整调用顺序安全修复覆盖问题。

可继续的软件路径是取得与板载一致且可审计的 SDK 源码/官方同步接口，建立单一命令所有者，并让 FSM 切换、增益初始化和完整帧发送在同一同步边界内完成；否则需要先由供应方确认 SDK 的线程/握手合同。另需保留 V251 高速横滚和足部问题的现场验收：必须具备防倾倒措施，并采集时间对齐的最终发送命令、实际关节状态/电流或力矩、IMU 与独立足端支撑真值。当前没有现场测试条件，也没有收到板载/运动授权；本追加只记录离线结果，不改生产控制逻辑、不同步板载、不启动机器人。

## 2026-10-27 追加：同频策略/仿真对照与 Z1 再启用缺陷

为避免只看最大倾角，给离线 `simulate_trial` 增加了有符号横滚、陀螺仪、左右大腿/小腿目标及实测关节角列；这是诊断输出变更，不改控制器。诊断版构建在 `/tmp/va15-spectral-build.XpIfNw`，重跑输出为 `outputs/va15_actor_mujoco_nominal_20261027/steady_015_spectral.csv`，摘要和拟合方法见 `outputs/va15_actor_mujoco_nominal_20261027/SPECTRAL_REVIEW.md`。

按原 V251 日志时间戳在 5.875 Hz 做含常量和线性趋势的正弦/余弦最小二乘拟合，未插值。实机左右大腿目标的幅值约 0.0618/0.0602 rad、相位差约 160 度；左右小腿约 0.0869/0.0421 rad、相位差约 179 度。横滚该频点幅值约 0.001078 rad，gyro-x 幅值约 0.04627 rad/s，相位领先约 97.5 度。actor 目标与按 DDS 映射、约 14 ms 对齐的已发送目标相关系数为 0.980–0.997；目标被另一个命令源替换不符合现有记录。实际关节响应频点幅值约为发送目标的 0.0016–0.108 倍，且相位不一致，仍不能仅凭稀疏日志确定是执行器、时序/测量还是策略-动力学闭环。

同一 checkpoint 的名义 MuJoCo 走行窗口为 6–10 秒、201 个 50 Hz 样本。5.875 Hz 下横滚幅值 0.000301 rad，左右大腿目标仅 0.003368/0.003560 rad，约比实机目标小 17–18 倍；仿真最强的 4–8 Hz 拟合随信号落在约 4.25–7.3 Hz，未复现与实机同强度的左右交替。MuJoCo 场景未跌倒并完成停止/趴下，但这不是 V251 故障复现。结论是现有物理模型不足以验证动作滤波或增益补丁，当前证据不支持提交这类调参。

SDK 范围修正：官方 [lowcmd_development 示例](https://github.com/unitreerobotics/z1_sdk/blob/master/examples/lowcmd_development.cpp) 展示了切换 LOWCMD 后进行低层控制；SDK 头文件也建议在 LOWCMD 使用独立控制线程并调用 `sendRecv()`。这提供了单一发送所有者的设计参考，但示例没有证明 `setFsm(LOWCMD)` 内部默认增益重置与后台收发并发安全，也不是与本仓库 `.so` 明确版本匹配的实现源码。停发线程后再调用阻塞 `setFsm()` 仍会失去状态握手收发推进；不能用简单顺序修改宣称解决竞态。

另发现独立的 Z1 再启用缺陷：首次进入 LOWCMD 后，`SetCommand` 的停用分支和 `SendPassiveCommands` 仅调用 `lowcmd->setPassive()`，没有请求 FSM 回到 PASSIVE；而 SDK 二进制明确包含错误信息 `Only state_passive could tranfer to state_lowcmd`，`setFsm(LOWCMD)` 对当前 FSM 非 PASSIVE 返回失败。因此同一进程内停用后再次启用可能在 LOWCMD 请求处失败。此问题尚未在真 SDK 测试台或机械臂上复现，不能据此判断其与 B2 横摆有关。修正它需要显式设计并离线验证 LOWCMD→PASSIVE→LOWCMD 的超时/命令连续性，再经用户确认后单独做实机验证；本轮没有改生产控制逻辑。

截至此追加仍无已验证的实机横摆修复。当前改动只有离线模拟器诊断列与报告；无实机连接、板载同步、控制启动或运动。

## 2026-10-27 追加：V251 actor 单步频率分量反事实

用 V251 保存的 70 维观测逐帧重跑 VA15 actor，与日志 raw action 最大误差 `2.14e-6`、RMSE `3.06e-7`，确认本地 checkpoint 与记录一致。再分别从速度命令、TCP 目标、基座角速度、重力投影、关节位置/速度及上一帧 raw action 中去除 5.875 Hz 拟合分量，观察同一策略单步输出变化。TCP 目标和速度命令中没有可见的该频率分量。再把角速度拆轴，单独去除 `gyro_z` 频率分量后，左右大腿动作对比幅值由 `1.680` 降为 `0.878`，小腿对比由 `0.580` 降为 `0.133`；`gyro_x`、`gyro_y` 效应不同。去除上一帧动作分量后左右大腿幅值反而上升。该非线性、单步且不推进物理状态的反事实不能区分策略与机体闭环因果；这仅指出策略对记录中的 `gyro_z` 输入敏感，不表示该物理轴发起摆动，也不支持直接添加角速度滤波。

脚本与逐项数据见 `patches/va15_stop_recovery_v252_20261027/analyze_actor_sensitivity.py` 和 `outputs/va15_actor_mujoco_nominal_20261027/ACTOR_SENSITIVITY_REVIEW.md`。当前不修改控制器、增益、传感器/动作滤波、接触门槛或策略权重。V251 的横摆故障仍未被证明修复；下一步定因仍需要具备防倾倒保护后的时间同步高频执行器响应、IMU 和独立足端支撑真值。未同步板载、未启动控制、未使机器人运动。
