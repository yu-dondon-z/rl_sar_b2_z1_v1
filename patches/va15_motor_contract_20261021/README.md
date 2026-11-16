# VA15 current-motor-health candidate (2026-10-21)

独立目录，不覆盖原生产程序或 0.15 m/s / 4 s 候选。初版仅允许
离线及人工 A/Y/B 静态检查；原 `run_trial.sh --forward-test` 继续拒绝，禁止直调
二进制绕过入口。静态流程完成且操作者确认实际趴稳后，新增的独立受限入口
见 [FORWARD_ONCE.md](FORWARD_ONCE.md)。没有自动启动机器人或机械臂运动。

## 新依据，不再只查看无说明的 IDL

通过官方文档站公开目录接口取得 B2 正文：

- [B2 底层服务接口](https://doc-cdn.unitree.com/9/135/zh/9_135_zh)：
  lost 为电机丢包信息；reserve[0] 电机错误标志位；reserve[1] 当前通信频率。
  bit_flag 0x01 为命令超时，0x04 为“不充电”状态，并非故障。
- [B2 故障码](https://doc-cdn.unitree.com/9/139/zh/9_139_zh)：
  说明电机过流、编码器异常、通信中断等故障位和来源。
- 公开目录来源：`https://robot-api.unitree.com/doc?space=B2_developer&locale=zh`。

官方仍没有说明本固件 lost 的累计/复位规则；**不宣称 lost=5 已被证明是
历史计数**，也不将该值清零、改写或在所有版本中忽略。

本轮只读 14.985 s / 1473 帧：DDS lost 固定
`[0,0,0,0,0,5,5,0,0,0,0,0]`；全部 reserve[0]=0；reserve[1]=533..540。
原始记录在兄弟目录 `va15_lost_audit_v2_20261021/outputs/lost_contract_20261021_WAhXxN/`。
无 tick 倒退/重复/>50 ms 间隔，末帧年龄约 1.22 ms。对应 q/dq/tau 有变化，
只排除整组常量，不作逐电机时效认证或动态负载验收。
操作者确认当前趴稳退出，App 报运控命令超时。原始 bit_flag=5 包含此位；
尚未证明它在程序持续发布时会清除，因此必须先进行静态接管复核。

## 适当放宽的具体边界

仅私有候选允许上述**精确原值向量**，不自动学习未知非零值。改为检查：

1. bit_flag 必须仅为 0x04：无命令超时、急停、组件故障且不在充电。
2. 所有电机 mode=10、reserve[0]=0、reserve[1] 在 400..800 Hz。
   频率界限是依据本次观测选的实验工程余量，**不是厂家认证阈值**。
3. lost 与本次观测逐通道一致，任何增长、下降或位置变化均拒绝；不重置基线。
4. 接收与源时基连续；重复 tick 不刷新时效；50 ms 内的新源数据。
5. 连续 3 秒通过才具备此项输入资格。A/Y 此前拒绝且必须重新按键。
6. 一旦已具资格，后续异常锁存至进程退出；不能持续推杆/按 Y 自动恢复。
7. 原 IMU、关节数值/速度/力矩、导数、支持和停稳检查仍生效；不伪造
   sport_fresh、contact、接触标定/validated 位。严格估计器默认行为仍拒绝非零 lost；
   只有显式启用当前健康契约的私有实例走替代路径。

权重、70/18 观测/动作、关节映射、动作历史/EMA、增益、TCP、起立/并行
趴下轨迹和 X/B/P 优先级未改。未来实验运动边界仍至多 0.15 m/s、单次
4 秒请求，但本版入口先禁走。条件估计依赖平地/无共同打滑等假设；
`hardware_qualified=false`，不代表实机运动合格或能在故障后自动安全卸力。

## 当前操作：仅板载一个人工终端

前提：实际趴稳、旧程序退出；无枪/软管、收臂区净空、有效防倾倒及已验证
应急措施。未满足前提仅做 offline-check。禁止多个控制程序并行。

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_motor_contract_20261021/run_trial.sh --offline-check
bash diagnostics/va15_motor_contract_20261021/run_trial.sh eth0 --standby
```

大写 RUN 后先保持趴卧/摇杆回中，等 `[Motor contract] ready=1 latched=0
bit_flag=4`。始终有 timeout 或其他拒绝项时不要起立，保持实际趴卧后退出
并交回日志；不清故障、不抬增益。如果 READY：A 起立，站稳后 Y 待机，
不推杆、不按 R1+Y；观察约 10 秒，B 趴下，实际趴稳 + Passive 后 Ctrl+C。
新版本保留 Z1 单终端自动启动和有界退出。

输出 `outputs/motor_contract_static_20261021_XXXXXX/` 包含终端和三路 CSV。
commands.csv 新增 bit_flag、contract_ready/latched/reason/bad_joint，以及每个
电机原始故障码和当前通信频率；不再只打印“center stick”掩盖真实拒绝原因。
先审查完整静态流程，再决定后续运动入口；不要反复推杆等待自动放行。

## 本轮验证与同步结果

- 本地 CTest 3/3、Python unittest 3/3 通过。包含默认严格估计器仍拒绝
  非零 lost、候选没有当前健康证据时仍拒绝、72 组逐电机错误注入等。
- `outputs/suite_final/summary.json`：31/31 预期行为场景通过，含正常迈步后
  放杆/X/B、命令超时、电机故障码、通信频率、lost 增长/下降、失效/过期源。
  故障场景不授予虚假停稳/下降资格；这**不是**故障后自动安全趴下承诺。
  正常基准模型前移约 0.384 m；全场景最大倾角约 2.48 度，均无模拟倾倒。
- 板载 `native_build_v2/` 编译及原生 golden 通过，最大差异 1.14441e-05；
  `--offline-check` 通过，没有 DDS/UDP/机器人初始化。
- 本机发包线程成功开始后才启动原始状态的 3 秒审查；不能用接管前的官方
  流量预先取得 READY。DDS Write 返回本身也不是电机接收确认。
- 板载原主程序/FSM/config/actor SHA-256 均与此前保持一致。
- 当前只有只读采样和离线执行，没有人工静态新契约验收，也没有实机行走验收。

静态测试实际趴稳退出后，可只读审查刚打印的输出目录：

```bash
python3 diagnostics/va15_motor_contract_20261021/review_static.py \
  outputs/motor_contract_static_20261021_实际目录后缀
```

审查器要求新契约 READY、实际 FSM A/Y/B/Passive 顺序、零运动输入、无策略
输出、无锁存/电机故障、正常退出和完整记录。即使全部通过也只给出
`software_static_review_pass`，固定 `motion_authorized=false`，不擅自确认
物理趴稳、失去力矩安全或放行下一次运动。
