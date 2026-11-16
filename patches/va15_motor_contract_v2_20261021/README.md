# Motor contract V2: fix cross-run absolute lost baseline

## 本次确认的起立阻塞原因

两次新测试 `motor_contract_forward_once_20261021_vUCpWN` 和 `9KbUJS` 全程
处于 Passive，腿部下发 kp 全为 0、没有 actor 推理；A 被软件准入判断拒绝，
不是已经执行起立却撑不住。原始 lost 是
`[0,0,0,6,5,5,5,0,0,0,0,0]`，与上一次不同；当前电机故障码始终全 0、
通信频率 534..541 Hz、bit_flag=4。

旧候选把上一次试验的绝对值写成了跨运行固定表，因此再启动时被卡住。
这个设计不合适；本版只改输入契约的初始化和其诊断记录，不改起立动作。
原目录和二进制完整保留，不在运行中热更新，不自动起立/行走。

## 新判定：每进程采集一次，READY 后不再重置

1. 只有本程序自己的 lowcmd 发包开始后才收集启动数据。
2. 仍要求 bit_flag=4（无命令超时/组件故障、不在充电），全部 mode=10、
   reserve[0]=0，reserve[1]=400..800 Hz，源时刻连续、源年龄不超过 50 ms。
   频率范围仍是实验工程边界，不是厂家认证参数。
3. 本进程启动阶段先记录其当下原始 lost 向量；原值不被改写。
4. 尚未 READY 时，有变化就重新计算 **3 秒连续稳定且健康** 的窗口；
   此时 A/Y 始终禁用，不能在初始化期间进入运动。持续变化不能通过。
5. READY 时锁定这份本进程基线。随后 lost 增长、下降、清零、通道变化或
   任一健康故障仍锁存拒绝到进程退出；A/Y/持续推杆不能重新学习或解除。

仍不宣称 lost=5/6 是厂家确认的历史计数或其复位语义已验证；这里只以已
公开的当前故障/通信频率等字段作为独立实验输入证据。官方来源：
[B2 底层服务接口](https://doc-cdn.unitree.com/9/135/zh/9_135_zh)、
[B2 故障码](https://doc-cdn.unitree.com/9/139/zh/9_139_zh)。

IMU/关节数值与导数检查、支持/静止条件、动作/接管时间、增益、TCP、VA15
权重/70观测18动作、动作历史/EMA、B 并行趴下和有界关闭均不改。
不修改接触阈值、validated 位或伪造 sport_fresh。私有模型估计依然受平地、
无共同打滑等假设约束，hardware_qualified=false，不是实机动态验收。

## 当前操作：仅恢复静态 A/Y/B 流程

仅在实际趴稳、旧程序退出、没有枪/软管、收臂区域净空、有可靠防倾倒和已
验证的现场应急措施时操作。`--standby` 禁止行走；不能调用二进制绕过。

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_motor_contract_v2_20261021/run_trial.sh --offline-check
bash diagnostics/va15_motor_contract_v2_20261021/run_trial.sh eth0 --standby
```

输入大写 RUN 后，先保持趴卧和摇杆回中，等待：

```text
[Motor contract] ready=1 latched=0 bit_flag=4 baseline_frozen=1
```

然后 A 起立，实际站稳后 Y 待机；不推杆、不按 R1+Y。观察约 10 秒，
B 趴下，实际趴稳并显示 Passive 后 Ctrl+C。若有拒绝/锁存，不反复按 A，
保存日志；保持实际趴稳后再退出。P/L1+X 会撤去支撑，不是正常趴下。

输出 `outputs/motor_contract_v2_static_20261021_XXXXXX/`；commands.csv 新增
contract_baseline_dds_0..11、contract_baseline_frozen、contract_stable_seconds、
contract_startup_changes。终端也打印本次 raw/baseline 向量及采集状态，
不再把跨运行旧值当作本次当前状态。

## 验证范围

单元覆盖不同进程使用不同原值、初始化变动重新计时、持续变动禁止 READY、
READY 后变化仍拒绝，以及原电机/时基/超时检查。仿真包括最新原值启动、
启动期间变更和持续变更的负例；结果保存在 `outputs/suite_final/summary.json`。
离线结果不保证实机不会晃动或倾倒；首次验证仅准备静态复测，不自动试走。

## 2026-10-22：静态复测后开放单次无视觉前进实验

现场记录 `outputs/motor_contract_v2_static_20261021_8apG1h` 已复核：
A/Y/B/Passive 状态顺序完整、无 actor/非零运动命令、电机当前故障码全零、
bit_flag=4、通信频率 537..541 Hz、READY 后基线不变、无锁存故障、CSV 无丢失，
录制器结束码为 0。操作者也确认现场正常。只据此开放实验入口，不写入任何
硬件认证或接触标定通过位，不宣称静态结果等于动态验收。

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_motor_contract_v2_20261021/run_trial.sh eth0 --forward-test
```

该入口启动前重新检查冻结源代码/权重/配置和上述静态记录。输入大写 RUN
只表示现场操作者决定启动这个受限实验；启动后仍从当前数据重新建立基线，
任何运行时故障/源时序错误仍拒绝运动，不能用旧日志替代当前支持证据。

操作：保持摇杆回中，等 Motor contract READY，再 A 起立；站稳后 Y 待机。
等 `[Restricted trial] armed` 后仅向前推摇杆。推杆是 **固定 0.15 m/s 目标的
触发器**，不是线性调速；原加速度缓变仍保留。首次观察 1 秒后即松杆，
若不稳则立即松杆或 X 请求停止，不等自动截止。每进程仅一次、前进请求
最多 4 秒（不是物理停稳时间保证）；不支持后退、侧移、转向或视觉 R1+Y。
X/松杆请求停止；B 同时要求支持条件满足后趴下。故障或支持证据无效时，
不能保证站稳捕获/趴下完成；按现场已验证的应急流程处理，不能靠强退/
断电当作安全停止。实际趴稳并显示 Passive 后再 Ctrl+C。不要连续重试。

记录保存在 `outputs/motor_contract_v2_forward_20261022_XXXXXX/`。所有控制
二进制、动作历史/EMA、增益、前进限制/停止逻辑和足端阈值均未改变。
本次只解除已完成静态评审的外层入口封闭，并保留数据检查和人工确认。

## 2026-10-22：Ctrl+C 后终端录制器不退出

本次主控制的五个循环、Z1 PASSIVE 应答和 CSV 关闭都已完成；残留的是
外层 `script` 录制器，而非仍在执行的主控制进程。旧启动器用 `exec script`
替换自身，使它之前启动的后台 `z1_ctrl` 也变成录制器的子进程。在现场
util-linux 2.37.2 中，录制器回收主命令后会因这个仍存活的无关子进程
卡在 `waitpid(-1, ..., WNOHANG)` 循环。

修正为保留 Bash 启动器，让 `script` 作为前台子进程运行，并原样传递退出码。
Z1 服务保持独立会话，不在启动器退出时被停止。本次只改录制器父子关系，
不改控制二进制、VA15 权重、增益、A/Y/B、接触判断或行走禁用条件。

离线 PTY 测试只使用 `sleep` 和打印命令，不连接机器人：

```bash
/usr/bin/python3 -m unittest discover \
  -s diagnostics/va15_motor_contract_v2_20261021 \
  -p 'test_recorder_exit.py' -v
```

覆盖旧版挂起复现、修正版正常退出、非零退出码保留、Ctrl+C 传递，以及
后台模拟服务不被停止。测试需要 Linux pidfd 和系统 Python 3.10+。
源码依据：[util-linux v2.37.2 pty-session.c](https://github.com/util-linux/util-linux/blob/v2.37.2/lib/pty-session.c)。

现场遗留录制器只在操作者确认 B2 趴稳、Z1 安全放置后定点清理；Z1 服务
未被结束或重启。该次 `terminal.log` 缺少录制器结束行，命令退出码未知，
不能据此记为完整通过或动态验收。退出修复后先复测 A/Y/B 静态流程：实际
趴稳、显示 Passive 后再 Ctrl+C，确认直接回到提示符，再评审日志。本日
后续静态复测已通过，受限行走入口和限制见上一节，不等于动态验收。
