# 单次受限前进：静态复核之后的独立入口

## 已核实，和仍未核实的部分

`outputs/motor_contract_static_20261021_VcRI5C` 的静态 A/Y/B/Passive 顺序完整，
退出正常，未执行策略推理；bit_flag=4，电机故障码全部 0、当前通信频率
534..540 Hz，没有契约锁存或记录丢失。用户随后确认实际趴稳。

新增入口只复用**同一份** `native_build_v2/va15_forward_trial`；未重新编译、
未修改原生控制源码、策略、配置、接管时间、增益或停止/支持门槛。
`check_static_evidence.py` 固定上述五个记录文件和编译 bundle 的 SHA-256，
并重新审查静态记录。缺失、变动或不匹配均拒绝。旧入口继续保留为静态版。

静态结果不是当前站姿证据，不是整机安全认证，也不授予动态停止/卸力权限。
每次仍需现场操作者确认实际条件、输入 RUN，且本机新发包后重新通过 3 秒
实时电机健康审查，站稳后再通过既有支持/静止检查。不把旧记录伪装成实时反馈。

## 板载单终端操作

先确保实际趴稳、旧程序退出；无焊缝枪和软管；前进/收臂区域净空；有有效
防倾倒保护和经过验证的应急方式，人员离开倾倒/摆臂区域。缺少条件不试走。

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_motor_contract_20261021/run_forward_once.sh --offline-check
bash diagnostics/va15_motor_contract_20261021/run_forward_once.sh eth0 --forward-test
```

1. 阅读提示，满足条件才输入大写 RUN；Z1 服务自动复用或启动。
2. 保持趴卧和摇杆回中，等 `[Motor contract] ready=1 latched=0 bit_flag=4`。
   出现 timeout、latched 或其他拒绝时不要起立，更不要反复推杆。
3. A 起立，实际站稳后 Y 待机；保持回中，等待 `[Restricted trial] armed`。
4. **不按 R1+Y/T，不开启视觉。** 沿前向推杆并保持。前推是按住触发，
   接受后固定请求 0.15 m/s，经原加速度斜坡增加，不按推杆幅度提高速度。
5. 第一次只看起步；有明显横晃、拖地失衡或机械臂异常，立即松杆或 X 请求停止，
   不等到限时。若始终稳定，单次正向请求最多 4 秒后自动请求停止。
6. 等实际停止后 B 请求趴下。实际趴稳 + Passive 后 Ctrl+C；不要站着退出。
7. 每进程只有一次正向请求；松杆、X/B、截止或异常后锁存停止，不靠持续
   推杆/重按 Y 再开一轮。完成后先检查本轮记录，不直接重复试走。

4 秒是正向请求预算，不是物理停车时间/距离保证。坏反馈可能阻止捕获站姿或
下降；不要用强制退出、断电或直接卸力替代停止。P/L1+X 是紧急被动，可能
撤去支撑，只按已经验证的现场应急流程使用。无本轮动态验收，不能保证不摔。

## 记录

`outputs/motor_contract_forward_once_20261021_XXXXXX/` 保存 terminal.log、
commands.csv、control.csv、actor.csv 和 telemetry_summary.json。保留所有原始
lost、故障码/通信频率、真实下发目标/增益和观测/动作供下一步比较。

原 `analyze_telemetry.py` 的 `mode_or_lost_bad_rows` 沿用严格旧检查，
不能将此计数直接解释为新契约的当前电机故障；查看原始 motor_fault_dds_*、
motor_frequency_dds_*、contract_ready/latched 和 bit_flag。任何摘要都不能
代替真实位移/抬脚/滑移观察或物理安全确认。
