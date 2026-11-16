# V2.4.3 实机无视觉会话核对（2026-10-25）

会话：板载 outputs/va15_blind_v243_e5tN0O，已只读复制至本地 outputs/review_blind_20261025/va15_blind_v243_e5tN0O。用户报告行走基本正常，并确认 X 停止、B 趴下正常，B2 已实际趴稳、程序已退出，仍无焊缝枪和软管。

## 源与完整性

- 测试 bundle：c0e8c2850e301c5e0249d27ed6f06fd8602bd5d04c2847b0eb79fbce5b929728，与此前板载 V2.4.3 离线验证一致。
- session.json：450aacecbc616f785853325f83614eab95cdce98680fda295bf98ede4fe3cea5。
- control.csv：ff6df658422a51ed93240b925e5cfd62568ad2cfa0be02dc6b76f305977c9e69。
- terminal.log：bb139eb5fe6946880524c1670ad56a7f8692ceeae1c5c0486d4f4ac136f02847。
- control 4157 行、command 4417 行、actor 705 行；三类 dropped=0，io_failed=false。control 是降采样诊断，不是控制线程频率。

## 观测到的状态序列

相对首条 control_start_ns；诊断记录为离散采样，时间差非外部运动捕捉真值。

| 时间（秒） | 状态 |
|---:|---|
| 0.000 | Passive |
| 6.774 | GetUp |
| 26.397 | Y 策略待机，尚未推理 |
| 29.175 | Armed |
| 51.737 | R1+Y，固定前进命令，推理开始 |
| 65.322 | 最后一条非零前进命令采样 |
| 65.345 | X，Decelerate |
| 66.366 | CatchStand，推理停止 |
| 68.606 | Settle |
| 69.155 | ReadyToLower，quiet=0.505 秒，contact bits=15，fail mask=0 |
| 71.912 | B 后 GetDown |
| 75.323 | Passive |

终端记录了 X、B 各一次、Parallel arm stow and B2 lowering completed、GetDown → Passive、Z1 PASSIVE reply=1、Recorder complete=1、COMMAND_EXIT_CODE=0。Settle 约 0.55 秒，全部采样 fail_mask=0；没有用超时或忽略接触强行下蹲。

非零前进命令样本跨度约 13.585 秒，命令 x=0.15 m/s，y=yaw=0。该数值不是测得的实际行走速度/距离。活动时独立模型高度下界约 0.512～0.552 m，gyro_z RMS 约 0.0934 rad/s；该估计依赖平地/无滑移，不是硬件合格性结论。未记录 safety_fault、torque_handoff_failed 或故障锁定。

## 结论和边界

本次可确认既定短程无视觉启停链完成，并有用户实际姿态确认；因此可准备下一次人工视觉实验，保留原策略/步态/机械臂接管/停止门槛。没有修改接触阈值、极性、源时效或故障规则。

本会话 vision_used=false、dynamic_tcp=false，不能证明真实胶带选择、视觉延时、方向修正、后方 TCP 历史接入或长时跟踪精度。未证明所有起步晃动和历史 B 等待均已解决，hardware_qualified/precision_qualified/visual_tracking_hardware_acceptance 继续 false。视觉实验应先站稳核对正确胶带，再人工短走、X 停、B 趴下；本轮工具不启动任何机器人动作。
