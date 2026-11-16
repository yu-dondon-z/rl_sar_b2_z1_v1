# R1+Y 机械臂接管候选修复

本补丁只修改 Python 执行器参考值和反馈冻结处理，不改起立/Y 展臂增益、模型、原始动作历史、视觉时效和相机外参，不需要重新编译 C++。

## 已定位的问题

20261107_133738_884367 日志中，策略接管后 joint2 参考值曾在约 20 ms 内变化 0.214 rad；最后电机反馈首次冻结的同一采样周期出现 0.422 rad 参考值变化。本地控制器持续回包不能证明机械臂电机反馈更新。原程序检测到缓存的运动向量后只打印提示，仍继续运行策略。

## 修改

- 仅对 tracking 的六个机械臂下发位置参考值限速 1 rad/s，即正常 50 Hz 下每步最多 0.02 rad。起点为上一条下发参考值；调度延迟不会累积补偿成大跳变。腿部原逻辑和原始策略历史不变。
- 1 rad/s 是本轮离线候选值，不是厂家保证值，也不是加速度/力矩限制。应通过实际执行器匹配的闭环验证确认影响。
- 电机 q/dq/tau 完全重复至少 0.2 秒且仍报告速度大于 0.05 rad/s 时，锁定 held，停止策略推理，保持上一条位置和增益。一个新回包不会自动恢复；此锁定下 A/Y/B/R1+Y 不发起新动作，L1+X 及原有 native 故障处理仍有效。
- 这是停止使用已知可疑反馈的措施，不是通信恢复功能，也不能保证动态姿态下静态保持的稳定性。底层随后进入 PASSIVE/native fault 时，仍执行原故障处理，不能依赖本补丁自动安全趴下。
- 日志新增 arm_reference_rate_limited、arm_reference_rate_limit_rad_s、motor_feedback_blocked；raw_action、测量速度、控制器回包时间保持原样。

## 验证与使用

安装前先关闭控制程序。安装器逐文件核对旧版 SHA256，保存旧文件及 MANIFEST，遇到本地修改停止覆盖；不改 config、assets 或 runs。安装不会启动机器人。

```bash
python install.py --target /home/unitree/rl_sar_b2_z1_VD2
cd /home/unitree/rl_sar_b2_z1_VD2
.venv/bin/python run.py --offline-check
.venv/bin/python tools/replay_arm_handover.py --session runs/20261107_133738_884367
```

日志回放只能验证同一记录输入下的命令限制和冻结阻断，不能代替闭环或实机测试。旧日志中的 rear work x=-1.02 m 缺少深度支持是独立的视觉覆盖故障，本补丁不伪造支撑点、不延长过期时间。底层 Z1 停止回包的具体触发机制仍需双向 UDP 抓包或实际电机故障信息确认。
