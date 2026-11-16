# V251 板载同步记录（2026-10-26）

已同步至 /home/unitree/rl_sar_b2_z1/diagnostics/va15_curve_entry_v251_20261026。
保留旧版本和原主程序；本地 V250 及其完整结果保留。没有启动相机、Z1 服务、控制程序或实机运动，没有释放厂商控制。

## 板载检查

- Python 62/62 通过。
- 近前视观测范围、方向、新旧历史、不外推、不跨帧补空缺和时效检查通过。
- 机身/TCP 实际历史及六组几何运动测试通过。
- 停止恢复测试用 PC4 原生编译/链接参数构建并通过：纯偏航瞬态继续恢复固定站姿；真实失联和滚转/俯仰异常仍锁止。仅内存测试数据，无机器人接口初始化。
- 256 条、每条 50 点的满缓存组合计算（前视 + 脚下参考 + TCP）共 1200 次，P99 1.712 ms，最大 2.341 ms。不是实际网络/相机/线程负载下的控制周期验收。
- 独立主程序已在板载重新编译；VA15 70/18 golden 最大误差 1.14441e-05，在既有浮点容差内。
- run_visual.sh --offline-check 通过；相机标定/视觉模型指纹由原入口保留，未启动相机验证当前地面。
- 本地和板载候选源码、脚本、资格快照及 README 的指纹一致；原主程序、base.yaml、VA15/config.yaml 和策略权重均未变。

板载 bundle SHA-256：6ae5816d271128854f43507c88eb2e7dc0d0a72f58da7da5e528c28d32aaeba0。

板载独立可执行文件 SHA-256：08aca06cf62613f90c5965af78df360bc86fdccb280daf2b644f8717ab2116f6。

证据保存在本地 outputs/board_checks/，其中 board_verification.json 记录源码核对、文件指纹及检查日志。
qualification.json / README.md 已在编译前冻结，不能事后改动被 bundle 绑定的文件来冒充重新验证。

## 人工测试入口

在板载 NoMachine 桌面终端执行，先确认 B2 实际趴稳、旧控制退出、无枪/软管及收臂路径无遮挡：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_curve_entry_v251_20261026/run_visual.sh eth0 --walk
```

只有你明确输入大写 RUN 后，程序才会启动实机接管流程。按 A 起立，实际站稳后按 Y 待机；确认窗口中的点在正确胶带上，再按 R1+Y 开始。X 停止、B 停稳后趴下；确认实际趴稳且 Passive 后 Ctrl+C 退出。沿用原按键，没有新增窗口 R、静态审查、短距离、四秒或单次机会限制。

这次使用 V251 独立入口，不是未替换的旧通用脚本，也不要继续用 V249 验证已知异常。

TCP 仍只使用实际观测历史，初始未看见的后方不会生成目标；相应历史到达后才出现 active=1。仿真三维跟踪误差仍约 7 cm，当前仅为人工实验候选，hardware_qualified / precision_qualified / live_robot_tested 仍为 false。
