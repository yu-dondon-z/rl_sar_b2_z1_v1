# 行走前反馈替代路线复核（2026-10-21）

用户要求重新检查以准备行走，或适当放宽。此轮选择重新测量实际数据和
查找替代来源，不伪造接触、标定状态或时间戳，不关闭失效检测。
操作者回复：没有重启或切换官方控制，目前趴卧。

## 当前观测发生变化，不能沿用上一轮 lost=5 结论

板载：`outputs/feedback_routes_20261021_yFSyg4/routes.json`；
本地：本目录 `outputs/initial_yFSyg4/routes.json`。

- 只读调查约 10.05 秒，lowstate 5012 条，时间连续、无倒退，无解析错误。
- **此次 lowstate 中 12 个电机 lost 均为 0**，所有 mode=10；没有改写原始值。
- 与上一轮 lost=5/tick 数百万相比，当前 tick 约 628148–638170。
  操作者否认重启/切换。当前 DDS 发现只有一个 lowstate 发布者，不代表
  已解释此前源时基变化或排除历史上的发布者重启/切换；原因仍未知。
- 用完全未修改的 C++ 严格估计器再次订阅，输出
  `outputs/independent_state_check_QYbmwj/`：978 帧，正常退出，没有电机
  输入失效，趴卧时不初始化四足支撑、高度/速度保持未定义。

因此当前不再需要为 lost=5 放宽校验，也不能认定它永久恢复。下一轮
保持原有非零 lost 拒绝，观察起立/控制接管过程中是否再次出现。

## 主动检查了此前未用到的反馈通道

ROS2 只读发现及实际订阅结果：

| 反馈 | 本次消息 | 关键限制 |
| --- | ---: | --- |
| rt/lowstate | 5012 | tick 连续；足端 raw 不是接触布尔值 |
| rt/sportmodestate | 5010 | source stamp=0；当前趴卧 body_height 约 0.0806 m |
| rt/lf/sportmodestate | 200 | source stamp=0 |
| rt/odommodestate | 5011 | source stamp=0，body_height=0，不能填入有效离地高度 |
| rt/lf/odommodestate | 200 | 同样无有效源时间戳，body_height=0 |
| rt/lio_sam_ros2/mapping/odometry | 0 | 发现只有订阅者、无发布者 |
| rt/lio_sam_ros2/mapping/re_location_odometry | 0 | 未接收样本 |

收到持续数据不等于字段物理含义已验证；不能用 position.z 代替离地高度，
不能把接收时刻伪装成源时间戳，不能从近零速度直接认定已站稳。

另外通过 CycloneDDS 自描述元数据动态读取了 rt/EstimatorData，未猜测
二进制布局，也没有操作其发布端。其真实字段为：

```text
contact_estimate[4]
position_2d[3]
position_3d[3]
v_body_3d[3]
v_world_3d[3]
```

当前趴卧样本 contact_estimate=[0.5,0.5,0.5,0.5]、position_3d.z≈0.18779，
消息没有源时间戳。未把 0.5 当作四脚接触，未给这些未知字段控制权限。
仅凭该包目前不能替代接触和离地高度证据。

## 采样器边界

probe_routes.py 只订阅已知标准 IDL 的 7 个通道、限定 0.5–30 秒、拒绝
覆盖输出，保存原始值和源时基/接收时刻的区别；不发任何控制/服务请求，
不加载策略，不启动 Z1。物理验证和控制权限固定为 false。7 项离线测试
覆盖无数据、零时间戳、重复/倒退、非有限值和只读接口。

```bash
cd /home/dong/rl_sar_b2_z1/patches/va15_feedback_routes_20261021
python3 -m unittest discover -s . -p 'test_*.py' -v
```

本次通过 SSH stdin 运行，没有安装新板载入口；未改生产代码/二进制/
VA15 权重/配置/增益/停止门槛。保持此前正常的 A/B 并行趴下版本。

## 下一步

由操作者使用既有启动器输入大写 RUN，A 起立并确认四脚实际落地，保持
零速度，不按 Y/R1+Y、不推摇杆。随后只读采集严格独立估计、原始 q/dq/
tau/IMU/足端和所有替代通道，核对是否再次出现 lost、tick/源时基重置，
以及当前站姿是否形成连续估计。若本轮仍不满足条件，不直接放宽接触
门槛。通过静态数据也不自动等同动态验收，需再准备并验证受限测试方案。

## 站姿采样已完成

操作者确认站稳后，未修改原生估计器的 1475 帧中，0.556 秒预热后持续
有效，估计高度中位数 0.514616 m、平面速度 p95=0.001180 m/s。另两组
原始记录中电机 lost 均为 0。操作者随后确认实际趴稳并退出，5 秒负例
没有形成站立支持；没有在本轮更改生产或放行行走。
完整窗口、统计、限制和后续实现项见 [standing_assessment.md](standing_assessment.md)。
