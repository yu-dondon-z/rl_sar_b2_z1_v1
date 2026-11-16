# B2 lost 字段只读核查（2026-10-21）

## 结论与边界

用户确认安全趴卧并退出后，SSH 确认原主控制进程已退出；既有 Z1
后台服务仍在，未操作它。主程序/配置哈希保持不变，两项旧 validated
位仍为 false。本次没有启动控制、发布命令、清零字段或更改门槛。

**已确认的直接触发项：**既有 `va15_independent_state_20261020` 的
`ValidateSample()` 将任何电机 `lost != 0` 视为无效。上一轮站姿 10 秒
在约 6.17 秒后失效；随后查到右前大腿 motor[1] lost=5。

用户确认站姿采样期间没有按 B/切换模式/改变姿态，故不能把失效归因于
操作者趴下。记录在 `../va15_real_static_review_20261021/`。

## 本轮 15 秒趴卧原始记录

板载：`/home/unitree/rl_sar_b2_z1/outputs/lost_audit_v7uyVL/raw.csv`、`raw.json`。
本地完整副本及未修改估计器的回放：`outputs/prone_v7uyVL/`。

- 1471 采样帧，持续约 14.988 秒；右前大腿 lost 全程为 5、变化次数 0；
  其余 11 关节 lost 全程为 0。
- 无 tick 重复/倒退或 >50 ms 间隔；没有源年龄 >100 ms 的帧。
  最后源年龄约 0.00093 秒。这是整帧数据时效，不证明每个电机更新正常。
- 1471 帧基本电机检查全部被 lost=5 拒绝，未出现其他基本电机拒绝原因。
  CSV 保留全部 q/dq/tau、mode、lost、reported ddq、温度、reserve、
  IMU、时刻和具体错误项，不把原始非零字段改写为 0。
- 将原始 CSV 输入**未修改的 C++ 估计器**离线回放：1471 帧、有效估计
  0、停稳候选 0。探针和审计工具均正常退出。
- 只读查询时主机 eth0 的累计 RX/TX error 均为 0。这不能排除电机内部
  总线异常，也不能用来认定 lost 字段失效或可以忽略。

这些证据只证明 **lost=5 在本轮观察期间保持不变，并继续触发既有严格拒绝**。
尚未证明它是累计丢包数、当前丢包率、锁存标记，或该固件何时复位。
不能宣称“只是历史计数、已修复、当前通信合格”或据此放开运动。

## 官方接口核对

本地配套 SDK 的 `include/unitree/idl/go2/MotorState_.hpp` 将 lost 定义
为 uint32_t，只做序列化/访问；未在该文件解释累计与复位规则。
本次查到的[宇树官方 ROS2 消息定义](https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/MotorState.msg)
同样只列 `uint32 lost`，不足以认证该字段在当前 B2 固件上的含义。
官方 Basic_services 动态页面未能读取到字段说明；没有拿其他机器人
型号的字段含义当成 B2 认证结果。

需要宇树确认当前 B2 固件中该字段的含义、复位条件、是否单调累计，
以及何种字段可独立证明逐电机新鲜度/故障状态；**不建议为消除数字而重启**。
如确认累计语义，才进一步设计“原值保留、变化事件、源时效、逐电机状态”
的候选判断，并做新离线/只读复核，不能直接把 `lost!=0` 删除。
在此之前不再反复起立试走，不修改 validated 位或接触阈值。

## 新增诊断工具

`audit_lost.py` 仅包含 rt/lowstate 订阅器；没有 publisher、运动客户端、
Z1、策略或估计器控制入口。首次非零、增长、下降/可能回绕均单独记录，
不将下降硬解释成回绕或恢复。重复 tick 不刷新源年龄。
所有结果固定 `control_authorized=false`、`hardware_qualified=false`。

脚本首次参数检查和单测可完全离线，不导入宇树 SDK：

```bash
cd /home/dong/rl_sar_b2_z1/patches/va15_lost_audit_20261021
python3 -m unittest discover -s . -p 'test_*.py' -v
python3 audit_lost.py --help
```

5/5 单测通过：恒定非零仍拒绝、增长/下降不被隐藏、模式/数值/速度/
力矩异常、无数据/陈旧/重复/倒退，以及仅允许只读 SDK 导入。
本次通过 SSH 标准输入执行此脚本，**没有安装板载脚本或更换控制二进制**；
板载只新增上述独立诊断输出。生产估计器和停止控制器均未修改。
