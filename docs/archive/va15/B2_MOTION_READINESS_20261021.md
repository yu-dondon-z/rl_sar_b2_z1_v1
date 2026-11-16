# B2 当前行走许可链复核（2026-10-21）

## 已通过与未通过要分开

操作者反馈新的并行趴下测试正常。本轮仅记录该人工静态反馈；没有新的
运动录像、定量实测或动态停止验收，因此不将其记为行走资格。
本轮 SSH 查询未发现主控制/启动脚本运行；未起立、未驱动 Z1、未发送
lowcmd、未释放官方服务，也未修改生产代码、策略、配置或二进制。

当前两条不同的判断链：

```text
生产真实起步
  -> ConfigureStopFeedback 读取足端标定/高层反馈验证状态
  -> VA15StopFeedbackTracker::Snapshot 标记 contacts/height/velocity 是否可用
  -> RLLocomotion::RealStopSettled 连续 100 个控制周期（0.5 s）
  -> 显式重新请求运动；不能持续推杆等待自动放行

独立估计器候选（仍是诊断/离线候选，不提供生产运动权限）
  -> ValidateSample 校验 IMU、mode、q/dq/tau、lost 等
  -> 动力学承重候选和条件高度/速度估计
  -> 审查结果；authority=0，不自动替代生产验证位
```

生产配置的两个验证位仍为 false、足端校准 ID 为空；因此即使 raw 足端
数值看起来正常，也不能通过当前生产门槛。lost=5 则是独立估计候选
持续失效的明确原因，不是“生产起步只差把 lost 清零”。不能混淆两者。

来源：
- `patches/va15_stop_parallel_20261021/production/rl_real_b2_z1_no_gun.cpp`
  的 ConfigureStopFeedback，以及同目录 FSM 的 RealStopSettled/起步门槛。
- `patches/va15_stop_feedback_20261020/work/va15_stop_feedback.hpp` 的 Snapshot。
- `patches/va15_independent_state_20261020/va15_independent_state.hpp` 的 ValidateSample。

## 这次新增的只读记录

板载 `outputs/lost_followup_20261021_UjAM3a/raw.csv`、`raw.json`；本地副本
`patches/va15_lost_audit_v2_20261021/outputs/followup_UjAM3a/`。
姿态标记为 unknown，不擅自把关节外形解释为操作者确认的趴卧或站立。

- 约 9.990 秒、985 帧，退出正常。
- DDS motor[1]（右前大腿）lost 全程 5，其他 11 个关节为 0。
- 没有计数变化、tick 重复/倒退/>50 ms 间隔，源年龄 >100 ms 的帧为 0。
- 985 帧基本电机检查均因 lost=5 拒绝，不代表其他各层输入已获认证。
- 没有把原始 lost 改写成 0，也没有把固定非零解释成当前一直丢包或确定的历史计数。

板载当前 FSM/config/actor 哈希保持本次并行版本：

```text
FSM    5ff38043ccab5ceeaeec09bf5828ac26da2b093a72a725d9cd43f132a0a91d94
config 8faa51628b3704ce3e7e3a384f1c0eb8bb59b7dcf5a0069992069efed8aab8d1
actor  8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098
```

## 当前不能靠软件猜测解决的条件

再次检索的[官方 ROS2 接口说明](https://github.com/unitreerobotics/unitree_ros2/blob/master/README%20_zh.md)
仅列出 uint32 lost，没有给当前 B2 固件的累计/锁存/复位规则。本地配套
SDK 的 MotorState 也只有该字段的存取和序列化。不能拿 G1/Go1 的解释
替代当前 B2，或以“长时间没变化”认定电机健康。

下一步需要操作员获取宇树 App 的电机告警信息和准确固件版本，并请厂商
确认以下问题。App 没有告警本身也不能代替下面的字段契约。

可直接转发给技术支持：

> B2 通过官方 SDK2 订阅 rt/lowstate，DDS motor_state[1]（右前大腿）
> mode=10、lost=5，其余 11 关节 lost=0。多轮 10/15/30 秒采样中 lost
> 始终未变化，tick 正常，q/dq/tau 有更新。一次站姿采样此前正常，约
> 6.17 秒后进入同样的持续拒绝状态。
> 请确认本机固件中 lost 是累计丢包、当前故障、锁存状态还是保留字段？
> 累计/复位规则是什么？哪些字段能提供逐电机反馈年龄和当前故障状态？
> 若是累计计数，官方建议的当前失联检测方法是什么？请一并核对本机固件版本。

在得到可靠解释前，不继续重复起立/试走，不提高增益，不降低接触门槛，
不将 validated 位设为 true，也不为消除数值而清故障或重启机器人。
若厂商确认累计语义，再设计“原值保留 + 计数变化 + 心跳/电机状态”新
候选并进行离线故障注入，不能直接删除 lost!=0。

生产门槛的另两项还需要接触证据验证和低层模式下的速度/高度有效性，
或经过验证的独立估计反馈适配；解决 lost 仅为后续候选验证的前置条件。

## 后续重新实测：lost 当前为 0

用户要求再次检查或放宽条件后，扩展只读发现/订阅：5012 条 lowstate
中 12 个电机 lost 都为 0，原生未修改估计器的 978 帧采样不再出现
电机输入失效。操作者确认趴卧，原估计器未错误初始化站立支持。
但 tick 比此前小，操作者否认重启/切换，源时基变化原因仍未知。

本轮不再把旧的 lost=5 作为当前持续事实，也没有为其关闭校验。
新增 odommodestate 通道有数据但源时间戳/高度为 0；SLAM 里程计无
发布者；EstimatorData 的接触数组在趴卧时为 0.5，没有源时间戳，均
不能直接当作合格反馈。生产限制及 A/B 版本未改。详见
`patches/va15_feedback_routes_20261021/README.md` 和原始记录。

## 操作者确认站稳后的当前静态复核

已完成 15 秒原生站姿采样：1475 帧，预热约 0.556 秒后 1421 帧持续
有效、1419 帧停稳审查候选；高度估计中位数 0.514616 m、速度 p95
0.001180 m/s。另行原始记录 lost 始终为 0。操作者随后确认 B 趴稳
并退出，5 秒冷启动负例未误认站立支持。

现在可以继续准备独立反馈控制的受限候选，不必因历史 lost=5 停滞。
生产仍未接入这一诊断估计源，没有自动赋予动态运动资格。详细限制、
时间窗（各组不是逐帧同步）和下一阶段必须完成的实现/测试见
`patches/va15_feedback_routes_20261021/standing_assessment.md`。
