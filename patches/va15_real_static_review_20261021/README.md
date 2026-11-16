# 2026-10-21 实机静态只读复核

本轮接续本机 B 停止离线候选，但没有将模拟适配或停止控制器同步板载。
不更改控制源码、配置、权重、增益、标定开关；不启动 B2/Z1 控制。
后续站立、趴下均须由操作者执行已验证流程，不自动发运动指令。

## 本轮已完成

- SSH 确认主机 `unitree-b2-pc4` 可达；初次检查没有主控制或 Z1 控制进程。
- 主控制源码、FSM、VA15 配置/权重、独立估计器源码哈希与此前记录一致。
  两项旧 validated 位仍为 false；`independent_B_stop_candidate` 保持原有 true。
- 用户明确确认趴卧后，调用既有修复后的只读探针，约 10 秒订阅 lowstate。
  没有发布 lowcmd、释放官方运动服务或初始化 Z1 控制接口。

板载原始记录：
`/home/unitree/rl_sar_b2_z1/outputs/independent_state_check_KZxiuF/`。
本地副本：`prone_KZxiuF/stream.csv`、`prone_KZxiuF/stream.json`。

978 帧、约 9.980 秒；fresh 978/978，tick 无重复/倒退。
首帧输入/连续性检查未就绪，其余 977 帧输入有效、连续。
全部 `load_DDS=0000`，初始化、有效估计、停稳候选均为 0。
高度/速度保持未定义，没有将趴卧的静止误报成站立；探针正常退出，退出码 0。
这只是已确认趴卧场景的负例复核，不是接触或动态停止验收。

## 站姿对照：出现输入失效，本次不通过连续静态复核

用户明确确认已站稳后，只读采样 10 秒；用户随后确认期间没有按 B、
切换遥控模式或改变姿态，结束时仍实际站稳。本次没有发送运动命令。

板载：`/home/unitree/rl_sar_b2_z1/outputs/independent_state_check_6wYQoC/`。
本地：`standing_6wYQoC/stream.csv`、`standing_6wYQoC/stream.json`。

- 985 帧；约 0.546 秒初始化，0.556 秒起形成可审查估计。
- 前段有效估计 552 帧、停稳候选 550 帧；该有效片段的高度中位数
  0.514816 m、速度 p95 0.00134144 m/s，不能代表整段一直有效。
- 从 6.16755 秒起，连续 379 帧 `Invalid motor/transport state`，估计器
  重置，输出无效并清空承重候选。此时 `0000` 不是四脚物理离地证据。
- 探针正常退出（exit=0）仅表示工具退出正常，**本次站姿连续复核不通过**。

随后独立只读订阅三次核对电机字段（未初始化控制器或发布命令）：
12 个 mode 均为 10，只有 DDS motor[1]（右前大腿）lost=5，其余为 0。
三次的 tick 分别 2486556、2487556、2488558，lost=5 未继续增长。
对应 motor[1] q≈0.77972 rad、dq 绝对值 <0.00058 rad/s、tau≈9.08–9.26；
全腿最大绝对力矩约 66.3–66.6，未触发 360 力矩范围或 30 rad/s 速度范围。
相关精确快照见 `standing_6wYQoC/motor_transport_check.json`。

现有 `ValidateSample()` 将任何 `lost!=0` 直接视为无效，因此该字段是
已确认的触发项。不能仅凭绝对值 5 宣称仍持续丢包，也不能未经字段语义
核实就忽略它或修改 validated 位。还需离线核查 lost 的语义、增长与时效
关系；本次没有修改估计器、阈值、生产程序或运动权限。

站姿采样结束后，应由操作者使用原 B 流程趴下；确认实际趴稳且终端
Passive 后再 Ctrl+C，禁止在仍站立时为清字段而重启或断电。

### 操作者已退出后的补查

用户随后确认已趴下并退出。只读确认主控制进程已退出，Z1 后台服务
未操作；另补采 15 秒原始字段，1471 帧中 motor[1] lost 始终为 5，
没有增长，其他电机 lost 为 0。整帧 tick/时效正常，但这不能单独证明
逐电机健康。未修改估计器的原始数据离线回放仍持续拒绝。

脚本、单测、官方接口核对和需确认的固件语义见
`../va15_lost_audit_20261021/README.md`。当前不放行行走，也不为清零
该字段而重启机器人；后续先取得字段语义及逐电机状态的可靠定义。

## 本轮采用的采样流程与退出边界

确认无枪/软管、Z1 安全放置、周围无人并满足既有静态测试条件后，由
操作者在板载原控制终端使用已验证 A 起立流程。只站立，不按 Y、R1+Y，
不推动摇杆。已在操作者明确确认实际站稳后完成上述站姿对照数据采集。

保持原控制进程运行，由只读工具订阅，不在站立中替换或重启控制程序。
结束时由操作者按 B，确认实际趴稳、终端 Passive，再 Ctrl+C。
没有实机动态运动资格；静态高度/速度估计也不能代替独立物理参考。

## 本次读取的板载来源 SHA256

```text
5228618f2be9094feb55cd5d8d6a1acd048792821600cfcf5adf28fbd31f64c8  src/rl_sar/src/rl_real_b2_z1_no_gun.cpp
435b18eddc2381c53ce7e31ab93386eaa86393e5c9eeb37841a942649e131cf3  src/rl_sar/fsm_robot/fsm_b2_z1_no_gun.hpp
8faa51628b3704ce3e7e3a384f1c0eb8bb59b7dcf5a0069992069efed8aab8d1  policy/b2_z1_no_gun/VA15/config.yaml
8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098  policy/b2_z1_no_gun/VA15/policy.pt
e76a733a3428ebaaa27b792b4005a69aac18e0eac2eb0824d9762379a707a3f9  diagnostics/independent_state_20261020_L7VGqH/va15_independent_state.hpp
ba28211b0433207144ff8e2ecdeffead18084a8aee3457bf67bfc47fc9bf9028  diagnostics/independent_state_20261020_L7VGqH/independent_probe.cpp
```
