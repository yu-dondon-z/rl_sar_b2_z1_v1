# VA15 实机支持反馈排查：只读候选（2026-10-20）

此目录新增独立诊断工具，不覆盖当前 B 停止候选。未改 actor、站立/行走
增益、FSM、停止门限或两个真实来源 validated=false。**不能用这个工具的
结果直接授权行走/下降**，也没有自动标定、启动运动或写入运行配置的入口。

## 为什么没有直接解锁

补充核实：[宇树官方 unitree_mujoco 的说明](https://github.com/unitreerobotics/unitree_mujoco#supported-unitree-sdk2-messages)
明确指出，实机关闭内置运动控制服务后不能读取 SportModeState，模拟器
则保留它用于分析。因此不能继续把“低层接管后等待 sport 恢复”当作
排查目标。生产停止反馈对 sport_fresh 的硬依赖是待替换的设计缺口，
并非可以让用户重启运动服务来解决（不可造成控制器争抢）。

当前运动入口需要真正有效的四足支持、高度和速度，不只是站立姿态。
低层控制期间 `rt/sportmodestate`、`rt/lf/sportmodestate` 都未提供可用新鲜
数据。先前采样里 FL 原始桥路信号与趴卧分布重叠，静态 A/Y/B 通过也不
能证明动态停止有足够反馈。直接改变 false 或降低阈值会隐藏这个问题。

## 本次实际做的工作

从之前板载实际 URDF 提取四腿关节变换、质量、质心、球形脚几何，源文件：
`../va15_stop_feedback_20261020/board_results/model_sources/board_b2_z1.urdf`，
SHA-256 `4bccdee2bfec530e0d8e22525ca9d9a5aea8ca7c1e1a7a4eadd979ee9ca2fb9f`。
不猜腿长、不使用策略的期望关节角或期望力矩代替反馈。

在静态模型下计算向上地面反力的等效值：

```text
tau_measured = g_leg(q, R) - J_foot(q)^T f_ground
f_equivalent_world = R (J_foot^T)^(-1) (g_leg - tau_est)
```

`g_leg` 由各关节下游连杆和脚的重力力矩求和，不把自身腿重误当成接触。
负向力保留符号；不使用 abs() 来“修复”接触。Jacobian 病态、输入非有限、
越关节限位、状态过期或跳序均拒绝有效结果。加速度、关节速度/加速度、
IMU 角速度和电机模式/lost 字段另作准静态适用性检查。

这条路线参考[关节力矩选取可靠支撑足的研究](https://arxiv.org/abs/2602.17393)，
但这里不是该论文的完整滤波器。当前仅调查静态承重指标；没有实现并验证
惯性/摩擦补偿、足接触概率或实机控制用状态滤波器。

限制必须保留：力矩测量比例/偏置、传动摩擦、关节与机身加速度、非足端
接触、外部约束、脚垫变形/滚动/打滑，都会影响反推力。整机共同滑动不能
仅靠脚间运动学一致性排除。姿态标签、URDF 数学正确或各脚值分离均不是
实际接触/外部身高/速度真值。静态代用品不能直接推广到动态停止捕获。

## 验证与复现

```bash
python3 -m unittest -v test_support_candidate.py
python3 validate_support_mujoco.py \
  ../va15_stop_feedback_20261020/board_results/model_sources/board_b2_z1.urdf
```

17 项单元测试，本机和 PC4 均通过。涵盖向上/向下力符号、自身重力、逐足
卸载、旧日志缺字段、姿态/速度/加速度异常、数据陈旧/传输故障、模型顺序
和来源、禁止生成控制权限等。64 组独立 MuJoCo 静态重力/Jacobian 参考通过：
力最大差约 2.85e-13，重力力矩最大差约 8.89e-15。**不是实机精度证明**。

旧静态采样回放 `outputs/old_pair_review.json`：逐腿等效垂向力趴卧中位数
约 12–13 N-equivalent，站立约 207–270 N-equivalent，存在明显分离。
但总等效垂向力约 956，URDF 自重参考约 775，相差约 23.4%；不能假设
传感器比例、模型质量或外部载荷哪一个正确，不自动乘校正系数。旧日志
缺少加速度/ddq/电机健康字段，所以只能作数值回放，不能完成适用性验证。

## 板载安装与操作边界

新工具在 PC4 的独立目录：
`/home/unitree/rl_sar_b2_z1/scripts/support_probe_20261020_LtTVai/`。
只依赖板载已有 Python/NumPy/Unitree Python SDK；不需要编译控制程序。
生产主源码、FSM、VA15 配置和 policy 哈希更新前后完全相同。

操作者确认趴卧且控制已退出后采集了新的完整趴卧数据：
`outputs/stop_diagnosis_20261020/prone_full_01/`（PC4 仓库相对路径），
976 帧、975 帧通过数值/准静态检查，首帧按序列预热无效。两条 sport
通道有效源时间样本均为 0。没有启动 Z1、释放官方控制或发布 lowcmd。

操作者随后确认已按既有流程站稳，允许只读站立采样：

```bash
cd /home/unitree/rl_sar_b2_z1
python3 scripts/support_probe_20261020_LtTVai/probe_b2_support.py eth0 \
  --seconds 10 --posture custom_stand \
  --output outputs/stop_diagnosis_20261020/stand_full_01
```

该程序不让机器人站起，也不接管或停止现有控制。标签必须对应操作者
实际确认状态。不要为了采样推摇杆、启动 actor、视觉模式或手抬承重腿。
采集后按已通过的 B 流程趴下，实际趴稳且 Passive 后才退出。

比较一次静态采样（只生成审阅报告，**不会生成标定或权限**）：

```bash
python3 scripts/support_probe_20261020_LtTVai/compare_support_captures.py \
  --prone outputs/stop_diagnosis_20261020/prone_full_01/samples.csv \
  --standing outputs/stop_diagnosis_20261020/stand_full_01/samples.csv \
  --output outputs/stop_diagnosis_20261020/static_pair_review.json
```

所有输出使用新目录/文件，拒绝覆盖已有采样。采样器同时保存原始力桥、
foot_force_est、tau_est、q/dq/ddq、IMU 加速度/角速度/姿态、mode/lost、
两条 sport 的接收年龄和源时间推进年龄，方便区分报文收到与来源真实更新。

## 尚未完成的行走资格

需要独立接触/卸载证据、力矩比例与偏置验证、动态惯性项处理、脚滑动和
高度/速度精度验证，以及动态 B 停止验收。不能把当前静态候选直接接入
`real_stop_feedback`，不能打开两个 validated 位。此目录的
`CONTROL_AUTHORIZED=False` 没有运行时修改入口。

## 新鲜站立采样结果

操作者确认站稳后，只读采集了 `stand_full_01`，989 帧，988 帧通过模型
及准静态适用性检查（首帧序列预热无效）。机身几何高度假设中位数
0.51313 m，无滑动脚假设下平面速度 p95 0.00520 m/s；仍不能称为独立
实测高度和速度。两条 sport 的有效源时间样本都为 0。

本次逐腿向上力等效值中位数约 208/228/251/264，而趴卧约 13/13/14/12。
总等效力约 950，与 URDF 自重参考仍差约 22.6%。不生成力标定或授权。

原始桥路信号的跨轮次问题并未消失：

| DDS 腿 | 旧趴卧→旧站立中位数 | 新趴卧→新站立中位数 |
| --- | --- | --- |
| FR | 486→512 | 490→544 |
| FL | 492→500（旧站立有重叠区间） | 495→480 |
| RR | 537→495 | 539→552 |
| RL | 514→552 | 517→474 |

关节和力矩对应的姿态接近，但原始桥路的变化方向不能跨轮次一致解释。
这不证明硬件损坏，也不能猜测通道重排或反转符号强行标定。进一步需要
厂家确认通道物理含义/桥路标定，或独立地验证替代接触估计。

本次完整报告 `outputs/current_static_review_v2.json`；保留了旧版回放结果，
其中浮点字符串计数（`"0.0"`）解析曾使回放拒绝准静态标志，已修正并新增
序列化回归测试。该回放 bug 不影响已采集的原始 CSV，也未用于控制授权。

操作者已确认再次趴下并退出。工具源码已保留版本备份后更新；没有修改
生产控制程序或重新授权行走。

返回趴卧后再采集 `prone_full_02`：981 帧、980 帧通过模型/准静态检查，
等效力再次降至约 12–14，几何高度假设约 0.1117 m。原始 CSV 全部保留。
完整三姿态循环（趴卧→站立→趴卧）并不替代独立逐足卸载真值或动态验证。
