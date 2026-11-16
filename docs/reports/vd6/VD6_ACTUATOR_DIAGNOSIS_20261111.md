# VD6 关节振荡、落脚关联与执行器诊断

2026-11-11。本轮只做本地分析和离线仿真。VD5 4498 仍为部署基线；没有训练、替换导出 actor、同步板载或启动机器人。

同日后续已完成原日志闭环时序与离线 actor 敏感性分析，见 [VD6_ACTOR_FEEDBACK_20261111.md](VD6_ACTOR_FEEDBACK_20261111.md)。新证据把重点进一步收敛到 joint1 自反馈关系和 joint1→joint6 耦合：在相同记录输入上，VD6 的 joint1 自位置/速度局部负导数绝对值较 VD5 减少约 49%/29%；MuJoCo VD6 joint6 目标高频能量中，joint1 速度/位置的有符号协方差份额合计约 69%。这些是离线归因与敏感性证据，仍不是闭环根因确认；该阶段没有新物理仿真或策略改动。

用户随后要求收敛测试并同意新版本短训。VD7 已从 VD5 4498 追加 100 轮至 4597，精简 MuJoCo 筛选未通过，已停止追加；详见 [VD7_SHORT_TRAINING_REVIEW_20261111.md](../vd7/VD7_SHORT_TRAINING_REVIEW_20261111.md)。没有执行此前建议的进一步小扰动物理归因实验，部署基线仍为 VD5。

其后用户同意且已完成**一次** MuJoCo 0.80 m 机身滚转物理脉冲配对（VD5/VD6 各自零扰动对照）。VD6 在该工况下的 joint1/6 和枪尖横向增量更大，但两版脉冲时四足支撑相位不同，不能据此单独确认 actor 因果根源。见 [VD6_ROLL_PULSE_REVIEW_20261111.md](VD6_ROLL_PULSE_REVIEW_20261111.md)；上段“没有执行”特指 VD7 短训截止时的历史状态。

用户随后同意考虑训练收敛问题，已从 VD5 4498 独立运行 VD8 600 轮，保留 100 轮检查点并做早/中/末 MuJoCo 对照。训练曲线不单调，0.70/0.80 m 也没有同一 checkpoint 同时改善抖动、速度和逐足冲击；已停止追加，详见 [VD8_ROLL_LANDING_REVIEW_20261111.md](../vd8/VD8_ROLL_LANDING_REVIEW_20261111.md)。

## 本轮结论

横向枪尖运动主要收敛到 joint1、joint6，且 VD6 在 MuJoCo 中生成的臂目标序列本身已携带更强振荡。固定底座、去除地面和 actor 后，同一 MuJoCo VD6 目标序列在两端都产生约 10.6 mm 的 Y 向高频 RMS；换成 Isaac VD6 的目标序列，两端降至约 3.2–3.5 mm。因此，单纯把退化归因于 MuJoCo 机械臂自身响应或某次限速触发，证据不足。

这还没有确定闭环振荡的最初触发源。原目标序列包含当时的身体运动补偿和接触/编码器反馈效应，固定底座重放打断了这些关系；重放结果不能当成步行改善或适配完全正确的证明。

新增了双端 200 Hz 实际关节位置/速度和运行时受力读取，但**尚不能称为已取得两端完全一致的“纯驱动力矩＋精确限速乘子”**：安装版本的 PhysX 张量接口只能读到投影关节合力矩，其中包含驱动、摩擦、限位及限速反力；纯驱动力矩及独立约束活动字段保留 `null`，没有用 PD 估算补齐。MuJoCo 的原生执行器力也须区别于隐式积分后的等效步平均力。

## 先用已有数据定位

已按交接指定顺序阅读正式评估、训练说明、VD5 基线诊断和总结果。没有重跑世界路径任务。

分析复用 `comparison.json` 中 VD5 4498、VD6 4997、两端、0.70/0.80 m、速度指令 0.20 m/s 的八个会话。统一 5–30 s，3 Hz 三阶高通，滤波两端各去 1 s。关节仍只有 50 Hz，枪尖/足部为 200 Hz。

旧 VD5 日志部分没有单列关节字段，故从已记录 actor observation 恢复：`q-default=obs[16:34]`、`qd=obs[34:52]/0.05`，角速度为 `obs[10:13]/0.2`。这只是解码原有传感器记录，没有重演或修改观测。

在每段平均姿态用命名关节 Jacobian 分解枪尖 Y 向运动。局部线性化对完整 Y 向高频信号的解释方差均超过 99.8%。MuJoCo、0.80 m：

| 项目 | VD5 4498 | VD6 4997 |
| --- | ---: | ---: |
| joint1 角度高频 RMS | 约 0.0044 rad | 约 0.0103 rad |
| joint6 角度高频 RMS | 约 0.0050 rad | 约 0.0120 rad |
| joint1 对 Y 的局部贡献 RMS | 3.51 mm | 8.02 mm |
| joint6 对 Y 的局部贡献 RMS | 4.07 mm | 9.70 mm |
| joint1/joint6 合计 Y 协方差份额 | 约 108% | 约 96% |
| 枪尖 Y 在 50 Hz 对齐信号中的频谱主峰 | 4.0 Hz | 8.0 Hz |
| 横滚角速度与枪尖 Y 高频信号相关系数 | 0.657 | 0.684 |

贡献 RMS 不能直接相加，协方差份额也不是因果百分比；超过 100% 表示其他关节存在抵消。以上枪尖频谱和协方差使用降至 50 Hz 的对齐信号，不能替代原正式评估中 200 Hz 计算的 2.75/9.24 mm。

落脚按原定义：至少 60 ms 腾空、连续至少 10 ms >10 N 接触。已保存逐足事件、冲击排序、事件前后关节速度及枪尖 Y 速度。VD6 MuJoCo 0.80 m，joint6 速度事件均值峰约在 FL/FR/RL/RR 接触后 140/100/140/120 ms（50 Hz 分辨率）；新 200 Hz 短窗对应约 135/105/145/115 ms。部分 joint1 峰在接触前，部分在后。对角足事件接近，周期性重叠，因此不能判定某只后脚冲击为最初根因，也不能用双向滤波后的相位宣称先后因果。

## 新增 200 Hz 诊断与验证

八组各 15 s，初态、目标、源 PD、actor、动作缩放均沿用原条件；0.70/0.80 m、两端、VD5/VD6 各一组。每组 3000 物理步、750 actor 步。新增 `joint_response.jsonl.gz` 和 `joint_diagnostic_schema.json`，前者不进入 actor。

共同记录：命名 actor 顺序的 18 关节步前/步后角度、速度、该物理步持有的目标及目标时刻。分析窗口为 5–15 s，不替代原来的 5–30 s 正式评估。

MuJoCo：

- 通过可选的只读 observer，在原生 `mj_step` 之后、限速投影及 `mj_forward` 覆盖前保存 `actuator_force`、`qfrc_constraint`、原生约束类型/ID/状态/力和投影前速度。
- 另存质量度量限速冲量、每关节活动及速度改变量。约束冲量单位 Nm·s，不能称为驱动力矩。
- `drive_effort18_nm` 是原生引擎在该步计算的执行器力，不是 `apply_targets()` 返回的 PD 估算；其求值相位在步前。`implicitfast` 的速度导数修正意味着它不等于积分后等效步平均驱动力矩，不能与 PhysX 步平均受力直接作同口径饱和对比。

PhysX：

- 从实际运行时读取 160/5、120 Nm、3.1415 rad/s，确认配置确实进入引擎；腿的原生 stiffness/damping 为 0，对应显式 PD。
- `get_dof_actuation_forces()` 在臂部为 0：它是用户施加力，不能冒充隐式驱动力。
- `get_dof_projected_joint_forces()` 是实际关节传递力在自由度轴上的投影，单独标为 `joint_solver_effort18_nm`。独立驱动力矩、驱动饱和及精确限速活动无法从该接口分离，字段为 `null`。`speed_near_limit18` 只是接近限值，不是约束激活证据。

与八段旧日志前 15 秒逐样本比较，新增记录后的枪尖机身坐标轨迹最大差均为 **0 m**；每物理步目标与 actor 原目标的差均为 **0 rad**。只读 observer 的测试也验证有无 observer 时状态逐位一致。

200 Hz 关节诊断的关键结果：

| 0.80 m、5–15 s | VD5 Isaac | VD5 MuJoCo | VD6 Isaac | VD6 MuJoCo |
| --- | ---: | ---: | ---: | ---: |
| joint6 运行时速度绝对最大值 rad/s | 17.698 | 2.387 | 7.657 | 3.142 |
| joint6 同步角度增量/5 ms 绝对最大值 rad/s | 3.138 | 2.387 | 1.655 | 3.142 |
| joint6 速度与增量速度差 RMS rad/s | 1.438 | 近 0 | 0.576 | 近 0 |
| joint6 超出名义限速的样本比例 | 5.2% | 0 | 1.2% | 0 |

这是两种离散求解结果之间的实测区别；不能把运行时速度尖峰直接当成同一步持续运动速度，也没有把差分速度替换到 actor 中。Isaac VD5 的 joint6 速度包含明显 >25 Hz 内容，解释了为什么 50 Hz 记录不足以核实其实际高频响应。

MuJoCo VD6 5–15 s，0.70 m 只有 joint6 的 2 个非零限速冲量样本；0.80 m joint4/joint6 分别 3/10 个，其余臂关节为 0。joint1/6 振荡持续存在，故“投影触发次数增加就是全部原因”不成立。两端纯驱动饱和目前没有得到同口径证实。

## 独立执行器读数核验

新增零重力、零摩擦、无接触、真实固定底座、已知惯量的单关节 fixture，不使用机器人策略。固定底座版本用运行时 `is_fixed_base=True` 与真实惯量校验；两个早期开发版本未建立真实固定底座，保留但不用于结论。

PhysX 的投影关节力在无摩擦/限位/限速干预的 fixture 中与 `I*(v_after-v_before)/dt` 相符，正常低幅工况 80 步 RMS 差约 1.8e-7 Nm；限速工况合力矩也与动量变化一致，但包含制动反力。该核验支持它是求解器实测合力，不能把步行时的这个值自动当成独立电机力。

相同惯量 0.05 kg·m²、0.05 rad 阶跃、源 160/5，首个 5 ms 后速度为 PhysX 0.58795、MuJoCo 0.50633 rad/s。当前 5.8 换算与 TGS 多位置迭代的实际离散响应不完全一致。饱和压力工况中，MuJoCo 原生 `actuator_force=120 Nm`，首步动量等效力约 75.95 Nm，进一步说明原生执行器力不能冒充积分后等效力。压力工况只用于接口/数值核验，不代表步行负载。

此外，源 PhysX 的 `friction=0.01` 是与关节传递载荷有关的摩擦系数，MuJoCo 的 `frictionloss=0.01` 是关节库仑摩擦力/力矩阈值；数值相同并不代表摩擦模型等效。本轮没有改动它们。

语义依据：[PhysX articulation force/implicit drive documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Articulations.html#link-incoming-joint-force)、[Omni Physics tensor API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html)。同时核对了本机安装的 API 源文件和 Isaac Lab `ImplicitActuator.compute()`；后者明确只计算 approximate torques。

## 固定底座同目标重放

新增 `replay_vd6_fixed_arm.py`。保留完整机器人惯性、重力、臂源 160/5、原限幅与 MuJoCo 数值换算；固定底座、移除地面，腿保持初始姿态，**不创建 actor、不使用任何接触信号**。只原样重放原会话前 15 秒六个臂关节的 50 Hz 目标。两端同组目标数组 SHA256 相同。

共同统计 5–15 s、200 Hz、3 Hz 高通、各去 1 s 边缘：

| 原目标序列来源（均为 0.80 m） | MuJoCo 固定底座 Y RMS | Isaac 固定底座 Y RMS | MuJoCo / Isaac 三轴向量 RMS |
| --- | ---: | ---: | ---: |
| VD5 MuJoCo 原会话 | 3.869 mm | 3.845 mm | 6.014 / 6.177 mm |
| VD6 MuJoCo 原会话 | 10.576 mm | 10.636 mm | 16.373 / 17.545 mm |
| VD6 Isaac 原会话 | 3.165 mm | 3.451 mm | 4.659 / 5.149 mm |

这把下一步重点指向**步行闭环如何生成不同的 joint1/6 目标序列**。它还不能区分根因究竟是足端冲击/机身 IMU、编码器响应及其相位差，还是它们经过 actor 后的反馈放大。对同一个目标序列，两端仍有数值差异，不能宣称执行器完全一致。

## 文件、验证与下一步

总索引与完整会话路径：[diagnosis.json](../../../outputs/VD6_ACTUATOR_DIAGNOSIS_20261111/diagnosis.json)。包含数据一致性检查、重放输入 SHA256、代码指纹、有效/排除的 fixture 记录、失败会话索引。

- 旧数据关节/事件分析：[existing_joint_analysis.json](../../../outputs/VD6_ACTUATOR_DIAGNOSIS_20261111/existing_joint_analysis.json)，[事件图](../../../outputs/VD6_ACTUATOR_DIAGNOSIS_20261111/existing_joint_touchdown.png)。
- 新 200 Hz 分析：[joint_response_analysis.json](../../../outputs/VD6_ACTUATOR_DIAGNOSIS_20261111/joint_response_analysis.json)，[joint6 与 RL 落脚图](../../../outputs/VD6_ACTUATOR_DIAGNOSIS_20261111/joint_response_touchdown.png)。
- 八个短时诊断目录为 `vd5_fixed70/80`、`vd6_fixed70/80`；固定底座实验在 `replay_vd5`、`replay_vd6`、`replay_vd6_isaac_targets`。
- `effort_fixture_isaac_fixedbase.json` 为有效力读数核验；早期 fixture 和失败/不完整 replay 保留，索引中标为不可用。

本轮实际运行的测试：速度冲量 8 项（含新增只读 observer 不改变状态）、适配合约 11 项、原步态指标 2 项，均通过；新增 Python 文件语法检查通过，两个仓库 `git diff --check` 通过。部署 TorchScript SHA256 仍为 `989895d8077841855b1aa6791aca24e3cf3eb4a7457bb3d902eec19fc6494075`。没有修改训练仓库代码。

下一项诊断应在保持 actor 和源增益的条件下，按实际日志追踪落脚前后 IMU/关节位置/运行时速度到 joint1/6 动作的闭环时序，必要时做清楚标记的单因素传感器/目标重放。不得把差分速度、关节合力矩或 PD 估算伪装成部署反馈/纯驱动力矩。先解决精确受力与约束分解的读取限制，再据证据判断适配修复或新训练；本轮不发布修复结论。
