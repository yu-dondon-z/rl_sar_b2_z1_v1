# VD1 源 PhysX / MuJoCo 质量、惯性与接触记录核对

日期：2026-11-03（Asia/Shanghai）。本次只读加载现有 URDF、USD、manifest 和 MuJoCo 模型，没有运行新仿真、修改生产模型或覆盖旧日志。

## 结论

1. 两份源 URDF 完全相同：SHA256 `14ae3abccfa4d7e5f74f69347ced43f7b1ec5bed728376972260d8b18130c9c6`。URDF 显式质量之和是 **82.25806718235386 kg**，不是 83.258 kg。
2. `dummy_target` 并非缺少 inertial，而是显式 **质量 0、惯性全 0、无几何碰撞体**；固定关节另设 `dont_collapse="true"`。生成 USD 仍显式写入质量 0、惯性 0。源 PhysX 运行时将它实际处理成 **1 kg**、`diag(0.00400000019)` kg·m²、局部 CoM `(0,0,0)`。警告记录显示 PhysX 对无效质量/惯性使用了小球近似。因此这里是运行时回退，不是 URDF 显式多出 1 kg。
3. MuJoCo 转换器只为 `mass > 0` 的 link 输出 inertial，因此该无几何 dummy 保留为质量 0 的固定坐标体。MuJoCo 总质量 **82.25806718235386 kg**，源 PhysX manifest 总质量 **83.25806194543839 kg**；差 **0.99999476308453 kg**，其中约 5.24e-6 kg 是其余质量的浮点差。
4. 消除固定关节折叠表达差异后，其余实体的质量、CoM、完整惯性一致到浮点精度。不能直接拿 PhysX 的合并 `base_link` 与 MuJoCo 的单独 `base_link` 比，也不能对 PhysX manifest 已完整表达的 3×3 惯性张量再乘一次 principalAxes 旋转。
5. **Isaac 诊断没有采集接触。所有旧 Isaac summary 的碰撞结论均为未知。** `first_nonfoot_ground_contact_time_s=null`、`first_gun_ground_contact_time_s=null` 和 `nonfoot_ground_contact_max_normal_force_N=0.0` 是借用 MuJoCo 汇总器后对缺失字段的默认输出，不是“测得没有碰撞”。旧文件保留不动；未来 Isaac 输出已经移除这三个误导性指标，新增 `contact_measurements_collected=false` 和明确的 unknown 说明。

## 证据位置

- 源 URDF：`/home/dong/b2_z1_VD2/source/b2_z1/b2_z1/assets/b2_z1/b2_z1_gun.urdf:1196`，质量/惯性在 1199/1200；固定保留关节在 1204，枪端偏置在 1211。
- MuJoCo 使用的相同 URDF：`/home/dong/b2_z1/source/b2_z1/b2_z1/assets/b2_z1/b2_z1_vd1.urdf`。
- 转换器正质量条件：`scripts/vd1_mujoco_scene.py:348`；349 为 `mass > 0`；429–450 的模型验证只验证 URDF 总质量和正质量 link，不验证 PhysX 运行时回退。
- 已生成 USD：`/dev/shm/vd1_isaac_response/run_20261103_133808_ndbnd0c9/usd_cache/original_b2_z1_gun.usd`，只读 USD API 查询 `/b2_description/dummy_target` 得到 authored mass=0、diagonalInertia=(0,0,0)、centerOfMass=(0,0,0)、principalAxes=identity。
- 实际 PhysX manifest：`/dev/shm/vd1_isaac_response/run_20261103_133808_ndbnd0c9/manifest.json:177` 为 body 顺序，dummy 为下标 25；232 为实际质量 1，512–521 为实际惯性，525/659 开始记录 CoM/姿态。
- PhysX 回退警告：`/home/dong/anaconda3/envs/env_isaaclab/lib/python3.11/site-packages/omni/logs/Kit/Isaac-Sim/5.0/kit_20261103_133801.log:4623`，4665 再次出现。
- 旧 Isaac 结果的误导性碰撞字段：`/dev/shm/vd1_isaac_response/run_20261103_133808_ndbnd0c9/straight/summary.json:82`，随后 83/84。它们应解读为 **未采集**，不能作为无枪碰地/无非足端接触的证据。
- Isaac 只记录姿态、速度、关节/力矩估计和 gyro crosscheck：`scripts/diagnose_vd1_isaac_response.py:79`；248 仅创建 IMU，没有 contact sensor。
- 默认字段来源：`scripts/diagnose_vd1_base_response.py:392`，使用 `r.get(...)`，396 的空列表 force 默认 0。
- 已修正未来输出语义：`scripts/diagnose_vd1_isaac_response.py:109`，115–119 显式标记未采集并移除三个接触指标；纯单元检查通过，无需重新运行仿真。

## 只读逐刚体合并比较

比较模型：`/dev/shm/vd1_base_response/response_20261103_133048_58ynhbss/straight/scene_cache/vd1_flat_ground_diagnostic.xml`。

按 PhysX 保留刚体名称，将 MuJoCo 的固定子体质量、CoM、惯性以平行轴定理合并到相同刚体。没有改变或步进任何模型。

| 合并后刚体 | MuJoCo 合并质量 kg | PhysX 实际质量 kg | 说明 |
| --- | ---: | ---: | --- |
| base_link | 41.4371000000 | 41.4370956421 | base 35.86 + head 3.6885 + tail 0.795 + 4×hip_rotor 0.2734 |
| FL_hip | 2.9464000000 | 2.9464001656 | hip 2.673 + thigh_rotor 0.2734 |
| FL_thigh | 4.8094000000 | 4.8093996048 | thigh 4.536 + calf_rotor 0.2734 |
| link_gun | 3.2590469824 | 3.2590470314 | 枪本身一致 |
| dummy_target | 0 | 1 | 唯一明显的不等效质量项 |

排除 dummy 后，全部 25 个保留刚体的最大绝对差：

- 质量：4.358e-6 kg。
- CoM 各分量：8.152e-9 m。
- 合并后完整 3×3 惯性张量各分量：6.918e-8 kg·m²。

dummy 的 MuJoCo 质量为零，所谓质量中心位置没有物理含义；不能将编译器为质量零体填充的 `body_ipos` 与实际 1 kg 刚体的 CoM 做有意义的误差比较。

## 局限与下一步含义

这证明目前两个运行时不严格动力学等效；即使 MuJoCo 验证“等于 URDF”通过，也没有复现源 PhysX 的 1 kg 回退。1 kg 位置在枪端固定框架，可能改变机械臂负载与机身耦合，但没有进行受控质量 A/B，因此不能据此宣布它就是摔倒的唯一原因，也不应直接修改生产模型来掩盖差异。

这次复现的是当前本地源配置和当前 PhysX 版本；尚无原策略训练时的实际 mass/inertia dump，不能仅据此断言训练历史必然采用相同回退。源 PhysX 的 60 秒姿态稳定性和零速度仍前进结果仍是有效观测；任何“没有非足端碰地/枪未碰地”的说法均需撤回为未知，直到真正采集接触。
