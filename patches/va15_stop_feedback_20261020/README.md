# VA15 行走后 B 无法趴下：离线修复候选

状态：已同步 PC4 并编译，通过 VA15 原生离线校验、本机 10 组离线回归；
新增的运动学影子诊断也已同步。修复退出生命周期后，板载连续 20 次只读
预检、10 次初始化失败退出测试通过。仍未通过实机动态停止/趴下验收，
不能放行行走。
板载更新前备份：`deployment_backups/stop_feedback_20261020_ixstjy4z/before.tar.gz`。
本轮没有启动低层实机控制或 Z1，也没有发送运动命令。

## 根因与边界

用户日志中 B 多次收到，但 `base_h=0, feet=[0000]` 一直不变。
归档恢复版本的实机 GetState 只填 IMU、关节，停止判断却使用初始化后的
仿真诊断字段。`lin_vel=0` 同样不是有效的实机零速证据。
因此 moving actor 一直等待无法满足的四足停止条件；零速 Y 待机没有运行
actor，所以之前 A/Y/B 静态测试没有触发该缺陷。

另一个缺陷：停止成功时从 Run 请求 GetUp，而 B 在随后的 CheckChange
才锁存，可能丢失同一帧收到的趴下请求。

## 改动

1. 实机状态中单独保存停止反馈及有效位，不追加到 VA15 的 70 维观测。
   读取 DDS foot_force 原始桥路值与 tick，读取 sport 的 body_height、velocity、
   stamp、error_code。高度不使用 odometry position.z 代替。
2. 原始 foot_force 不是牛顿，也不是接触布尔值。只有带 ID 的、经过当前
   实机验证的逐足零点、方向、开关阈值才参与中值、滞回和去抖判定。
   DDS 顺序 FR/FL/RR/RL 映射为 FL/FR/RL/RR。
   没有复制旧部署中有争议的标定或用绝对值消除负信号。
3. 低状态重复 tick、重复 sport 时间戳不能刷新数据年龄。
   缺失、过期、非有限值、重置或未验证来源都不视为已停稳。
   接触未知显示 ????，高度/速度未知显示 nan；非足触地不可直接测量时
   明确显示 unknown，不把默认 false 当作没有碰撞的证明。
4. 实机在尚无停止依据时拒绝启动行走。有效四足静止支持需连续 0.5 秒；
   被拒绝的持续摇杆命令不会在数据恢复后自行启动，需先归零再重新请求。
   A 起立、Y 静态待机、B 从未行走的待机趴下仍可使用。
5. B 在 Run 前锁存并持续归零速度。实机确认停止后结束 actor，按支撑力矩
   连续转换至既有站立参数，完成站立过渡后收臂、趴下；不绕道丢失 B 的
   GetUp。A/Y 不允许从运动中直接绕过停止判断。
   X 仅停止，不自动产生趴下请求。紧急被动请求仍优先。
6. 新增仅订阅、不发布命令的 sample_b2_stop_feedback.py，供后续采集。

策略权重、观测项和顺序、归一化、raw-action/EMA、动作缩放、关节路由、
行走增益、站立增益、最小停止高度 0.48 m 和姿态门限 0.15 均未更改。

## 只读采样结论与剩余事项

2026-10-20 趴卧只读采样：979 条，DDS FR/FL/RR/RL 原始值中位数
486/492/537/514，腿关节位置稳定。sport 的时间戳始终为 0；其原始高度
约 0.0801 m，速度约 4.8e-10 m/s。零时间戳不能用来证明来源持续更新，
也不能由此断言所有 sport 报文都冻结。未验证该来源或自动生成足端标定。

随后操作者按既有流程站稳、保持零速度，采集站立数据 987 条，并再次
趴卧采集 975 条。关节反馈稳定，但 FL 原始足端信号出现约 465 和 502
两个区间；区间内关节位置、力矩估计和 IMU 没有相应变化，且与趴卧信号
重叠。不能仅靠中值滤波、换符号或降低阈值完成可靠标定，也不能据此断定
传感器损坏。详见 `board_results/standing_assessment_20261020.md`。

低层站立时另行同时监听 `rt/sportmodestate` 和 `rt/lf/sportmodestate`，
未收到两者报文；lowstate 仍约 500 Hz。不能继续依赖这两个来源完成停止
高度/速度判断。仍需核查足端数据来源及物理含义，并验证独立状态估计。

配置中以下两项保持 false，当前候选明确禁止手动或视觉行走：

```yaml
real_foot_contact_calibration_validated: false
real_stop_sport_feedback_validated: false
```

这不是让用户直接改 true 的开关。需要先保存趴卧与操作者确认四足支撑的
站立数据，确认各足信号可解释且无矛盾，再确定标定；还要确认官方运动
服务释放以后 sport 高度、速度仍持续有效。若低层接管后不提供这些字段，
需实现并验证独立的运动学/接触状态估计，不能使用默认零值或陈旧数据。

候选也不承诺传感器在行走中失效后能自动安全趴下：缺失支撑依据时仍拒绝
强制下降，保留 B 并告警，需要现场安全处置。超时不是有效支撑证明。
本次离线验证不能证明动态停止、防摔、足端标定或软管负载实机性能。

## 独立运动学影子诊断（已同步，禁止控制授权）

从本次实际板载 URDF 提取四腿几何，保留原文件 SHA-256，不使用猜测腿长。
输入是原生 DDS 顺序 FR/FL/RR/RL 的关节位置、速度与 IMU 四元数/角速度。
通过正运动学和雅可比计算每只脚的几何位置，并输出下列条件假设：

- 平地接触假设：`h_i = r_foot - (R_WB * p_foot_B).z`。
- 脚端中心不动假设：`v_base_W_i = -R_WB * (J_i * dq_i + omega_B × p_foot_B)`。
- 输出逐足一致性、tick 连续性和数据新鲜度；没有有效接触证据时明确显示
  `stance_evidence=UNKNOWN authority=0`。

它不是接触检测器或完整惯性导航：悬空但同关节姿态也可能产生相同几何；
脚垫变形、脚端滚动、坡度、打滑都会破坏上述假设。`control_authorized`
在 C++ 中固定为 false；结果只存放在诊断字段，不填入 actor 观测或真实
停止反馈，也不替代四足接触、零速或允许下降的证据。

已有数据回放：站立高度假设中位数 0.514324 m，速度假设 p95 0.005245 m/s；
两次趴卧高度假设中位数约 0.1077/0.1091 m。这些是模型假设下的估计，
不是外部实测高度或实机验收。64 组 URDF 独立 MuJoCo 几何/雅可比参考及
无效输入、陈旧数据、滑动/悬空歧义测试通过。

```yaml
real_kinematics_shadow_only: true
real_kinematics_model_file: b2_leg_kinematics.json
```

新增诊断首轮板载预检退出出现堆内存错误。检查发现旧主程序的 DDS 回调
依赖对象先于订阅读取器销毁，且旧框架 LoopFunc 使用 detach，shutdown
不实际等待线程结束。实机程序已改用可等待结束的独立循环类，在数据仍存活
时关闭并等待 DDS 读取器；订阅成员最后声明，覆盖构造异常的析构次序。
未修改通用框架循环或 Z1 SDK 循环；诊断退出不准备低层被动命令。
修复后 20 次 B2-only 只读预检及 10 次构造失败退出均正常，不代表低层控制
循环或物理停止已通过实机验收。

新增备份：`deployment_backups/kinematics_shadow_20261020_s72aspih/`。
其中 `before.tar.gz` 是影子模块更新前备份，`preflight_b2_only.log` 保留首次
失败，`first_shadow_before_lifetime_fix.bin` 保留出现问题的二进制供追查。
`build_lifetime_fix.log`、`preflight_lifetime_fix_01.log`、`repeat_preflight_*.log`
与 `repeat_unwind_*.log` 记录修复后的验证。

## 离线复验

`work/` 是针对板载归档版本的独立源码快照，不会覆盖本机新克隆的旧基线。
默认只读使用本机现有框架头文件和 LibTorch 2.3.0；路径可通过 CMake
的 VA15_BASELINE_ROOT、VA15_TORCH_ROOT 显式设置。

```bash
cd /home/dong/rl_sar_b2_z1/patches/va15_stop_feedback_20261020/work
cmake -S offline -B offline_build -DCMAKE_BUILD_TYPE=Release
cmake --build offline_build -j2
ctest --test-dir offline_build --output-on-failure
```

10 组测试：停止状态机、原生 VA15 对照、运动学与授权边界、
反馈有效性/映射/陈旧数据、raw action/EMA、遥控键位、站立增益、
实机循环等待与生命周期、13 项模拟启动器测试、只读采样器。
状态机测试只操作命令缓冲并使用明确标记的合成反馈，不是动力学仿真。

## 本轮板载同步位置

本轮确认操作者已安全趴下、实机控制进程不存在，并核验旧文件哈希后，
备份源文件、配置和 cmake_build_va15/bin/rl_real_b2_z1_no_gun，再按下表
选择性同步，在板载重新编译。未覆盖整个目录，未传输本机编译的二进制。

| work 内文件 | 板载仓库内位置 |
| --- | --- |
| rl_real_b2_z1_no_gun.cpp | src/rl_sar/src/rl_real_b2_z1_no_gun.cpp |
| rl_sdk.hpp | src/rl_sar/library/core/rl_sdk/rl_sdk.hpp |
| va15_stop_feedback.hpp | src/rl_sar/library/core/rl_sdk/va15_stop_feedback.hpp |
| va15_kinematics_shadow.hpp、va15_kinematics_model_io.hpp | src/rl_sar/library/core/rl_sdk/ |
| va15_joinable_loop.hpp | src/rl_sar/library/core/rl_sdk/ |
| fsm_b2_z1_no_gun.hpp | src/rl_sar/fsm_robot/fsm_b2_z1_no_gun.hpp |
| b2_z1_no_gun/VA15/config.yaml、manifest.json | policy/b2_z1_no_gun/VA15/ |
| check_va15_legacy.py、sample_b2_stop_feedback.py | scripts/ |
| b2_z1_no_gun/VA15/b2_leg_kinematics.json、kinematics_golden.json | policy/b2_z1_no_gun/VA15/ |
| test_va15_kinematics_shadow.cpp、replay_va15_kinematics_shadow.cpp、test_va15_joinable_loop.cpp | scripts/ |

板载只读采样器同步以后，可在保持趴卧、不启动自定义控制的情况下执行：

```bash
cd /home/unitree/rl_sar_b2_z1
python3 scripts/sample_b2_stop_feedback.py eth0 --seconds 10 --posture prone \
  --output outputs/real_tests/stop_feedback_prone_01
```

它不会起立、释放官方控制、发布 lowcmd 或连接 Z1。之后站姿采样需操作者
按已验证流程操作，不能由采样器自动切换姿态。采样器只保存原始数据，
不会生成标定或赋予行走许可。
