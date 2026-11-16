# VA15 独立接触／机身状态估计第一版（2026-10-20）

状态：**已实现 C++ 估计器、离线审查、实机静态日志回放及板载只读探针；
尚未接入实机停止控制，行走仍锁定。** 新文件位于独立目录，没有更换
VA15 model_5000、70/18 观测动作、raw action/EMA、增益或现有 A/Y/B 控制。

## 本版实现

```text
真实 lowstate：q / dq / tau_est / IMU / tick / mode / lost
    ├─ 连续性、时效、关节限位、数值、传输检查
    ├─ dq、gyro 的带滤波差分 → 关节／机身角加速度
    ├─ URDF 连杆质量、质心、完整惯量、IMU 偏置位置
    │     → 含机身运动的 Newton–Euler 逆动力学
    │     → 扣除重力、运动惯性 → 各脚世界系反力等效值
    │     → 带模型误差余量、滞回和去抖的承重候选
    └─ IMU 预测 + 候选支撑足的位置／速度／平地高度约束
          → 18 维线性 KF：机身位置、速度、四个脚心位置
          → 高度、速度、协方差和“可审查的停稳候选”
```

不依赖 raw foot_force、SportModeState、策略命令、步态相位或仿真真值。
实机已有采样的 reported ddq 全零，本版明确不使用；不将零值当成真实
关节加速度。IMU 安装偏移从源 URDF 读取，补偿该位置的角加速度／向心项。
初次形成估计需连续约 0.5 s 的安静四腿承重候选；在该条件下估计一次
IMU 静态偏置，不在行走中自动把运动“校成零”。

姿态取自现有 IMU 四元数；KF 不是完整 InEKF，不重新估计姿态。KF 的
绝对水平位置和航向无外部约束，不应视为世界真值。设计参考
[Cheetah 的位置/速度估计器状态结构](https://github.com/mit-biomimetics/Cheetah-Software/blob/master/common/include/Controllers/PositionVelocityEstimator.h)
及[接触辅助惯性估计的可观测性研究](https://arxiv.org/abs/1805.10410)；本目录
是独立实现，并未复制其控制器源码或宣称实现完整论文算法。

### 明确限制

- “承重候选”不是已验证接触传感器。力矩比例、偏置、摩擦、转子惯量、
  非足端接触、外部约束、模型质量误差仍可能造成错误推断。25%/30 N 的
  模型误差余量是审查用启发式，**不是经过实机认证的误差上界**。
- 机身高度依赖平地假设，脚速度约束依赖无滑动／近似点接触；滚动、
  脚垫变形、坡面和共同打滑都可能破坏假设。共同匀速滑动仅靠这些传感器
  不可区分，本版显式报告 `common_mode_slip_observable=false`。
- KF 协方差不包含所有上述模型误差，低协方差不能证明接触或物理安全。
- 实机站姿总力等效值与 URDF 自重约 23% 的差异没有通过自动缩放掩盖。
  “N-equivalent”不能直接作为已标定牛顿值。
- tick 重复不更新时间，跳变／中断／异常输入使估计无效，陈旧输出清成
  NaN，不能把丢失速度当成零。重新初始化会重建局部里程计原点，不能
  不清缓存就直接替换视觉历史轨迹的坐标来源。
- `control_authorized=false`、`hardware_qualified=false` 为编译期边界。
  本版探针没有运动发布器、运动服务客户端、策略或 Z1 控制路径。

## 数学与边界验证

32 组静态 + 64 组动态参考，由源 URDF 独立构造 MuJoCo 浮基模型，覆盖
机身平移/旋转加速度、IMU 杠杆臂、不同逐腿关节运动和逐腿卸载。C++ 与
参考最大力矩差约 1.98e-10、最大反力差约 5.52e-10，仅证明计算一致性。

边界测试覆盖安静初始化、无载/负载符号、反向 IMU、非法关节/力矩、
各腿单独卸载、陈旧/重复/重置/时钟、reported ddq 不参与、偏置修正，
以及无法观察共同滑动、不产生控制权限。本机和 PC4 两组 CTest 均通过。

```bash
python3 export_independent_model.py \
  ../va15_stop_feedback_20261020/board_results/model_sources/board_b2_z1.urdf \
  --output outputs/b2_independent_model.json
python3 generate_dynamic_goldens.py \
  ../va15_stop_feedback_20261020/board_results/model_sources/board_b2_z1.urdf \
  --output outputs/dynamic_goldens.json
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

导出和各回放均拒绝覆盖已有输出，复验时请使用新输出路径或新工作目录。
源 URDF SHA256：`4bccdee2bfec530e0d8e22525ca9d9a5aea8ca7c1e1a7a4eadd979ee9ca2fb9f`。

## 实机静态日志回放

| 日志 | 帧数 | 初始化后可审查估计 | 停稳候选（无控制权限） |
| --- | --- | --- | --- |
| 已确认站稳 | 989 | 935 | 933 |
| 已确认趴卧 1 | 976 | 0 | 0 |
| 已确认趴卧 2 | 981 | 0 | 0 |

站立初始化约 0.54 s；估计高度中位数 0.51343 m，平面速度 p95 0.001154 m/s。
这些是静态记录上的条件估计，没有外部高度/速度测量作真值，不能称为
实机精度验收。候选没有把趴卧时的低关节速度伪装成站立。

## 独立仿真审查：14/14 通过

最终与源码一致报告：`outputs/suite_v3/summary.json`（保留 v1/v2）。
场景使用既有 VA15 actor 和 B 停止候选，显式 B2 PD 200 Hz、物理 1 ms，
Z1 为既有隐式伺服代理，**仿真真值驱动参考 FSM，估计器仅旁路审查**。
没有向估计器提供真值接触、位置、速度；真值只用于比较统计。

覆盖静止、前向指令 0.1/0.2/0.3 m/s、不同按 B 相位、摩擦 0.5/1.0、
30 秒运动及数据冻结/力矩 NaN/反向 IMU 三项故障。正常工况统计的最差值：

| 审查指标 | 最差工况值 |
| --- | --- |
| 条件估计有效率 | >=98.42% |
| 机身高度绝对误差 p95 | <=6.11 mm |
| 平面速度向量误差 p95 | <=0.01624 m/s |
| 悬空脚（真值载荷 <5 N）被标承重候选的比例 | <=0.824% |
| 大载荷接触（真值 >100 N）漏判比例 | <=8.70% |
| 错误的四足停稳候选 | 0 |

故障生效后 0.15 s 起，三项测试各 2070 帧都不再输出有效估计/停稳候选。
这只验证拒绝错误估计，**不证明故障时能自动保持平衡或趴下**。参考 FSM
使用真值，所以这些结果也不是“新估计器参与控制后的闭环停止测试”。
尚未验证实机动态停止、横移/转向、视觉、带枪/软管、坡地、共同滑动。

```bash
cmake -S . -B build -DVA15_BUILD_MUJOCO_REVIEW=ON
cmake --build build -j2
python3 run_review_suite.py --output-dir outputs/new_suite
```

仿真依赖前一独立停止候选已构建的 native core；CMake 显式引用该只读
归档和既有 CPU LibTorch/MuJoCo，不下载依赖，不运行实机启动器。

## 板载只读工具

PC4 独立目录：
`/home/unitree/rl_sar_b2_z1/diagnostics/independent_state_20261020_L7VGqH/`。
板载原生编译，不上传本机二进制。之前诊断源码保存于其
`before_covariance_*/` 目录；生产控制程序、配置、策略保持不动。

在**板载诊断终端**执行（趴卧时也可执行，不会控制起立）：

```bash
cd /home/unitree/rl_sar_b2_z1
./scripts/check_b2_independent_state.sh eth0 10
```

每次创建新的 `outputs/independent_state_check_XXXXXX/`，约 100 Hz 处理
lowstate，每秒打印候选状态，10 秒自动退出。它不发布 B2/Z1 命令，不释放
官方服务，不更改控制配置，输出始终 `authority=0`。只读终端可中断；这
不表示可以在站立时中断另一终端的实际控制程序。

若以后做在线静态复核，应由操作者用已有 A 流程起立并确认站稳，在另一个
终端运行该探针；不按 Y/R1+Y、不推摇杆。实际控制终端仍按已有 B 流程
趴下，确认 Passive 后再退出。**当前没有行走放行命令。**

### 2026-10-20：修复只读探针退出时的堆损坏

用户的 `independent_state_check_0kM15p` 在保存 983 帧后出现
`free(): invalid next size (fast)`；输出文件存在不代表进程正常退出。
本次在 PC4 复现并定位到 **ROS 环境混入不匹配的 DDS 库**：

- 普通交互式终端自动加载 `/opt/ros/humble/lib/x86_64-linux-gnu`。
  旧探针使用 `DT_RUNPATH`，被 `LD_LIBRARY_PATH` 优先覆盖，加载了 ROS
  的 `libddsc.so.0.10.4`，而 C++ DDS 仍来自 SDK（生成头对应 0.10.2）。
- 在该环境运行旧二进制复现同样的 `free()` 错误。Memcheck 还捕获
  `ddsi_sertype_init_flags` 在 72 字节类型对象末尾之外写入 4 字节。
  不是因为趴卧时高度/速度输出 NaN，也不是模型权重出错。
- 最终诊断二进制固定 SDK `DT_RPATH`；包装脚本再将 SDK 库放在其子
  进程搜索路径首位，并检查 C/C++ DDS 实际路径匹配。不修改系统 ROS
  或用户 `.bashrc`，也没有重新编译/替换生产控制器。
- 探针回调改为持有共享收件箱，不引用栈变量，不另起 SDK 队列线程。
  显式 `native.close()`（先 `prevent_callbacks()`）完成后才释放监听器
  所属 SDK 对象和工厂。中间直接回调版本曾被 ASan 抓到释放后访问，
  因而没有将其交付；最终版本已完成退出回归。
- SIGINT/SIGTERM 只请求退出，由主线程收尾，分别返回 130/143；正常
  完成返回 0。脚本明确打印最终退出状态，不以 JSON 文件存在代替成功。

板载证据（均位于本目录 `outputs/`，只订阅 lowstate）：

| 验证 | 结果 |
| --- | --- |
| `valgrind_ros_mismatch_ihcnEd/memcheck.log`，旧版本 + ROS 环境 | 捕获堆越界写 |
| `exit_regression_sanitize_pinned_v2/summary.json`，最终版本 + ROS 环境 | ASan/UBSan 24/24 通过 |
| `exit_regression_release_pinned_v2/summary.json`，最终版本 + ROS 环境 | 普通运行 14/14 通过 |
| `valgrind_fixed_fQdQe4/memcheck.log`，最终版本 + ROS 环境 | 非法访问 0，确定/间接/可能泄漏均 0 |

退出回归覆盖重复启动/退出、完整 10 秒、SIGINT、SIGTERM 和非法网卡。
Memcheck 最终非法访问检查显式关闭了未初始化值告警：配套 DDS 的发现
报文在首次完整检查中有 `sendmsg` 未初始化字节告警，日志原样保留，
没有将它宣称为已修复。仍可达 24 字节也原样报告。

本机/板载两项原有 CTest 通过；估计器头文件、96 组参考数据和生产
控制源码/FSM/配置/VA15 策略的校验值均未改变。旧探针和包装脚本留在
板载 `before_heap_fix_k5PIXo/`，**旧二进制只作为复现证据，不要恢复运行**。

仅在明确要执行**只读联网回归**时使用（不属于自动 CTest）：

```bash
python3 check_probe_exit.py --binary build/independent_probe \
  --model outputs/b2_independent_model.json --interface eth0 \
  --output-dir outputs/new_exit_review --cycles 10
```

趴卧时 `load_DDS=0000`、`initialized=0`、`h/speed=nan` 是未形成四腿
支撑估计的正常拒绝结果；不是通过放宽条件来生成站立或行走许可。

## 下一步边界

下一阶段才是将合格独立来源接入停止状态机，移除对 sport_fresh 的硬依赖，
先以估计反馈而非真值做离线闭环 B 停止/故障验证，再制定实机动态资格流程。
不能现在将旧 `real_foot_contact_calibration_validated` 或
`real_stop_sport_feedback_validated` 手动改 true，也不能假造 sport_fresh。
