# 旧版压缩包恢复 + VA15 实机部署基线

板载目录：`/home/unitree/rl_sar_b2_z1`。
原压缩包：`/home/unitree/rl_sar_b2_z1.zip`（保留）。
既有目录 `/home/unitree/rl_sar_b2_z1_old` 保留未改。
压缩包解压检查通过，共 26895 个文件、约 5.42 GB。
部分归档文件名编码有误，使用 7z 解压；解压前检查了路径。

## 策略来源与文件

VA15 明确使用：
`2026-09-28_15-07-31_va15_curriculum_from2500/model_5000.pt`。
原检查点 SHA-256：
`91ba123a894a3396b45f70721033a29340d74f8ff82542594625f6f78ff98f79`。

板载文件均相对于上述目录：

- `policy/b2_z1_no_gun/VA15/model_5000.pt`：原始训练检查点。
- `policy/b2_z1_no_gun/VA15/policy.pt`：重新导出的 TorchScript actor。
- `policy/b2_z1_no_gun/VA15/config.yaml`：独立部署配置。
- `policy/b2_z1_no_gun/VA15/manifest.json`：检查点、模型、配置和参考数据哈希。
- `policy/b2_z1_no_gun/VA15/golden.json`：供板载原生推理校验的参考输入输出。
- 原 `robot_lab` 配置和权重保留，不再由当前 B2+Z1 FSM 默认选用。

`base.yaml` 通过 `deployment_policy_profile: VA15` 选择策略。
真实程序在起立前加载同一份配置；FSM 进入策略时也选择 VA15。
起立和策略待机的机械臂目标统一为
`[0,2.18,-1.60,-0.90,0,0]`，避免仅进入策略就更换姿态。
这不是实机工作空间或碰撞验证。

## 参数契约

- actor：70 → 512 → 256 → 128 → 18，ELU，无经验观测归一化。
- 观测顺序：速度指令 3、TCP 目标位姿 7、角速度 3、投影重力 3、
  相对关节位置 18、关节速度 18、上一动作 18。
- 角速度缩放 0.2，关节速度缩放 0.05，其余按保存的训练配置。
- policy 周期 0.02 s，控制配置周期 0.005 s。
- 动作缩放 0.25，原始动作截断 ±5，执行动作 EMA alpha=0.2。
- 上一动作观测使用截断后的原始 actor 输出，EMA 独立保存，
  重启策略时清零。保留旧框架输出侧的关节目标限制。
- 保留训练默认关节位置作为观测偏置，与物理起立姿态区分。
- TCP 默认目标 `[-0.775,0,-0.35,0,-0.707107,0,0.707107]`，四元数 wxyz。
- B2 策略 kp=160、kd=5，Z1 策略 kd 保留旧值 7；策略权重不变。
- 根据本轮起立日志，固定刚度 200/300 时站姿误差约 0.31 rad。
  恢复独立站立参数：B2 hips/thighs kp=450、calves kp=675、kd=15，
  Z1 站立 kp=60、kd=1。完成起立插值后用约 2 秒平滑升至这些值。
  fixed_kp 200/300 和策略增益仍各自保留，不加全局倍数。
- Y 待机和后续站立保持采用同一套站立参数。B 收臂时保留已有支撑命令，
  机身下降时平滑回到 fixed gains，避免刚进入趴下就丢掉支撑力矩。
  这是命令连续性修复，物理站立和趴下效果仍需操作者验证。
- 去除旧配置将小速度请求强制放大到 0.20 m/s 的最低速度限制。
- 保留旧实机程序的速度限制（其中 yaw 硬限制为 0）；这不是全向/转弯验收。

## 编译与检查

新编译目录为 `cmake_build_va15`，避免使用归档中指向 `/home/dong` 的缓存。
修复 CMake 对空 ROS_DISTRO 的一个未引用条件表达式。

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
cmake -S src/rl_sar -B cmake_build_va15 \
  -DUSE_CMAKE=ON -DUSE_MUJOCO=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build cmake_build_va15 --target rl_real_b2_z1_no_gun -j2
./scripts/run_b2_z1_real.sh --offline-check
```

启动脚本已指向新编译目录，并先验证 VA15 权重和配置。
已通过：板载编译、依赖加载、256 组 Python 导出一致性（误差 0）、
8 组原生 C++ actor 对照、70 维观测组装、原始上一动作/EMA/reset 单元测试。
原生与参考最大绝对差约 `1.14441e-5`；跨 PyTorch/LibTorch 的 float32
比较使用 atol=1e-5、rtol=2e-6，未截断比较中的 actor 输出。

已执行一次 B2-only 只读预检，成功接收状态，各电机本次快照 lost=0。
未接管官方控制、未启动 Z1 控制器、未发送 B2 运动命令。

## 从头开始操作

第一步，在板载终端只检查 B2 通信：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
./scripts/run_b2_z1_real.sh eth0 --preflight --disable-z1
```

## 后续：独立 B 停止／趴下候选（已同步，仅静态实测）

操作者再次确认 B2 安全趴下、控制程序退出后，已备份并同步独立 B 控制器，
在 PC4 重新编译，通过原生离线检查、板载内存状态机测试与 B2-only 只读预检。
备份：`deployment_backups/controlled_getdown_20261020_HMsouK/before.tar.gz`。
VA15 权重及 70/18、缩放、EMA、站立／策略增益均未更改。

候选将 B 的减速、四足窗口捕获、独立固定站立、静止确认与收臂下降分开。
本机 13 组回归通过；无枪无管 24 组动力学测试完成趴下，5 组无效／陈旧
反馈测试拒绝下降。1 kg TCP 代理负载只有 6/8 完成，不能用于带载验收。

两个真实来源 validated 仍为 false，实机行走继续锁定。现在仅准备操作者
复验无枪／无管、零速度 A→B 或 A→Y→B，不是行走后 B 的实机放行。
详情见 `patches/va15_controlled_getdown_20261020/README.md`，板载对应说明为
`docs/VA15_CONTROLLED_GETDOWN_20261020.md`。本轮没有发送任何运动命令。

Z1 联合预检和实机运行现在只需要一个板载终端。脚本自动后台启动
`/home/unitree/z1_controller/build/z1_ctrl`，或复用已经运行的同名进程，
并检查该进程的 UDP 8071 socket。存在多个控制器或端口未就绪会阻止继续。
离线检查、`--disable-z1` 和未确认 RUN 的取消路径不会启动 Z1。

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
./scripts/run_b2_z1_real.sh eth0 --preflight
```

自动启动的控制器日志位于 `outputs/z1_controller/`。主程序退出后保留控制器
供后续复用，不自动结束已有控制器。若此前控制器还在手动打开的旧终端中，
请在 B2/Z1 已安全趴卧且主程序退出后，在旧 Z1 终端 Ctrl+C 一次；之后均从
这个单终端入口启动。不要在低层控制运行中关闭控制器窗口。

SDK 默认连接本机 `127.0.0.1:8071`，不要把机械臂下位机
`192.168.123.110` 用作 `--z1-ip`。
联合预检会发送 Z1 PASSIVE 模式报文，应确保机械臂已安全放置。

实际接管由操作者按阶段进行，命令为：

```bash
./scripts/run_b2_z1_real.sh eth0 --run
```

该命令需要明确输入大写 RUN，随后可能驱动 B2/Z1。此轮未运行该命令，
未进行站立、停止、趴下或行走验收。首轮从无视觉、零速度站立验证开始；
现场防护、独立急停和正常退出能力应先验证。

2026-10-20 已将实机遥控器恢复为熟悉的映射：

| 功能 | 键盘 | 遥控器 |
| --- | --- | --- |
| 起立 | 0 | A |
| 策略待机 | 1 | Y（保留 R1 + 方向键上作为兼容键） |
| 切换视觉轨迹 | T | R1 + Y |
| 速度归零 | Space | X |
| 趴下请求 | 9 | B |
| 被动/紧急处置请求 | P | L1 + X |

组合键优先，按住 R1+Y 不会反复切换，松开 R1 不会误触发 Y。
X 由原来的导航开关改为速度归零；摇杆回中后才允许重新给速。
B 发出趴下请求后锁住手动速度，直到再次按 A。
L1+X 请求紧急被动并锁定，需重新启动程序才能解除。
遥控输入只在控制线程消费，推理/预检读取状态不会触发按键。
当前默认不启动视觉；R1+Y 需要启动时启用视觉并满足轨迹有效性条件。
视觉进入条件检查实际策略 FSM、准备计时和退出/故障状态，不再要求待机时
actor 已经开始推理；仍需通过新鲜轨迹和里程计检查。
趴下后确认 Passive 再退出；此前失败过的运动后停止不能视为已经修复。

## 备份与范围

`deployment_backups/va15_legacy_setup_20261020/before.tar.gz` 保存了归档恢复后、
修改前的 base/实机源码/FSM/启动脚本；`cmake_before.tar.gz` 保存原 CMake 文件。
同目录保留 configure.log、build.log、offline_check.log、preflight_b2_only.log。

本基线没有搬入前一天的接触门限、shadow 支撑诊断或停止控制器实验。
恢复旧程序不等于修复足端数据的含义，也不等于 VA15 已通过实机行走验收。

键位更新前备份：`deployment_backups/familiar_remote_20261020_hRmNc4/before.tar.gz`。
按键回归测试覆盖组合键优先级、长按/释放、启动时按住按键、X 后摇杆回中、
B 后速度锁定和紧急锁定。键位变更不修改 VA15 权重、配置、增益或停止控制器。
板载重新编译、按键逻辑测试和 VA15 原生推理离线检查已通过。
本次未启动控制程序，未通过实机按键驱动任何动作。

## 站立参数与单终端启动修复记录

备份目录：`deployment_backups/stand_and_single_terminal_20261020_inIBet`。
`before.tar.gz` 包含修改前程序、配置、manifest 和可执行文件。
VA15 policy.pt SHA-256 仍为
`8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098`。

新增测试覆盖站立增益单调有界变化、Y 待机增益保持、收臂保留目标/增益、
趴下第一帧命令连续、下降阶段回到固定增益，以及 B 取消排队的 Y 请求并在
可切换阶段优先进入 GetDown。原生测试只操作内存中的 FSM
和命令缓冲，不模拟真实载荷，不发送机器人命令。
单终端启动的 13 个隔离测试使用假控制器进程和假 socket 输出，覆盖新启动、
复用、多进程、错误端口归属、禁用、取消 RUN 等路径；没有驱动实机。
板载现有控制器 PID 16848 的复用与 UDP 8071 归属检查也已通过；这一步
只检查进程和 socket，没有连接 SDK 或向机器人发命令。

下次只做 A 起立、零速度观察、B 趴下；暂不按 Y、R1+Y 或推动摇杆。
若仍不完全站起，请保留新增的 joint/actual/target/kp/tau_est 日志。

## 行走后 B 无法趴下：修复及独立诊断（已同步，动态未验收）

随后操作者报告 A/Y 待机与 B 趴下正常；无视觉前进能迈步，但运动后 B
卡住。用户日志证实 base_h=0、feet=0000。实机 GetState 没有填入停止
流程依赖的仿真诊断字段，因此既有停止门限永远无法满足。
操作者已确认安全趴下并退出后，本机准备了独立的反馈有效性与 B 锁存
修复候选，保存在 `patches/va15_stop_feedback_20261020/`，详见其中 README。

本轮没有 B2 有线直连，SSH 未连通，故尚未同步、编译板载或做实机验收。
候选在当前足端标定及低层 sport 高度/速度未验证前拒绝启动行走；未强设
接触或调低门限。策略权重、70/18 契约、行走增益及站立增益未变。
不要在旧板载程序上继续用行走反复尝试 X/B，需先恢复连线并完成只读采样。

随后操作者恢复有线连接：核对板载旧版本哈希且确认无实机控制进程后，
已备份至 `deployment_backups/stop_feedback_20261020_ixstjy4z/before.tar.gz`，
选择性同步修复源码/配置并重新编译；VA15 原生离线检查通过。
只读趴卧采样 979 条，原始足端中位数 DDS FR/FL/RR/RL 为
486/492/537/514；sport 时间戳始终为 0，不能据此验证反馈更新。
随后操作者完成站立采样并安全趴下退出：站立 987 条，后续趴卧 975 条。
足端 FL 信号在静止时出现与关节/力矩无相应变化的双区间，并与趴卧重叠；
低层站立时两个 sport 状态通道均没有报文。未生成足端标定，两个 validated
仍为 false，当前仍禁止新行走。详细数据保存在补丁的 `board_results/`。

已进一步同步基于实际板载 URDF 的独立关节运动学/IMU 影子诊断：不改变
70 维观测、策略输出或停止反馈，也不提供控制许可。站立回放高度假设约
0.5143 m，趴卧约 0.1077/0.1091 m；这些不是外部测量或接触证明。
日志输出 `[Kinematics shadow] ... stance_evidence=UNKNOWN authority=0`。
平地和无滑动假设仍需独立接触证据验证，不可用它强制放行行走/趴下。

新模块首轮只读预检退出时触发内存错误，已修复旧实机主程序中的订阅回调
对象销毁次序与 detach 线程退出问题。板载重编译、VA15 原生校验、20 次
只读预检和 10 次初始化失败退出通过，本机 10 组离线回归通过。
本轮没有释放官方运动服务、启动 Z1 或发布运动命令，未验证动态停止。
备份及验证日志：`deployment_backups/kinematics_shadow_20261020_s72aspih/`。

当前可在板载终端进行下列离线/只读复查，不要启动 `--run` 行走：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
./scripts/run_b2_z1_real.sh --offline-check
./scripts/run_b2_z1_real.sh eth0 --preflight --disable-z1
```
