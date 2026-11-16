> 历史归档：内容和命令对应原记录日期，不是当前VD13操作指令。当前状态见[文档入口](../../README.md)。

# VD1：B2＋Z1 双相机 MuJoCo 部署

日期：2026-11-02，Asia/Shanghai。本入口只启动本机 MuJoCo 和视觉进程。
旧 VA15、实机程序与日志保留；不启动板载通信或机器人控制。Isaac 中的 VD1
入口与 VC1 也继续保留。

## 当前状态与启动

### 无视觉、固定末端目标、连续平地调试

只看基础步态（不启动相机或实机）：

```bash
cd /home/dong/rl_sar_b2_z1
bash scripts/debug_vd1_flat_walk.sh
```

打开窗口后点击窗口、按空格开始；再次空格暂停，默认60秒。显示实际速度、
倾斜与一米网格；末端目标固定在机身坐标系，不是锁死关节或世界位置。
沿用实验 `impulse` / 5 ms / 隐式位置转换的平地基线，不能作为视觉跟踪通过。
详见 [调试说明](VD1_FLAT_WALK_DEBUG.md)。

### 2026-11-03：0.40 m/s 与平缓曲线预览

MuJoCo 启动器和底层运行器默认改为前进速度指令 0.40 m/s、最大物理裂缝
曲率 0.05/m（stage 1）。裂缝是一条连续的平缓 S 形曲线：起始 2 m 为直线，
之后 20 m 内平滑弯曲并回到直线，最大横向幅值约 0.169 m；不自动增加曲率。

```bash
cd /home/dong/rl_sar_b2_z1
python3 scripts/play_vd1_mujoco.py \
  --speed 0.40 --curvature-stage 1 \
  --velocity-constraint impulse --physics-dt 0.005 \
  --physx-implicit-position \
  --output-root /dev/shm/vd1_mujoco_runs
```

打开 MuJoCo 与前/后相机检测窗口；关闭 MuJoCo 或终端 Ctrl+C 退出。
只改变仿真速度指令与场景曲率，不重训、不更换策略、不改实机配置。视觉
仍使用前 RGB 与后 RGBD，失效时保留原降速/停车逻辑；0.40 m/s 是工作段指令，
不是实际速度保证。本入口不读取 `policy/b2_z1_no_gun/robot_lab/config.yaml`
或 `base.yaml`。以下历史 0.20 m/s 直线验证结果不代表新条件已通过跟踪验收。

本次检查：35 项 MuJoCo/启动器回归测试通过；上述物理选项下，RHANet 双相机
闭环完成 500 个控制步（10 s），无运行错误或倾斜停止，458 步命令有效。会话
`/dev/shm/vd1_mujoco_checks/session_20261103_125424_x8nv7tw0` 保留日志和相机图像。
包含起步阶段的 TCP 投影误差 P95 为 12.26 cm、最大 12.99 cm，仍有后视裂缝
过短导致的短时拒帧；这只是参数生效/仿真可运行检查，未达到 5 cm 跟踪目标。

### 历史 0.20 m/s 直线基线

静态双相机与真实 RHANet 已通过100步检查。道路碰撞几何已修正为有限宽度，
直线段使用原生 box，避免早期巨大凸路面造成的虚假接触。一组实验性执行器
和积分设置下的 MuJoCo 闭环可连续运行30秒并前进约13.2 m，但尚未达到跟踪
验收：全程 TCP 投影误差 P95=7.51 cm、最大13.41 cm，足端边缘净空最小
约-4.15 cm。5秒可视化 GUI 检查确认场景及两路相机感知均可用，但启动阶段
指标不能代表稳态跟踪。下面给出用于复现这组直线基线的命令；启动成功不等于
动态跟踪通过。

```bash
cd /home/dong/rl_sar_b2_z1
python3 scripts/play_vd1_mujoco.py \
  --curvature-stage 0 --speed 0.20 \
  --velocity-constraint impulse --physics-dt 0.005 \
  --physx-implicit-position \
  --output-root /dev/shm/vd1_mujoco_runs
```

默认打开 MuJoCo 窗口及前 RGB、后 RGB、后深度检测窗口，使用 RHANet，前进
速度指令在上述历史命令中为 0.20 m/s。关闭 MuJoCo 窗口或在启动终端按 Ctrl+C 结束本次
会话；入口仅清理自己创建的两个子进程组。

`--curvature-stage 0/1/2/3` 对应物理裂缝最大曲率
`0/0.05/0.10/0.15 1/m`。先验证直线，再逐级增加曲率；道路的几何参数
只属于场景与独立评价器，不能传给 actor 或视觉控制器。

```bash
# 打印命令及来源记录，不启动仿真或视觉进程。
python3 scripts/play_vd1_mujoco.py --dry-run

# 暂停物理积分的静态相机诊断，不执行 actor 动作。
python3 scripts/play_vd1_mujoco.py --static-only --headless --max-steps 150

# 有重力、RHANet 视觉闭环，限定 500 个 50 Hz 控制步。
python3 scripts/play_vd1_mujoco.py --headless --max-steps 500 --curvature-stage 0

# 逐步增大物理裂缝曲率。
python3 scripts/play_vd1_mujoco.py --curvature-stage 1
```

`--no-display` 只关闭感知图像窗口，保留 MuJoCo 窗口；`--headless` 关闭全部
窗口，离屏图像仍参与检测。`--classical` 显式切换为传统图像诊断，不能把其
结果记作 RHANet 通过。`--static-only` 保留模型重力参数，但不执行物理积分，
只检查模型与相机，不代表站立或行走通过。`--max-steps 0` 持续运行。

`--velocity-constraint` 默认仍为 `off`，保留原生 MuJoCo 执行器。上方
命令显式选择的 `impulse` 是实验性速度约束积分，不是 PhysX 精确复现，也
不是已验证的实机部署参数。候选
`implicit_brake` 与 `native_velocity` 是物理层速度约束近似实验，尚未验证
与 PhysX 等效或动态行走可用，不作为默认部署建议。后者使用额外的单边
MuJoCo velocity 执行器提供制动，原18维actor和原motor保持独立。
`native_velocity` 首次动态运行在0.78秒触发停止，尚未解决行走问题。

另一个候选 `impulse` 在积分器内以完整24自由度质量矩阵求解36个单边关节
速度约束冲量，同时包含浮动基座的反作用，属于额外的物理求解步骤。它会
将求得的冲量响应写入 `data.qvel` 并重新计算传感器，不是逐项裁剪18个
关节速度。不能将它描述为“完全不修改物理速度状态”，也不能据此预先声称
与 PhysX 等效或动态部署通过。策略目标、原PD增益与观测仍保持原契约，
不改写测量掩盖结果。选择任一候选时会在会话参数中明确记录。

`--physics-dt 0.001/0.005` 用于对比物理积分时间步，默认0.001秒。两者
均保持50 Hz actor与200 Hz腿部PD；每个PD周期分别积分5或1个子步，不通过
更改策略频率拟合结果。修正位置积分顺序后的 `impulse` 候选跑满30秒，速度
约束残差在 `1e-8` 内且未注入正动能；其 TCP/足端评价仍未达标，不能判为
跟踪通过。

`--physx-implicit-position` 为另一项默认关闭的积分公式转换候选：PhysX的
位置驱动同时隐式处理位置和速度；MuJoCo 3.2.3 的 `implicitfast` 原生处理
速度隐式项。对于无饱和线性驱动，对应转换为
`native_kv = source_kd + physics_dt × source_kp`，5 ms时为5.8。
策略源参数仍是 `kp=160、kd=5`，此选项记录为求解公式转换，不把数值变化
描述为经验调增益。它尚待与原始驱动的实际响应及动态行走比较，默认关闭。
公式依据见 [PhysX 隐式驱动推导](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/_downloads/6acf3afb8f69452757e0e766b5a22978/implicitDrives.pdf)
和 [MuJoCo 3.2.3 数值积分说明](https://mujoco.readthedocs.io/en/3.2.3/computation/index.html)。

已完成的30秒候选运行（新路面、RHANet、0.20 m/s、直线、5 ms、`impulse`、
`--physx-implicit-position`）保存在
`/dev/shm/vd1_mujoco_tests/session_20261102_171714_eeloen3_`。该路径是内存盘，
重启后不保证保留；其中 `summary.json`、压缩 actor/评价/视觉记录和截图可用于
复核。250步 GUI 运行保存在
`/dev/shm/vd1_mujoco_gui/session_20261102_174017_zz5ji02v`，所有步均有有效
视觉命令，确认 MuJoCo 窗口、前 RGB、后 RGBD 和枪尖标记可见；其启动段 TCP
投影误差 P95=3.44 cm、最大6.99 cm，足端边缘净空最小=-1.27 cm，含趴起阶段，
不是稳态验收。仿真和 GUI 均已结束，未启动板载或实机控制。本机文件系统
空间不足，本次输出放在 `/dev/shm`，重启后会丢失；留档前请将
`--output-root` 改到有空间的持久盘。

默认运行环境 `/home/dong/anaconda3/envs/crack_vision/bin/python` 已实际导入
MuJoCo 3.2.3、PyTorch 2.4.1、NumPy 1.24.4 和 OpenCV 4.13.0；仿真与视觉
使用独立进程。入口在启动时再次实际导入检查，不根据模块文件是否存在判断
环境可用。`--mujoco-python` 与 `--vision-python` 可分别指定环境。

## 策略与信息边界

使用 `LiqinLu/b2_z1` 的 `dual_cmd` 分支、提交
`b64b0faf8b4db3a9c1c2e05235521f61ab4dc47d` 的原六轴 Z1 actor：

```text
/home/dong/b2_z1/.pretrained_checkpoints/VD1/exported/policy.pt
SHA256 86be6c2d1c7a0e56fe4bd93c2d62622469c030b0cdc6d10db7af001eae3f889a
```

只加载导出的 TorchScript actor，不加载完整 checkpoint、critic、AMP 判别器
或训练命令生成器。输入为 70 维、输出为 18 维，无观测历史；50 Hz actor、
200 Hz腿部 PD 更新，动作对应 `default_q + 0.25 × raw_action`。MuJoCo 默认
使用1 ms积分子步，每个PD周期保持5个子步；`--physics-dt 0.005` 则每周期
保持1个子步。每个actor周期执行4个PD周期；细分积分不改变训练策略的
50 Hz观测/动作与200 Hz腿部PD更新频率。关节顺序
按训练策略明确映射，不能直接使用 MuJoCo 遍历顺序。

70 维输入为本体目标速度3、TCP目标7、IMU角速度3、IMU投影重力3、关节
相对默认角18、编码器关节速度18、上次原始动作18。IMU与编码器是本体可用
信号；相机标定和固定工具变换属于标定信息。前 RGB 提供裂缝纠偏，后方
对齐 RGB-D 测量地面、裂缝和枪上光学标记。控制核心与图像帧绑定复用
`/home/dong/b2_z1/scripts/vision/{vd1_control,vd1_frames}.py`，视觉复用
同目录 `run_dual_camera_bridge.py`，避免复制出不同控制逻辑。

世界位姿、真实 TCP、裂缝解析路径、真实横向误差、足端接触力不能进入
actor、视觉结果或控制指令。渲染真实可见的物理裂缝与标记合法；渲染真值
轨迹线或仿真 TCP 调试点会造成泄漏，因此不叠加这类标记。独立评价文件可
记录仿真真值，但其数据不能流回控制器。视觉无效或过期时由原控制核心降速
停步，不补真值轨迹，不刷新旧帧时间，不伪造接触。

## 记录与验证

每次会话使用独立的
`outputs/VD1_MUJOCO/session_年月日_时分秒_随机值/`。`session.json`
包含 actor、源文件及URDF引用网格的 SHA256、启动命令、环境版本及结束原因；`play.log` 与
`vision.log` 保存两进程输出，`summary.json` 由仿真进程给出。只有进程退出
成功且 summary 合法，入口才记录正常完成。

相机图像采用8槽环形文件，避免无限增长。未改动的视觉桥将完整原始检测
记录写入本会话私有管道，入口逐字节无损压缩为
`dual_vision_packets.jsonl.gz`；结束后只移除无数据的管道节点，保留压缩
测量。模型转换缓存默认放在 `/dev/shm/vd1_mujoco`，可用 `--cache-dir`
指定。相机采集时间来自真实单调时钟，不用文件修改时间或处理结束时间代替。
本入口还将 CUDA、NVIDIA GL 和 Mesa shader 缓存分别定向到
`/dev/shm/vd1_cuda_cache`、`/dev/shm/vd1_gl_cache` 与
`/dev/shm/vd1_mesa_cache`，实际环境变量保存在 `session.json`；不清理旧缓存
或用户日志。需要把本次全部输出放入内存盘时，可显式指定
`--output-root /dev/shm/vd1_mujoco_tests`。

已完成静态会话 `session_20261102_163125_bb5couk3`：暂停物理积分，100步
双相机感知有效，仿真时间保持0，不能据此评价动态跟踪精度。旧路面失败会话
`session_20261102_163219_jiesbo4f`、候选约束失败会话和道路修正后的运行均
继续保留；失败会话不能与当前结果混为一谈。30秒运行的 RHANet 推理中位数
约148 ms、P95约219 ms；控制器收到的视觉采集年龄中位数约293 ms、P95约
496 ms，最大约797 ms。时延和后段误差累积是优先诊断项，但现有数据不足以
将误差唯一归因于时延；不使用仿真真值补偿延迟。

本次完整回归命令：

```bash
/home/dong/anaconda3/envs/crack_vision/bin/python -m unittest discover \
  -s tests -p 'test_vd1_mujoco*.py' -v
```

结果为29项 MuJoCo runner/契约/候选测试、4项场景碰撞测试，以及36项复用的
控制/帧/几何测试全部通过。它们验证模型、执行器映射、传感器边界、视觉输入
边界及速度候选的局部性质，不证明 MuJoCo 已达到5 cm TCP或足端不踩线目标。

MuJoCo 的动力学、接触和渲染与 Isaac 不同。此前 Isaac 的跟踪误差和不踩线
记录不能作为 MuJoCo 通过的证据；需要以新会话的动态运行、实际相机有效率、
TCP 地面投影误差和足底距裂缝边缘的独立评价判断。当前仅有直线阶段的记录；
曲率应在直线基线通过后按 `0.05、0.10、0.15 1/m` 逐级增加，现无曲线阶段
通过记录。静态相机检查也不能代替动态闭环验收。
