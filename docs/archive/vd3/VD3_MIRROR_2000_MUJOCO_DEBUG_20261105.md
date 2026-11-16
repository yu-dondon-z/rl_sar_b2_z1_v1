# VD3 镜像 2000：MuJoCo 部署与 VS Code 调试

当前本地 MuJoCo 默认策略已改为镜像微调 `model_2000`。选型依据见
[完整评估](../../../outputs/VD3_MIRROR_REVIEW_20261105/final_review.md)。原 10500 和训练日志保留。

部署策略：`policy/b2_z1_no_gun/mujoco_vd3_mirror_2000/policy.pt`，
SHA256：`51eda90b2b77204ef66d949523486250e88c7062aeb1e86f0dc5478924291c78`。
同目录的 `provenance.json` 记录原始检查点、哈希与评估来源；启动器和仿真子进程
共用 `scripts/vd3_mujoco_policy.py` 校验策略。

## 直接看效果

```bash
cd /home/dong/rl_sar_b2_z1
bash scripts/play_vd3_mujoco.sh
```

显示 MuJoCo 跟随视角和双相机感知叠加图。默认速度指令 0.20 m/s，
平地黑线最大曲率 0.40/m（第 4 档），最急弯处半径约 2.50 m；50 Hz 策略、200 Hz 物理，沿用通过评估的
implicit-position 适配和 joint-velocity impulse 约束。
关闭 MuJoCo 主窗口或在启动终端按 Ctrl-C 结束本次仿真并保存日志。

终端启动、诊断脚本默认值和两个 VS Code VD3 调试入口均已改成第 4 档。
参数在启动时生成实际场景；修改后需重新启动会话。恢复先前 0.05/m 对照可传
`--curvature-stage 1`；0.30/m 对照可传 `--curvature-stage 5`。
曲率沿 S 曲线变化，0.40/m 是最大绝对曲率，不是全程常数。

## 一次 60 秒可视诊断

```bash
cd /home/dong/rl_sar_b2_z1
bash scripts/debug_vd3_rear_vision.sh --show
```

3000 个 actor 步为 60 秒仿真时间；需要持续看效果时：

```bash
bash scripts/debug_vd3_rear_vision.sh --show --max-steps 0
```

无窗口的 60 秒诊断：

```bash
bash scripts/debug_vd3_rear_vision.sh
```

调试结果保存在终端打印的 `outputs/VD3_REAR_VISION_DEBUG/session_*/`。
每次建立新目录。结束时打印并写入 `rear_debug_summary.json`，包括：

- 策略路径和 SHA256、实际结束原因、跟踪／保持／停车步数。
- 双相机有效包、后视拒帧原因、采集时效、后视中心线间隙和工作点覆盖。
- `steady_evaluation`：排除前 5 秒后的实际前进速度、机身／TCP 世界高度
  P95−P5、TCP 地面投影误差 P95／最大值／超过 50 mm 的比例。
- `foot_height_width_mm` 和 `foot_vertical_force_mean_n`：FL／FR／RL／RR 的
  高度幅度和平均垂向支撑力，便于继续观察左右步态差异。这些不是对称性合格判定。

少于 5 秒时，稳态统计标记为不可用。完整 `summary.json` 仍保留包含启动阶段的误差，
不能把排除启动后的精度当作全程精度。仿真异常会返回非零退出码。

`play.log`、`vision.log`、`actor_trace.jsonl.gz`、`evaluation_truth.jsonl.gz`、
`dual_vision_packets.jsonl.gz`、`latest_dual_overlay.png`、`front/`、`rear/`
和 `viewport_robot.png` 均随会话保存。

## VS Code：选入口后按 F5

已在 `/home/dong/b2_z1/.vscode/launch.json` 和
`/home/dong/rl_sar_b2_z1/.vscode/launch.json` 加入相同入口，保留已有配置：

- **VD3：镜像2000＋双相机 MuJoCo（可视调试）**：显示窗口，持续到关闭窗口／Ctrl-C。
- **VD3：镜像2000＋双相机 MuJoCo（60秒诊断）**：无窗口运行 3000 步后自动汇总。

在“运行和调试”下拉框选择名称，再按 F5。每个入口已固定
`crack_vision` Python、部署目录和双相机参数，并启用 `subProcess: true`。
启动器、仿真和感知子进程可以分别在调用栈面板中选择；原 IsaacLab 解释器设置保留。

建议在以下实际执行文件设置断点或日志断点：

| 文件 | 观察位置和变量 |
|---|---|
| `scripts/run_vd1_mujoco.py` | `controller.update(...)` 后看 `command`、`packet`、`rejection` |
| `scripts/run_vd1_mujoco.py` | `build_actor_observation(...)` 后看 `observation`（70 维）与 `action`（18 维） |
| `scripts/vd1_visual_guard.py` | `GuardedVD1VisualController.update`，看跟踪／后视保持／停车切换 |
| `scripts/run_vd1_dual_camera_bridge.py` | `process_pair`，看前后相机的实际感知结果 |
| `/home/dong/b2_z1/scripts/vision/dual_camera_perception.py` | `process_front`、`process_rear`，看分割和中心线 |

暂停子进程会影响真实帧时效，恢复时可能暂时保持或停车；连续观察步态可使用日志断点。
建议通过关闭 MuJoCo 窗口或终端 Ctrl-C 正常结束，让会话完成汇总。

## 感知与模型边界

外部感知由前 RGB 和后 RGBD 图像生成，actor 使用 70 维命令／本体观测，
不加载 critic。世界位姿、地面投影误差、足端真值只用于结束后的独立评估。
当前为 `rear_seam_direct`：后相机测焊缝并给出末端目标，
`visual_tcp_feedback=false`，尚不是后相机检测枪尖的视觉 TCP 闭环。

模型沿用 B2＋六轴 Z1、零附加质量焊枪与原有碰撞。没有调整训练／部署增益，
也没有改动 `robot_lab/policy.pt`。速度指令是 0.20 m/s，之前 60 秒验证的
实际稳态前进速度约 0.219 m/s。左右抬脚幅度差仍存在。

本次镜像 `model_2000` 对应的训练参数快照中，`commands.base_velocity.forward_speed=0.2`，
实际命令生成器对行走环境输出 `[0.2, 0, yaw_rate]`；每 3–8 秒重采样时，约 20%
环境抽到站立并输出零前进速度。YAML 中的 `lin_vel_x: [0.0, 0.2]` 是范围配置，
不能将其误解为本命令生成器随机采样行走速度。训练指令也不代表实测速度。
依据是训练运行
`/home/dong/b2_z1_VD2/logs/rsl_rl/b2_z1_vd3_curve_amp/2026-11-05_09-57-36_vd3_leg_mirror_from_10500/params/env.yaml`
和 `CurveVelocityCommand._update_command`。

## 本次验证

以下是修改曲率前、最大曲率 0.05/m 时的验证记录，不作为 0.40/m 的性能结论。
当时使用上述诊断脚本完整运行 3000 步／60 秒，正常以 `max_steps` 结束。
3000 步全部为 `tracking`，有效双相机包 504/504，无运行异常、无倾倒停机。
排除前 5 秒后：实际前进速度 0.2188 m/s，机身高度 P95−P5 为 6.01 mm，
TCP 高度幅度 16.47 mm，TCP 地面投影误差 P95 为 17.54 mm、最大 23.32 mm。
全程含启动的最大投影误差为 118.08 mm。

本次四脚世界高度幅度 FL／FR／RL／RR 为 78.0／89.5／92.3／66.8 mm，
说明左右抬脚幅度仍不一致。原始结果：

```text
outputs/VD3_MIRROR_2000_DEPLOYMENT_CHECK_20261105/session_20261105_160439_unmklgfs/rear_debug_summary.json
```

另用 VS Code 已安装的 debugpy 1.8.20 做 DAP 实际启动与子进程调试验证：
自动连接环境探测、视觉与仿真子进程，在 `run_vd1_mujoco.py` 构建 actor 观测处
命中条件断点，继续后正常完成 500 步／10 秒，退出码 0。验证过程保留在：

```text
outputs/VD3_MIRROR_2000_DEPLOYMENT_CHECK_20261105/debug_adapter_check.json
outputs/VD3_MIRROR_2000_DEPLOYMENT_CHECK_20261105/check_debug_adapter.py
```

启动参数与诊断统计的 17 项单元测试通过，两处 VS Code JSON 与 shell 语法检查通过。
新的稳态汇总器还与此前独立评估的 2000 会话逐项交叉核对。

首次启动检查时曾冗余传入 `--gun-mass-mode zero`，VD3 入口因模型本身已是零附加质量
而拒绝启动；已去掉该参数并完成上述复跑。失败记录
`session_20261105_160236_a4qurt0_` 原样保留。

交付时已打开 MuJoCo 主窗口及前 RGB／后 RGBD 叠加窗口，确认进入 `tracking`。
本次可视会话为 `outputs/VD3_REAR_VISION_DEBUG/session_20261105_160721_a06cvnbg`，
通过 `--show --max-steps 0` 启动，关闭主窗口后会自动写入诊断汇总。
