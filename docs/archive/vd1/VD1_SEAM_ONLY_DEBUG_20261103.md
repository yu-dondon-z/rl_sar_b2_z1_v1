# VD1 无标记双视觉调试（2026-11-03）

当前F5默认使用“后RGBD测焊缝，直接给策略末端目标”。取消两个彩色球的检测要求及`0.35 × (目标位置 - 视觉TCP)`反馈项。原actor、70维输入／18维输出、PD、路面接触和前后视转向方法继续沿用。

后续地面更新：F5默认已改成黑胶带场景，曲线作为地面纹理绘制，宽2 cm、厚度为0；Z=0平地接触，无沟槽／侧壁。目标生成和视觉控制逻辑继续沿用。本说明下方60秒数据为更新前的物理裂缝场景。

当前可视地面已扩至190 × 100 m，保留中央宽6 m的黑胶带纹理分辨率，两侧增加同色平地，以减少地面边界对视觉的影响。曲率已恢复0.30/m，碰撞仍由同一平面承担。

当前F5增加零质量枪体实验：仅在生成的MuJoCo模型中把`link_gun`质量与惯量归零，保留枪体外形、碰撞、安装座和其他链接。总质量约78.999 kg，原URDF与actor不改。下方历史60秒结果使用原枪质量约3.259 kg，不是此次实验结果。

零质量实验实际运行3000步／60秒，无运行错误或倾斜保护终止，但未正常向前行走：X净位移−2.34 m，最大倾斜15.12°；tracking 872步、rear_hold 670步、stopped 1458步。首次stopped为1.80秒，原因为后视未覆盖工作位置且超过保持期限。不能视为连续跟踪通过。127项本地测试通过，记录为`outputs/VD1_DUAL_VISION_DEBUG/zero_gun_checks/session_20261103_203618_5ccw8wrg/`。

## 使用

在VS Code选择 **VD1：双视觉＋黑胶带（MuJoCo 调试）**，按F5，或执行：

```bash
bash scripts/debug_vd1_dual_vision.sh
```

默认曲率0.30/m（`--curvature-stage 5`）、`--gun-mass-mode zero`、前进指令0.4 m/s、最长60秒仿真。追加`--gun-mass-mode original`可恢复原枪质量。指令速度不等于实测速度。RHANet热身并得到有效双相机结果后开始行走。显示MuJoCo与三联相机窗口，按Ctrl+C结束。

## 当前流程

1. 后RGBD以真实渲染的对齐深度拟合地面，剔除高出地面的枪身／腿部边缘；RHANet检测裂缝并从深度反投影得到机身坐标下的焊缝点。保留置信度、最少点数、长度、几何间隙、深度支持与前景过滤，后视有效性不再取决于是否看见枪尖。
2. 前RGB以RHANet分割裂缝，按测得的前向覆盖选择组件，用后视测得地面将像素射线转换成三维点。
3. 前后路径通过原有帧配对、时效与后视连续性检查。联合三次拟合生成0.4–0.6 m引导点；拟合失败回退前视可见点转向。
4. 后视路径必须覆盖机身坐标`x=-1.02 m`。在该位置插值得到`y_seam`，直接生成`[-1.02, y_seam, -0.30]`。X和Z沿用训练工作位置；深度负责测焊缝，目标Z并非地面或沟底高度。
5. 位置裁剪到原工作空间`x∈[-1.08,-0.96]`、`y∈[-0.15,0.15]`、`z∈[-0.34,-0.26] m`，再作0.30秒平滑与0.15 m/s位置变化限速。末端姿态沿用原固定目标。取消视觉枪尖误差修正，不伪造TCP测量。
6. 底盘指令、末端目标与编码器／IMU等本体信息组成原70维actor输入；actor输出18关节动作，由原PD执行。仅启动时通过编码器正运动学初始化目标过渡；行走中不另行计算枪尖位置给视觉控制器。
7. 后视短时失效仍进入`rear_hold`，冻结末端目标并使用有效前视转向；超过0.6秒或前视／帧检查失败则减速，恢复继续要求不同的新帧。无标记不等于无焊缝或无深度也能继续运行。

没有增加真值输入。解析曲线、世界位姿及真实TCP仍只用于场景生成与独立评价。当前仍无相机帧运动补偿，焊缝目标处于图像捕获时刻的机身坐标系。

## 调试与记录

- MuJoCo场景不生成青色／品红色标记球；当前调试中枪体质量与惯量归零，外形、碰撞及不可见的`tool_tcp`评价site仍保留。原URDF不修改，其他链接仍保留原质量／惯量。
- 三联窗口为前RGB转向、后RGBD焊缝目标、后深度。后视只显示测得焊缝中心线，不再显示测得TCP或TCP视觉误差。
- 在`scripts/vd1_visual_guard.py`的`rear_seam_target()`或`update()`设断点，可以查看原始后视点、`rear_nominal_target_b`、限幅与平滑后的`tool_pose_b`。
- `session.json`、`runtime_contract.json`与`summary.json`记录`tool_target_mode=rear_seam_direct`、`visual_tcp_feedback=false`、`requires_tcp_markers=false`、`tool_markers_in_scene=false`。感知结果不再产生`tcp_measured_b`、`markers`或TCP误差。
- `session.json`记录`gun_mass_mode`，`runtime_contract.json`与`summary.json`还记录实际`gun_mass_kg`和`robot_mass_kg`，区分原质量与零质量试验。
- `status.tcp_projection_error_m=null`表示没有视觉TCP评价；`summary.evaluation`中的TCP误差来自独立仿真真值评价，未反馈控制。
- 命令行追加`--visual-controller original`可对照旧的双标记与视觉TCP反馈流程。

`VD1_DUAL_VISION_CONTROL_LOGIC_20261103_原文对应版.txt`已按用户要求保留原九节结构并更新为当前黑胶带、无标记感知和直接末端目标流程，可对照阅读。

## 本次检查

117项本地VD1测试通过，覆盖无TCP字段直接生成目标、忽略遗留TCP字段、缺焊缝／地面拒绝、无标记RGBD反投影、前景枪边缘剔除、去掉标记球后质量惯量不变及F5参数转发。

完整RHANet／MuJoCo无标记检查记录：`outputs/VD1_DUAL_VISION_DEBUG/seam_only_checks/session_20261103_190717_ebhftmeb/`。运行期间纳入清单的源码没有变化。

| 项目 | 结果 |
| --- | --- |
| 终止原因 | `max_steps`，无运行错误 |
| 运行长度 | 3000步，60秒仿真 |
| 世界X净位移 | 28.50 m，越过2–22 m弯曲区 |
| 最大机身倾斜 | 6.01° |
| 控制状态 | tracking 2978步，rear_hold 22步，stopped 0步 |
| 有效控制命令 | 3000步 |
| 视觉结果包 | 847包，后视均有效；0包包含TCP测量／标记字段 |
| 底盘路径横向误差P95 | 2.77 cm |
| 独立TCP投影误差P95／最大 | 4.24／7.10 cm，包含启动阶段 |

本次运行证明无标记双视觉流程能够完成这条仿真路径，不代表焊接精度验收。取消视觉TCP修正后，实际枪尖误差由原策略的跟踪能力决定。最小足端水平边沿间距为负，仍未通过“足端全程避开裂缝边缘”验收；本次没有进一步调整控制或训练策略。

`latest_dual_overlay.png`可在VS Code直接打开，`summary.json`记录上述统计，`actor_trace.jsonl.gz`包含命令／观测／动作，`dual_vision_packets.jsonl.gz`保留图像感知结果，`evaluation_truth.jsonl.gz`为独立仿真真值评价。
