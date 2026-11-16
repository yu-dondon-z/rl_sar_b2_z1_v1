# VD12最终模型：Play与MuJoCo仿真入口

2026-11-14。正式6000轮训练已结束，使用`model_5999.pt`。两端查看脚本已隔离，默认VD5 actor不变；**仅本地仿真，不是板载、实机或真实双相机部署**。

后续更正：本文MuJoCo结果是**真值命令下的诊断回放**，不能当作视觉部署交付。双相机图像部署入口为 `bash scripts/deploy_vd12_vision.sh --viewer --vision-window --duration 30`，使用RHANet处理实际渲染的前RGB、后RGB-D，且不回退到本文的真值命令。视觉实跑结果单独记录，不沿用本文的“跑满30秒”结论。

## 直接使用

Isaac Lab Play：

```bash
cd /home/dong/b2_z1_VD2
bash scripts/play_vd12_final.sh --viewer --real-time --duration 30
```

MuJoCo仿真：

```bash
cd /home/dong/rl_sar_b2_z1
bash scripts/play_vd12_mujoco.sh --viewer --duration 30
```

两个入口均默认固定正式VD12最终`model_5999.pt`，不会误选2000轮、VD11或smoke。Isaac包装使用`env_isaaclab`；MuJoCo包装使用已有`crack_vision`，启动前检查依赖，避免之前`env_isaaclab`没有`mujoco`的问题，不安装新包。不要同时运行两个窗口。

Isaac窗口首次启动需加载资产。脚本按请求时长运行，或在首次违规／终止时停止，不跨reset继续演示。MuJoCo窗口可用鼠标调整视角；提前关闭窗口会以非完成状态退出，不冒充跑满。两端均保留原始日志和独立输出目录。

不打开窗口的复核：

```bash
cd /home/dong/b2_z1_VD2
bash scripts/play_vd12_final.sh --num_envs 4 --reset-mode mixed --duration 30 --no-render
cd /home/dong/rl_sar_b2_z1
bash scripts/play_vd12_mujoco.sh --duration 30
```

历史`play_vd12_checked.sh`仍默认2000轮，保留旧评估复现；日常看最终模型用新的`play_vd12_final.sh`。需要其他检查点可显式传`--checkpoint /完整路径/model_编号.pt`，仍须通过来源、课程和配置核对。

## 本轮实际验证

最终checkpoint SHA256：`f69ec8e2ea444a51d7e8248c56a5f89c9ab615afd9e32ab22a273c26e08cfd5d`。内部迭代5999，92个tensor及AMP统计均有限；保存环境／当前几何profile匹配，归档中226个source/scripts文件与当前内容一致。训练进程已结束，TensorBoard记录到5999。

已执行Isaac名义起步30秒（含实际RTX视频）与4环境mixed各30秒；全部仅时间上限终止，未触发整枪禁区、支地、底盘接触或倾倒终止。真实5秒、30秒画面可见机械臂正常抬起。MuJoCo最终入口名义直线30秒也完成，未触发安全终止；另已完成3秒viewer窗口启动测试并正常退出。这是隔离查看入口，不是默认策略发布。

入口验证：Isaac最终包装2项测试、MuJoCo入口14项测试通过；独立11种保存配置失配全部被拒绝。最终MuJoCo源码与运行manifest的SHA一致。

| 实际样本 | Isaac名义 | Isaac mixed 4环境 | MuJoCo名义直线 |
| --- | ---: | ---: | ---: |
| 最小保守净隙下界 | 29.66 mm | 30.17 mm | 36.46 mm |
| TCP >3 Hz三轴RMS | 1.491 mm | 1.414 mm（合并） | 1.006 mm |
| 0.5–3 Hz跟踪误差三轴RMS | 4.433 mm | 3.945 mm（合并） | 2.350 mm |
| 2秒startup后原始位置误差三轴RMS | 24.63 mm | 各环境18.74–21.14 mm | 16.90 mm |
| startup后枪轴角误差均值 | 14.03° | 各环境13.64–14.30° | 13.81° |

滤波评分窗口统一为3–29秒：丢弃2秒startup及两端各1秒滤波边界，WORLD XYZ坐标，不是底盘横向投影。MuJoCo只是名义复位／直线命令且没有训练随机化，不能用表中数字宣称MuJoCo比Isaac更好或两端精确等价。

Isaac mixed的815个完整触地事件，FL／FR／RL／RR的50 ms峰值p95分别237.2／249.4／260.4／265.3 N；前20 ms最大下降速度p95为0.842／0.783／0.763／0.630 m/s。协议同2000轮分析（30 N触地阈值、20 ms卸载、50 ms峰值／冲量），但只代表这些样本，不能宣布砸地彻底解决。名义回放仍有FL孤立落地峰值541.1 N，不隐藏这一尾部事件。

**高频抖动和常见落脚数值已有较好的阶段表现，但枪轴仍有约14°平均误差，原始跟踪误差也未消失，不能据跑满30秒宣称任务验收完成。** 最近500轮回报仍有上升，训练结束不等于已经收敛。本轮不继续训练、不改奖励或增益。

## MuJoCo入口的范围

- 载入同一2.93 kg、1100 mm新枪模型和最终70→18 actor，使用世界焊缝起点锚定、2秒下放和0.80 m后随工作点；不恢复此前错误的固定机身目标。
- 这是仿真真值生成的直线路径命令，不是相机图像识别；真值不额外追加给actor。名义起步是训练mixed复位之外的显式差分。
- 使用同源整枪44对保守净隙规则，200 Hz采样、四子步锁存、20 ms边界拒绝，保存首次违规而不自动reset。负下界是保守禁区违反，不冒充穿透深度或接触力。
- 复用既有隐式位置数值适配与质量度量速度冲量，不是PhysX精确移植，也没有本轮新调增益。Mu臂的既有离散适配使用原源参数160／5及`h*Kp`数值转换，不能笼统称两个引擎的原生求解器／gain完全一样。
- Mu日志里的`actuator_force`和约束缓冲区来自实际native求解，不是PD估算。力样本属于步开始状态，q／qd／几何属于5 ms后的步末状态；本轮不把二者假装同相来计算触地前速度，也不报告伪造的跨引擎逐脚p95。

## 原始结果与脚本

- [Isaac实际视频](/home/dong/b2_z1_VD2/outputs/vd12_checked_play_20261114/play_20261114_173752_ukej6hft/play.mp4)、[名义结果](/home/dong/b2_z1_VD2/outputs/vd12_checked_play_20261114/play_20261114_173752_ukej6hft/summary.json)、[分析](/home/dong/b2_z1_VD2/outputs/vd12_play_analysis_0d_dfk6j/analysis.json)。本次是实际RTX画面，不是2000轮的简化离线重建。
- [Isaac mixed结果](/home/dong/b2_z1_VD2/outputs/vd12_checked_play_20261114/play_20261114_174843_aq5c26tt/summary.json)、[分析](/home/dong/b2_z1_VD2/outputs/vd12_play_analysis_ats90zew/analysis.json)。
- [MuJoCo最终入口完整动态结果](/home/dong/rl_sar_b2_z1/outputs/vd12_mujoco_play/play_20261114_175405_awn0nitk/summary.json)、[TCP-only分析](/home/dong/rl_sar_b2_z1/outputs/vd12_final_cpu_metrics_20261114_nJmvRA69/play_20261114_175405_awn0nitk_tcp_analysis.json)、[3秒viewer测试](/home/dong/rl_sar_b2_z1/outputs/vd12_mujoco_play/play_20261114_175707_ojq20npt/summary.json)。各目录保留200 Hz实际物理日志及50 Hz actor日志；不以未对齐的Mu力／足速计算触地指标。初次动态结果目录亦保留。
- [Isaac最终包装](/home/dong/b2_z1_VD2/scripts/play_vd12_final.sh)、[受保护Play实现](/home/dong/b2_z1_VD2/scripts/diagnostics/play_vd12_checked.py)、[MuJoCo包装](/home/dong/rl_sar_b2_z1/scripts/play_vd12_mujoco.sh)、[MuJoCo实现](/home/dong/rl_sar_b2_z1/scripts/play_vd12_mujoco.py)。

全部旧检查点、日志和actor保留；没有板载同步、实机控制或默认actor替换。默认VD5 actor SHA256仍为`989895d8077841855b1aa6791aca24e3cf3eb4a7457bb3d902eec19fc6494075`。
