# VD13训练完成后的Play与MuJoCo

2026-11-16。已做好固定最终检查点的独立启动入口，Isaac Play和MuJoCo真值命令回放均完成30秒；**双相机视觉闭环尚未通过，不能称为已验收部署。** 没有改奖励、追加训练、替换默认actor、导出板载包或操作机器人。

## 模型与启动命令

固定使用：

```text
/home/dong/b2_z1_VD2/logs/rsl_rl/b2_z1_vd13_mount75_amp/2026-11-15_23-51-49_vd13_mount75_resume/model_6199.pt
```

检查点轮号6199，满地面课程；75cm夹持、2.93kg管身、link06中TCP=(0.825,0,0.1)m、0.95m工作距离。两端启动前校验安装／URDF哈希、保存配置、课程、关节顺序及70维输入／18维位置动作；拒绝混用VD5／VD12。历史模型和入口保留。

Isaac Lab真实Play，显示窗口并按实际时间播放：

```bash
cd /home/dong/b2_z1_VD2
bash scripts/play_vd13.sh --viewer --real-time --duration 30
```

默认单环境名义重置，保留原训练任务与终止；这是**几何教师命令**，不是图像视觉。去掉`--viewer --real-time`可离屏保存真实Isaac视频和200Hz日志。训练分布补充检查可用`--reset-mode mixed --num_envs 4 --no-render`。首次失败或原生episode结束即停止，不用自动reset掩盖坏姿态；输出到`outputs/vd13_checked_play/`。默认生成`play.mp4`。

MuJoCo只看策略／模型动作（**不含视觉，不是视觉部署**）：

```bash
cd /home/dong/rl_sar_b2_z1
bash scripts/play_vd13_mujoco_truth.sh --viewer --duration 30
```

真正双相机部署入口：渲染前RGB／后RGB-D → RHANet → 测得裂缝几何 → 命令 → VD13 actor：

```bash
cd /home/dong/rl_sar_b2_z1
bash scripts/deploy_vd13_vision.sh --viewer --vision-window --duration 30
```

**该视觉命令当前会在后视支撑不足时保护停止，不是成功演示命令。** 不回退到真值命令，不伪造接触、路径或帧龄。图像来自MuJoCo渲染，不代表真实相机验证。MuJoCo shell固定使用已有`crack_vision`环境，不需要往Isaac环境安装mujoco。按Ctrl+C／关闭窗口结束。输出分别在`outputs/vd13_mujoco_play/`、`outputs/vd13_vision/`；失败返回非零退出码。

## 本次实际结果

| 检查 | 结果 | 最小保守整枪净隙下界 |
| --- | --- | --- |
| Isaac名义重置，1环境 | 30秒完整episode，仅time_out，无其它原生终止或几何守护失败 | 34.16mm |
| Isaac训练mixed重置，4环境 | 全部30秒，仅time_out，无其它原生终止或几何守护失败 | 33.30mm |
| MuJoCo名义重置、直线真值命令 | 30秒，未触发保护，无自动重置 | 37.36mm |
| MuJoCo双视觉，RHANet | 2.24秒停止：后视裂缝不足以支撑0.95m工作点 | 34.07mm |
| MuJoCo图像提取对照，classical | 2.14秒，同一后视支撑失败；只做诊断，不替代默认RHANet | 35.95mm |

名义Play的[实际Isaac视频](/home/dong/b2_z1_VD2/outputs/vd13_checked_play/play_20261116_090541_mk3wlzg5/play.mp4)中可见抬臂及枪尖下放。保守净隙是URDF简化碰撞几何的离散排除下界，且不检查整个父体link06，不是完整网格自碰撞／穿透证明。

200Hz日志离线统计：去掉2秒启动，采用三阶Butterworth >3Hz、双向滤波、两端各舍弃1秒，统计3–29秒。Isaac名义／mixed的三轴枪尖高频RMS分别1.70／1.58mm；MuJoCo直线真值样本1.07mm，其中世界Y为0.55mm。Isaac场景有随机初始朝向和训练曲线，**世界Y不能直接等同机械臂左右方向**，也不能与MuJoCo直线样本作受控改善比较。

Isaac名义样本启动后三轴原始位置误差RMS仍24.38mm，枪轴角误差均值13.39°；mixed分别为23.19–26.64mm、12.43–14.55°。低高频RMS不代表完整位姿精度已达标。落脚事件按向上净力>30N、卸载至少20ms后触地，统计随后50ms峰值；Isaac mixed四脚P95依FL/FR/RL/RR为309／249／318／304N。MuJoCo直线样本为259／223／277／287N，先将原生力日志与前一状态样本按相同采样时刻对齐。没有同安装、同路径／重置的匹配旧基线和明确验收阈值，不能据此宣称砸地已解决。

## 视觉为什么未通过

启动下放枪体时，后视中可见裂缝的远端被枪体遮挡／排除。RHANet记录中，sequence98的工作点仍直接可见；101需要25.4mm显式局部线段估计；104、107已超出原有50mm远端估计上限。最后有效路径长度约0.35m，并非“识别有路径就足以生成工作点”。[末帧覆盖图](/home/dong/rl_sar_b2_z1/outputs/vd13_vision/vision_20261116_090739_nnajf09m/latest_dual_overlay.png)可看到这一截断。

已有0.6秒历史观测缓存仅按已发送速度命令传播，不是实测里程计。离线重放同一批真实图像包／控制时刻，复现了停止：接收107时缓存从95／98／101／104变成仅107；代码在重叠路径不一致超过25mm时丢弃冲突旧路径，避免把不同路径平均起来。当前缓存因此不能可靠桥接遮挡。可见性变化和运动传播失配是后续要处理的问题；不能仅据此确认物理驱动或网络是根因。

静态水平底盘的30.3cm前瞻不保证启动／俯仰／跟踪中的工作点始终可见。本次没有改相机标定、工作距离、遮挡阈值、帧时效或缓存期限来绕过失败。下一步优先解决真实图像的动态工作点覆盖和观测的自运动补偿；不要为此先改奖励或盲目追加训练。需要改变实际安装／工作距离时，应先明确新物理配置，再判断是否训练。

## 原始记录与回归

- [Isaac名义结果](/home/dong/b2_z1_VD2/outputs/vd13_checked_play/play_20261116_090541_mk3wlzg5/summary.json)、[mixed结果](/home/dong/b2_z1_VD2/outputs/vd13_checked_play/play_20261116_090910_ojfybew6/summary.json)。
- [名义离线运动分析](/home/dong/b2_z1_VD2/outputs/vd13_play_analysis/vd12_play_analysis_t4gb_s2n/analysis.json)、[mixed离线分析](/home/dong/b2_z1_VD2/outputs/vd13_play_analysis/vd12_play_analysis_k4j1tvto/analysis.json)：沿用公共VD12格式，manifest／checkpoint／URDF实际为VD13。
- [MuJoCo真值命令结果](/home/dong/rl_sar_b2_z1/outputs/vd13_mujoco_play/play_20261116_091311_t13mmrpe/summary.json)；同目录保留200Hz实际状态、原生驱动力／约束日志和来源清单；不是用PD估算冒充实际驱动力。
- [RHANet双视觉结果](/home/dong/rl_sar_b2_z1/outputs/vd13_vision/vision_20261116_090739_nnajf09m/summary.json)、[classical图像对照](/home/dong/rl_sar_b2_z1/outputs/vd13_vision/vision_20261116_092437_f_dpinu5/summary.json)。同目录保留真实相机包、采集时刻、叠图和终止原因。
- CPU回归：训练侧`test_vd12_checked_play.py`17项；MuJoCo侧全部`test_vd12*.py`82项（含动作回放14项、视觉入口16项）、`test_vd13*.py`14项，均通过。三个shell语法及帮助入口也通过。包含两端拒绝错误安装、旧actor、未完成课程和错误工作距离；测试通过不覆盖上述视觉实跑失败。

完整连续视觉未通过；当前只交付隔离的仿真／诊断入口，不发布默认actor，更不开展实机部署。

## 后续：用户指出的姿态缺陷

同日用户指出Isaac Play后部下沉、前脚过窄；30秒未终止不等于步态合格。进一步样本显示后髋较前髋低约12cm、前脚横向站距中位数约21cm。按用户确认，在训练工作区新增[VD14独立姿态奖励候选](../../../../b2_z1_VD2/docs/VD14_POSTURE_TRAINING_20261116.md)，仅加强机身水平和前脚最小站距代价。已做无学习管线检查，未启动训练、未产生改善证据；本报告中的VD13结果和MuJoCo入口均保留，默认actor没有替换。动态视觉遮挡问题仍未解决。
