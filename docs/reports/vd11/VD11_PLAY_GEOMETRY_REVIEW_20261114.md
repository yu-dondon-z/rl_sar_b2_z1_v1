# VD11 Isaac Play 与新枪穿模复核

日期：2026-11-14。结论：**VD11使用了正确的新枪质量，但现有actor没有通过几何验收，不能部署。** 本次未训练、未调奖励/增益、未修改碰撞mask，未连接实机；全部失败日志与检查点保留。

## 版本核对

- VD5 Startup 4498：旧零附加枪重资产，`link06`原质量0.2887581 kg，不适用于现在的2.93 kg长枪。
- VD10/VD11：外径56、内径52、长度1100 mm、枪质量2.93 kg；新TCP与安装方式。Isaac运行时核对`link06=3.2187581 kg`，标称整机81.929016 kg，包含枪重。
- 本次只用VD11 `model_5999.pt`，SHA256为`1db996f55949b8346ff37d91647e94823dd62f0347b7af426bed0ef0b8995dff`。旧VD5默认导出未更换，也不推荐拿它试新枪。

## 本次实测

| 工况 | 首次确证相交 | 结果 |
| --- | --- | --- |
| Isaac训练任务，恢复最终课程，4环境，保留80% AMP/20% nominal复位设置及训练随机化 | 1.280 s，env0和env2 | 枪中心线进入`tail_link`实体，立即停止 |
| Isaac训练任务，明确改为单环境nominal复位，其余同训练 | 1.240 s | 同类尾部相交，立即停止 |
| MuJoCo，修正为训练公式的世界焊缝直线子工况，nominal复位 | 1.200 s | 同类尾部相交，立即停止 |

它们不是等价的统计配对，也不能由短回放计算总体失败率。几何监测只读真实关节角，不修改接触力、不向70维actor增加真值。它使用同一源URDF独立FK核对枪的外圆柱与机器人碰撞primitive；只有枪中心线进入实体的充分证据才报`invalid`，保守包络无法判定的报`uncertain`。负净隙下界不是穿透深度；检查只覆盖已建碰撞形状和离散采样时刻。

Isaac采用Train任务，不使用会改复位/曲线设置的原Play任务。严格加载70→18确定性actor，核对动作和观测关节顺序、观测缩放/裁剪、驱动与物理设置，从checkpoint恢复`ground_blend=1`、`curve_level=0.1`后重新reset。真实接触、原终止与训练随机化保留；无优化器或策略导出。名义复位差分在通过原profile检查后显式设置概率0，并在manifest标明。视频使用外部视口，不是图像感知命令。

MuJoCo不是完整训练任务：无随机化/AMP复位，只是一条直线；命令通过理想几何teacher生成，不是前后视觉闭环。修复了旧固定body-z随底盘下沉把目标拉入地下的问题，但没有改变actor。修正后仍相交，不能把原因仅归给旧目标或查看器。

## 训练端既有证据

对先前32环境最终评估的**env0**已有200 Hz真实6轴臂角进行离线复核，无物理推进：

- 1.240 s首次进入尾部实体；当时joint2=0.0693、joint3=−0.2082 rad，尚未贴限位。
- 1.505 s首次进入底盘主箱；之后大量姿态持续相交，关节也逐渐贴近0 rad限位。
- 当时工具接触力为0；训练配置关闭自碰撞，因此没有相应接触力并不代表没有穿模。
- 源URDF与评估归档SHA一致，FK复算TCP高度与日志误差小于1.4 μm。

旧日志只有env0臂角，腿部与其余31环境的关节角没有记录；此离线结论仅覆盖env0的底盘/臂碰撞体，不能推广成32环境穿模率。首次相交发生在触限之前，不能据微小限位求解误差认定唯一根因。

这足以否决“现有VD11已经可以部署”，但不等于证明唯一根因，也不等于从短回放推断PPO是否收敛。训练物理与验收存在自相交漏洞，旧奖励平台和抖动数值都不能替代物理有效性。

## 日志与视频

- [Isaac混合复位视频](../../../../b2_z1_VD2/outputs/vd11_checked_play_20261114/play_20261114_140636_r4drt8ro/play.mp4)、[摘要](../../../../b2_z1_VD2/outputs/vd11_checked_play_20261114/play_20261114_140636_r4drt8ro/summary.json)。4环境全量实际关节角/速度/目标、TCP、姿态与接触记录在同目录`physics_200hz.jsonl.gz`。
- [Isaac名义复位摘要](../../../../b2_z1_VD2/outputs/vd11_checked_play_20261114/play_20261114_141120_ypfy08_h/summary.json)。此轮额外验证了ActionTerm和关节观测term实际joint IDs，均为训练的18维交错顺序。
- [Isaac单机器人侧视视频](../../../../b2_z1_VD2/outputs/vd11_checked_play_20261114/play_20261114_141431_us59qdqe/play.mp4)、[1.240秒原状态截图](../../../../b2_z1_VD2/outputs/vd11_checked_play_20261114/play_20261114_141431_us59qdqe/03_invalid_geometry_env0_1.240s.png)。同名义初态的取景检查，改为跟随底盘yaw的侧视，不改变物理姿态；再次在1.240秒截停。该轮文件完整保存后，禁用快速退出的Kit在插件清理阶段卡住，已只终止本次自启动的仿真进程；它不是一轮额外成功回放。
- [纠正世界命令后的MuJoCo摘要](../../../outputs/VD11_MUJOCO_DIAGNOSTIC/mujoco_20261114_140253_vyeneoav/summary.json)、[确证相交原状态截图](../../../outputs/VD11_MUJOCO_DIAGNOSTIC/mujoco_20261114_140253_vyeneoav/state_frames/1.200s_invalid_geometry.png)。
- [已有Isaac env0离线几何复核](../../../../b2_z1_VD2/outputs/vd11_existing_pose_geometry_audit_20261114_486e10c3/existing_final_env0_geometry_summary.json)、[关键帧](../../../../b2_z1_VD2/outputs/vd11_existing_pose_geometry_audit_20261114_486e10c3/env0_actual_keyframes.json)。

本次Play没有声称记录机械臂实际驱动力矩；记录的是实际q/qd及引擎接触力，不能拿PD估算替代真实驱动力矩。

## 本地查看命令

Isaac保真混合复位：

```bash
cd /home/dong/b2_z1_VD2
bash scripts/play_vd11_checked.sh --viewer --real-time --num_envs 4 --duration 30
```

只看一台机器人、名义站姿启动（这是明确的复位差分，不是混合复位原分布）：

```bash
cd /home/dong/b2_z1_VD2
bash scripts/play_vd11_checked.sh --viewer --real-time --num_envs 1 --reset-mode nominal --duration 30
```

默认检查点已钉住本次VD11最终模型。去掉`--viewer`即可无界面记录视频；每次创建新目录，不覆盖旧日志。发生确证相交就提前停止并留存原姿态，**30秒是上限，不保证跑满，更不应关闭guard凑满时长**。

请用上述shell包装入口：本机Kit快速退出可能吞掉Python非零返回码，最终入口保留快速退出避免插件清理卡住，并由包装脚本读取独立结果，确证几何无效返回2、缺失结果返回1；不能只凭Kit自己的退出0判成功。

MuJoCo独立诊断（不是正式部署）：

```bash
cd /home/dong/rl_sar_b2_z1
MUJOCO_GL=glfw /home/dong/anaconda3/envs/crack_vision/bin/python scripts/diagnose_vd11_gait.py \
  --checkpoint /home/dong/b2_z1_VD2/logs/rsl_rl/b2_z1_vd11_motion_quality_amp/2026-11-13_17-56-49_vd11_motion_quality_long_20261113/model_5999.pt \
  --viewer --duration 30 --save-frames
```

## 下一步

先核对整根枪的目标与过渡可达且不穿自身，再在隔离的新训练候选中补正确的枪—机身/机械臂选择性碰撞或明确的几何约束与终止，并与MuJoCo对齐；不能只开查看器全自碰撞，也不能靠调增益掩盖错误姿态。先通过物理有效性检查，再固定配置充分训练、复核左右摆动和逐脚冲击。保留VD11作为新枪对照，不回退到零枪重VD5冒充新枪方案，不自动追加长训或更换默认actor。

本次CPU回归：Play契约19项、独立几何15项、MuJoCo诊断16项全部通过；这些测试不能取代上述失败的物理验收。原VD5导出SHA256仍为`989895d8077841855b1aa6791aca24e3cf3eb4a7457bb3d902eec19fc6494075`。
