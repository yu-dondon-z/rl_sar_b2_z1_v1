# VD3 MuJoCo 部署记录

2026-11-05 更新：当前默认策略为镜像微调 `model_2000`，启动命令不变。
当前配置、VS Code 入口和诊断指令见
[镜像 2000 部署与调试](VD3_MIRROR_2000_MUJOCO_DEBUG_20261105.md)。以下保留 11 月 4 日的 10500 部署记录。

本地 MuJoCo 已接入 VD3 第 10500 轮导出策略：

```text
/home/dong/rl_sar_b2_z1/outputs/VD3_SUPPORT_REVIEW_20261104/compare_10500_12500/policy_10500.pt
SHA256 f45e42e506e407fbba0ba77e7d3ab78f4f09564a7d349ae92f7a62d6900e68dd
```

场景使用 VD3 的 `b2_z1_vd3_zero_gun.urdf`。枪的视觉和碰撞几何并入 `link06`，没有独立枪刚体；名义总质量为 78.9990202 kg，TCP 是 `link06` 上的虚拟 site。actor 仍接收 70 维观测，输出 18 维动作；评估用的世界位姿和接触信息只写入独立日志，不返回 actor。

启动带前 RGB、后 RGBD 感知和 MuJoCo 窗口的部署：

```bash
cd /home/dong/rl_sar_b2_z1
bash scripts/play_vd3_mujoco.sh
```

默认参数是 0.2 m/s、0.05/m 曲率、`rhanet` 双相机感知、后视短时保持保护和 200 Hz 物理控制。窗口关闭或 Ctrl-C 结束；每次会在 `outputs/VD3_MUJOCO/session_<时间>_<随机后缀>/` 保留 `session.json`、`play.log`、`vision.log`、相机图像、actor 轨迹、评估真值和 `summary.json`。

无窗口测试：

```bash
bash scripts/play_vd3_mujoco.sh --headless --no-display --max-steps 3000
```

`3000` 个 actor 步是 60 秒。最近一次完整双相机回放：

```text
outputs/VD3_MUJOCO/session_20261104_193948_6h3p54dl
```

结果为 3000 步、60 秒、`max_steps` 正常结束；有效视觉命令 2836 步，tracking 2694 步，rear_hold 142 步，stopped 164 步。最大倾角约 8.92°，前进位移约 8.48 m，TCP 投影误差 P95 约 2.32 cm，前视曲线横向误差 P95 约 3.76 cm。后 RGBD 中心线出现断段时，感知会拒帧并由 guarded 控制器执行短时保持或停车；这是真实输入时效/几何检查的结果，未伪造有效帧。

这项验证覆盖了 MuJoCo 动力学、双相机图像链路和策略推理，不等同于实机验收。相机检测仍由现有 RHA/几何桥处理，策略本身并未从 RGB/RGBD 图像端到端输入。
