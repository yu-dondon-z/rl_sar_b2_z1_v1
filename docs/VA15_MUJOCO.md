# VA15 Isaac → MuJoCo 仿真部署

独立目录：`policy/b2_z1_no_gun/VA15`。不覆盖 V19/v4，不更新实机。
来源：`2026-09-28_15-07-31_va15_curriculum_from2500/model_5000.pt`。
`config.json` 保存 checkpoint 路径及 SHA256。

## 启动

VS Code 打开 `/home/dong/rl_sar_b2_z1`，选择
**部署 VA15：MuJoCo 裂缝视觉（model_5000）**，按 F5。

```bash
cd /home/dong/rl_sar_b2_z1
MUJOCO_GL=egl /home/dong/anaconda3/envs/crack_vision/bin/python scripts/run_b2_z1_v4.py \
  --profile policy/b2_z1_no_gun/VA15 --mode vision --display-vision --speed 0.22
```

按键须在启动程序的终端输入，不需要回车：

1. `0`：站起，等待 `Standing settled`。
2. `1`：策略接管准备，等待 `Arm start pose ready`。
3. 视觉显示有效后，`T`：开始视觉轨迹跟踪。
4. 再按 `T` 或空格：停止跟踪，缓停。
5. `9`：自动先减速、恢复支撑，再收臂趴下。无需先手动清零。
6. `Ctrl+C`：退出仿真及本次视觉子进程。

按 `1` 后显示圆柱形焊枪并启用物理碰撞；按 `9` 收臂前隐藏并移除碰撞。
初始趴下和按 `0` 站起阶段不显示焊枪。这里只模拟装卸，不要求操作实物。
圆柱半径 2.8 cm、全长 1.10 m，保留原有延伸 TCP 和 1 kg 负载代理。
这是简化工具模型，并非按实际焊枪质量和惯量标定的模型。

## 对齐内容

- 70 维观测、18 维动作、原始上一动作观测；无观测归一化。
- 50 Hz 策略、200 Hz 腿部 PD、1 kHz MuJoCo；动作限幅 5，缩放 0.25，EMA 0.2。
- 原始默认关节角和后向 Z1 安装；PD 160/5，力矩限幅保留训练设置。
- TCP 工作空间 z 为 `[-0.55, -0.30]`，未沿用旧版过高下限。
- 焊枪碰撞体按 `1` 装入、按 `9` 移除；TCP 坐标链保持不变。
  训练模型没有这段实体碰撞几何，加入碰撞后的接触行为属于部署差异。
- 视觉采用 RHACrackNet 网络和深度反投影，不使用真值曲线代替网络结果。
- 地面为曲线高度场凹槽，名义宽 2 cm、深 5 cm；高度场栅格使边缘有离散误差。
- 裂缝底部路径加 0.06 m 世界竖直偏移，即 5 cm 槽深 + 1 cm 路面离地目标（2026-09-30 修改）。策略权重和训练时 9 cm 目标未更改；实际高度受跟踪误差、工作空间限幅及焊枪碰撞影响。此补偿适用于本仿真地面，不是实机通用标定。
- 每次启动使用独立临时交换目录，终端打印其位置。

## 检查与限制

导出脚本：`scripts/export_b2_z1_va15.py`，拒绝覆盖已存在 VA15 目录。
256 组随机输入的导出网络误差为 0。
2026-09-29 本地回归（修改到 1 cm 前的旧配置，不代表新高度效果）：

- 本体 0.22 m/s 行走 45 秒，无摔倒，但累计右偏约 1.4 m。
- 最终配置视觉测试：第 50 秒发起按 9，约第 57.42 秒完成趴下；
  跟踪结束时机身前进约 9.07 m，15 秒之后跟踪阶段最大倾角约 1.96 度。
- TCP 激活样本相对名义裂缝 `y(x)` 的误差均值约 1.56 cm、最大约 14.64 cm
  （包含首次接入过程）；这是世界 y 方向误差，不是完整三维跟踪误差。
- 未发生跌倒，最终状态 `idle`；这只是一次短程仿真回归，不代表长期鲁棒性。
- 测试轨迹保存于 `outputs/VA15/`。

复测视觉及运动中趴下：

```bash
MUJOCO_GL=egl /home/dong/anaconda3/envs/crack_vision/bin/python scripts/run_b2_z1_v4.py \
  --profile policy/b2_z1_no_gun/VA15 --mode vision --speed 0.22 \
  --headless --auto-start --duration 60 --auto-get-down-at 50 \
  --output /tmp/va15_mujoco_retest.json
```

纯本体无航向反馈时仍存在累计向右偏移，不应称为稳定直线行走。
所有验证仅针对 MuJoCo；不代表实机可直接使用。
