# 前视路径估计：板载同步（2026-11-06）

后续 R1+Y 实机故障与局部二次曲线修正见
[FRONT_LOCAL_RECOVERY_20261106.md](FRONT_LOCAL_RECOVERY_20261106.md)。
下文三次曲线及回放结果是上一安装版本的历史记录。

本次在现有正式目录增量更新，保留策略、标定、C++ IO、站立增益与手柄状态机。
策略仍为 VD3 mirror model_2000，前进指令 0.20 m/s，50 Hz。

## 本版逻辑

- 前 RGB 单独观测焊缝，通过与 MuJoCo 相同的四系数三次曲线 Kalman 估计生成转向。
  关闭前后焊缝联合拟合；后 RGBD 仍提供原工作点的末端目标。
- 前投影使用既有前相机标定、图像接收之前最近一次 B2 IMU 重力方向以及固定高度。
  主进程向共享环形缓冲发布真实 B2 接收时间与重力，不积分位置、速度、航向。
  图像/IMU 接收间隔超过 100 ms、没有前序 IMU 时拒绝前投影。
- 图像和 IMU 对齐依据均为主机接收时间，不能声称硬件曝光同步。
  不改写图像时间，不放宽原 40 ms 配对和 1 s 帧年龄限制。
- 配置 `front_nominal_base_height_m: 0.55` 是沿用仿真的平地模型假设，
  **不是现场标定值或实时高度**。机身高度变化、地面坡度、近场外推均会带来偏差。
- 一起安装此前已做录制图验证的视觉耗时修复，见 `VISION_DEADLINE_20261106.md`：
  按需解码 JPEG、中心线局部计算、数值库线程调度。没有改视觉/actor 权重或增益。
- 后视失效仍保留原末端保持及停车逻辑。前视几何独立不代表两个视觉管线已异步解耦。

## 现场启动

在板载终端：

```bash
cd /home/unitree/rl_sar_b2_z1_VD2
source .venv/bin/activate
bash start_real.sh --interface eth0
```

沿用手柄操作：A 起立、Y 到准备位、R1+Y 开始视觉跟踪；再次 R1+Y 停止，
B 按原流程收臂再趴下，L1+X 为原紧急停止。按键需释放后再按。
仅命令启动不会绕过 A 自动行走。出现 ready 后查看 `[vision]` 的实际有效性和原因。
`runs/<时间>/control.jsonl.gz` 的 `visual_status` 包含
`front_steering_method=front_only_cubic_kalman`、`rear_used_for_steering=false`、
`body_motion_estimation=false` 以及曲线外推诊断。

## 不运动的检查

```bash
python run.py --offline-check
python run.py --camera-preview --interface eth0 --seconds 15
python tools/replay_front_path.py --session runs/20261106_123235_704827 --prefix working_pair_ --iterations 3
```

离线检查不初始化控制或相机。相机预览不打开 B2/Z1 控制。
回放使用保存的真实图像和当时控制日志中的 B2 IMU，保留原始图像时间；
在历史时钟上计算几何有效性，不能把回放标记成实时帧有效性或实机精度验收。
原 `check_visual_stream.py` 是相机/耗时诊断，使用旧的后视平面投影；
它不替代本版前视估计器回放或实机跟踪验收。

增量包安装器对原文件哈希核对后才覆盖，并在 `patch_backups/front_path_*`
保存修改前文件、原 MANIFEST 和补丁记录。无需重新编译 C++。
当前版本的 MuJoCo 120 秒没有倾倒，但 TCP 投影 P95 约 12.8 cm，
尚未达到用户要求的 5 cm。板载同步不能作为连续实机行走或末端精度通过的证明。

本次同步验证结果及实际备份目录记录在开发机部署报告中。

真实会话 `20261106_123235_704827` 的 `latest_valid_` 图像回放能生成前视估计，
但 `working_pair_` 图像的前视近端约 1.31 m，拟合截距 -0.816 m，超过原 0.8 m
范围而拒绝。没有据此扩大范围、修改标定或改写时间。说明真实 S 弯的盲区外推
仍可能拒绝进入 ready/tracking，不能声称这次同步已解决所有视觉行走问题。
现场首次观察可将机器人沿较直的线段对齐，依据实际 `[vision]` 原因判断。
