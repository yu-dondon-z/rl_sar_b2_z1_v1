# R1+Y 后仅机械臂晃动：诊断、修正与板载安装

时间：2026-11-06。接续 `VD2_FRONT_PATH_BOARD_SYNC_20261106.md`。
输入：用户终端附件 `61414f76-d371-4363-99c5-9b4fa9a1ec31/已粘贴的文本.txt`，
以及板载真实会话 `runs/20261106_143908_824883`。
旧程序、日志、标定全部保留；本轮没有启动 Z1、主控制、preflight 或机器人运动。

## 直接故障及证据

R1+Y 两次实际进入 tracking，各 22 个 actor 步，持续约 0.4401 s，然后因前视
估计越界进入 braking。退出日志分别是：

```text
front inferred curve exceeds local tracking range (curvature=-0.086/m, intercept=-0.869m, target_y=0.349m)
front inferred curve exceeds local tracking range (curvature=-0.245/m, intercept=-0.973m, target_y=-0.369m)
```

两次最大 vx 指令均为 0.012361687 m/s，tracking 内指令积分均为 0.0016911475 m。
这是指令积分，不是物理位移。两秒的速度渐入还没完成，策略已开始驱动机械臂，
随后停车，因此出现末端动一下、底盘几乎不动的表现。
本次直接退出由几何触发，B2/Z1 反馈保持新鲜；没有因此宣布实时年龄问题已消失。

前视的具体问题：

1. 最长前向跨度的分量选择会将左侧地面/墙边选为焊缝，近端投影偏左约 1.15 m；
   真正中央胶带的近端通常在 x 约 1.18 m、y 约 -0.056 m。
2. 完整远处 S 弯的三次拟合对盲区截距很敏感，远处弯折把近场外推拉偏。

原站稳观测窗口 45 包中，旧前视估计仅 13 包接受，22 包范围越界、10 包拟合残差
超过 4.5 cm。后视实际覆盖 x=-1.02 m 的工作点仅 25/45 包，另有短段和缺深度。
这两类故障需要分别处理，不能把后视无测量伪装成可用数据。

## 已安装修正

更新 ID：`vd2_front_local_20261106`，10 个文件。

- 前 RGB 独立地面投影模式按最近可达端点选分量并提取中心线，避免长侧边优先。
- 最近可见 0.55 m 的局部二次曲线＋三系数 Kalman，替代完整 1.5 m 三次拟合。
  多帧只平滑曲线系数；多项式原点变换不表示机身位姿或运动估计。
- ready 中要求连续 3 个不同且一致的有效前后测量。重复包不计数，几何跳变、
  无效包或后视无工作点重置检查；已有启动请求 5 s 超时、取消、停止逻辑保留。
- 前视曲线滤波状态带入 tracking，前进和末端渐入仍从零开始，避免把准备过程当行走。
- 不调整增益、策略、标定、速度 0.20 m/s 或帧时效/配对阈值。

## 验证

开发机：视觉回归 32 项、前视估计器 12 项通过；完整重新组装候选离线检查
81/81 通过。第一次手工复制到旧临时包的检查有一次 MANIFEST 过期失败，随后
重新组装，未修改测试或绕过哈希检查。

MuJoCo：`mujoco/session_20261106_145754_duim8mdh`，0.20 m/s、0.4/m，
120 s / 6000 步全部 tracking，无倾倒。TCP 投影 P95 0.11673 m，
机身横向 P95 0.05547 m，最大倾角约 4.01°。**末端 5 cm 目标仍未达到**。
此仿真不能替代实机连续行走验收。

板载先复制到隔离目录 `/home/unitree/vd2_front_local_check_20261106_ob8cxy3r`，
安装、自检 81/81 通过。CPU 复算本次保存图像，各 3 次：

| 图像组 | 总处理时长 | 前视估计 | 前后工作点几何 |
|---|---|---|---|
| working_pair_ | 306.5～322.4 ms | 有效 | 有效 |
| latest_valid_ | 309.1～328.5 ms | 有效 | 有效 |
| valid_pair_ | 260.9～285.5 ms | 有效 | 后视不覆盖 x=-1.02 m，拒绝 |

这些是保存图和历史 IMU 的离线复算，保留真实接收时间，不是完整图像序列，
不包含现场 USB/SDK 采集、主程序并行负载或真实执行器响应。

随后安装正式目录 `/home/unitree/rl_sar_b2_z1_VD2`：

- 原文件与 MANIFEST 备份：
  `/home/unitree/rl_sar_b2_z1_VD2/patch_backups/front_local_20261106_152323_204307`。
- 正式离线检查 81/81 通过，4.449 s；MANIFEST 904 个文件全部一致。
- 策略、`python/control.py`、`native/io.cpp`、前后标定和 deployment.yaml 安装前后
  哈希均一致，见证据目录 `formal_install.json`。无需 C++ 重编译。
- 安装前后确认没有 `z1_ctrl`、主控制或 launcher 运行。
- 补丁在开发机 `/home/dong/rl_sar_b2_z1_VD2_front_local_20261106.zip`，
  板载 `/home/unitree/` 同名文件；SHA256
  `f01eb7683a14cef77f9b24f7a143297dce5d9e009d89c822d4f7bf17ed20e30e`。

完整证据目录：`outputs/VD2_FRONT_START_FAILURE_20261106_aBAG4t/`，含原会话、
修改前代码、板载安装前 MANIFEST、离线检查、板载复算 JSONL、MuJoCo 轨迹与汇总。
复现补丁：`deployment/vd2_real/tools/build_front_local_patch.py`。

## 现场下一步

```bash
cd /home/unitree/rl_sar_b2_z1_VD2
source .venv/bin/activate
bash start_real.sh --interface eth0
```

A 起立，站稳后 Y，到 ready 后按 R1+Y。`waiting for stable distinct geometry (n/3)`
表示等待新测量；达到 3 后才进入策略。再次 R1+Y 或 X 停止，B 沿原流程收臂趴下。

若仍无法连续行走，先读新会话真实退出原因，重点区分前视拒绝、真实帧年龄、后视
未覆盖工作点/缺深度。后视尚有安装外参和深度覆盖问题，不能靠改变前视阈值解决。
没有完整连续实机数据时，不宣称视觉行走或末端精度已经修好。
