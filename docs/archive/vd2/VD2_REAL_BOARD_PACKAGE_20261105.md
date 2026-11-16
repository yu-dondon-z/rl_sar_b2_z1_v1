# VD2 实机交付包

交付文件：`/home/dong/rl_sar_b2_z1_VD2.zip`，31,797,670 字节（30.32 MiB）。

SHA256：`10c63bc7c729881aa0cecae59727a2fa098b208bf05f2f04f3ac782618ecca16`。

包名按用户要求为 VD2，策略实际为当前已选的 VD3 左右镜像约束 model_2000，actor SHA256 为 `51eda90b2b77204ef66d949523486250e88c7062aeb1e86f0dc5478924291c78`，前进指令 0.20 m/s，70 维观测 / 18 维原始动作，0.25 缩放。

目标板载仍为 `unitree@192.168.123.164`。用户确认后 D435i 已安装、尚未标定；配置显式保持未标定，不以仿真外参替代。准备工作全在开发电脑完成，没有远程连接、上传、运行控制或机器人运动。

## 目录和说明

- 实机候选源代码：`deployment/vd2_real/`，独立入口，不改旧 VA15 或当前 MuJoCo 入口。
- 完整中文操作：`deployment/vd2_real/README_板载编译与运行.md`。
- 最终打包目录：`outputs/VD2_REAL_BUNDLE_20261105_final/rl_sar_b2_z1_VD2/`。
- 本地构建记录：`outputs/VD2_REAL_BUNDLE_20261105_final/build.log`。
- 本地离线记录：`outputs/VD2_REAL_BUNDLE_20261105_final/offline_check.log`。
- 本地编译产物不进入 ZIP；包内包括完整构建源代码、两种架构 SDK、actor/RHANet 权重、FK URDF、配置、标定工具及测试。

## 实现范围

新的 C++ 入口负责 500 Hz B2 / Z1 唯一 IO；Python 负责 50 Hz 策略和手柄状态机，视觉在独立进程。保留原实机 B2 `0x0A` 伺服模式、SDK 原始 Z1 增益和 PASSIVE/LOWCMD 状态；内存回复测试核对实际 SDK 序列化。实机对称站姿与训练默认关节角分开，进入行走与停止回站立时过渡。足端力只保留原始记录，不伪造接触，停止到位条件使用测得关节速度、IMU 运动和编码器到位。

前 RGB 使用本机原标定；后 D435i 对齐 RGB/深度、统一去畸变、保留真实接收/设备时钟。没有全局位姿、模拟接触、已知裂缝曲线、线速度真值进入 actor。后视仍为 `rear_seam_direct`，`visual_tcp_feedback=false`，不是新实现的枪尖视觉闭环。

`--offline-check` 不开真实传输；`--camera-preview` 只开相机；`--preflight` 订阅 B2 并向 Z1 交换 PASSIVE；`--run` 完成标定检查，等待 A 才接管 B2。手柄 A 站起、Y 准备臂、R1+Y 跟踪/停止、X 停止、B 停止后收臂趴下、L1+X 阻尼/PASSIVE。正常退出前先用 B 趴下，Ctrl+C 为阻尼退出而非完整趴下。

## 验证结果

在新的最终目录执行本机源码构建成功，C++ CTest 2/2、Python 离线 17/17 通过。包括 9 组既有成功 MuJoCo 记录的观测/动作对比、RHANet 实权重图像推理、C++/ctypes ABI、停止/B 请求保留、棋盘格外参变换和未标定前置拒绝。动态链接确认采用随包 DDS，无额外 ROS 依赖。ZIP CRC 通过，871 个清单文件逐一 SHA256 一致，共 872 个条目。

未验证：板载本机编译和运行、相机实际延迟、后相机实测外参、真实足底支撑/停止/趴下、Z1 真实增益和时延响应、实际枪重、TCP 地面投影误差。这是实机候选交付，不把本地软件测试称为实机成功。
