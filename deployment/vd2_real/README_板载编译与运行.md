# VD2 板载部署包（2026-11-05）

2026-11-09：前视转向改为最近可见拟合段内的目标，避免盲区外推造成反向纠偏；
前后视改为独立处理、任一路完成即可发布，分别保留原始采集时间和帧号。
单个网络推理仍互斥以控制 CPU 争抢，几何后处理互不等待。
没有放宽 1 秒时效，也不刷新沿用帧的时间。详见
[INDEPENDENT_VISION_20261109.md](docs/INDEPENDENT_VISION_20261109.md)。
本次仅 Python 更新，无需重编 C++；现场效果仍需验证，历史说明保留如下。

2026-11-06 最新本地候选：修正 Z1 指令排队、制动时机械臂目标跳变，
并减少前视对干扰分量的重复计算，见
[COMMAND_REFERENCE_FIX_20261106.md](docs/COMMAND_REFERENCE_FIX_20261106.md)。
**该候选尚未安装到板载，必须重编 C++ 后才能运行**；下文历史更新中的
“无需重编译”不适用于该候选。未改变 A/Y/R1+Y/X/B、站立增益、策略或帧时效。

2026-11-06 后续：R1+Y 后只晃一下机械臂的诊断与局部拟合修正见
[FRONT_LOCAL_RECOVERY_20261106.md](docs/FRONT_LOCAL_RECOVERY_20261106.md)。
现使用最近可见段二次曲线估计，替代下述旧三次曲线版本；启动前检查 3 个不同有效测量。

2026-11-06 新增前视路径估计板载版本，操作及验证边界见
[FRONT_PATH_BOARD_20261106.md](docs/FRONT_PATH_BOARD_20261106.md)。
本模式前 RGB＋IMU 重力＋配置高度独立估计路径，后 RGBD 提供末端目标，
不做机身运动估计；沿用 A、Y、R1+Y、B 和原站立/停止状态机。

压缩包按要求命名为 `rl_sar_b2_z1_VD2.zip`，内部策略是已在 IsaacLab / MuJoCo 选择的 **VD3 左右镜像约束续训 model_2000**，不是早期 VD2 权重。前进指令固定 **0.20 m/s**，转向来自前 RGB，后 D435i 的 RGB-D 焊缝测量生成末端目标。实机没有预设曲率；MuJoCo 的 0.4/m 只是仿真场景参数。

这是新的独立实机候选入口，已完成的本地验证见 `VALIDATION.md`。它没有继承 VA15 的动作 EMA、旧增益覆盖链或接触模型门控。原始 ZIP 的后置外参为空；之后用户提供完整的近似安装外参，并确认板载相机预览及 B2/Z1 预检通过。最新更新见 `docs/REAR_EXTRINSICS_20261105.md`，包含这组外参及实测零位偏差导致准备目标被拒的修复。实机运动和跟踪误差尚未验证。

2026-11-06 单终端入口已在板载启动到 idle，随后遇到反馈计时误退出。对应增量修复与安装命令见 `docs/FEEDBACK_FIX_20261106.md`：修正状态复制/时钟顺序及接管前连续等待计时，新增真实反馈年龄日志；不用重编译 C++。本地完整副本离线检查 38/38 通过，板载是否仍有真实断流待新日志确认。

原实机 GetUp 恢复已安装到板载，用户随后确认 A/B 没有问题、Y 展臂正常；旧位置补偿候选已撤下。用户确认现场没有枪和软管。

当前更新：首轮优化后现场已进入 tracking，但 0.6–0.8 s 的计算周期仍导致过期停车，后视也会选中未覆盖工作点的短段。新增 CPU 布局优化、后台存图、有测量支持的小深度空洞投影与工作点分量选择，见 `docs/VISION_REALTIME_20261106.md`；旧修复过程保留在 `docs/VISION_START_20261106.md`。无需重新编译 C++，尚未以现场行走验收。

## 1. 传输与解压

在当前开发电脑执行（只传输，不启动程序）：

```bash
scp /home/dong/rl_sar_b2_z1_VD2.zip unitree@192.168.123.164:/home/unitree/
```

在板载终端执行。使用新的独立目录，保留原 `rl_sar_b2_z1`：

```bash
cd /home/unitree
unzip -n rl_sar_b2_z1_VD2.zip
cd /home/unitree/rl_sar_b2_z1_VD2
uname -m
ip -br addr
```

SDK 库包含 `x86_64` 与 `aarch64`，CMake 按本机架构选取；只在本地 x86_64 编译验证过，ARM 需要在对应板载编译。本包不包含训练环境、IsaacLab、MuJoCo、旧运行日志或多 GB 的 LibTorch。

## 2. 依赖和编译

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev python3-venv python3-pip unzip
bash build_board.sh
```

输出为 `build/lib/libvd2_io.so`，以及本地测试程序；脚本最后自动执行 CTest。
**编译和 CTest 不接管机器人，不读取真实相机。** 测试使用内存回复及新建的本机回环 UDP 端口，不访问机器人网络或正式 8071/8072 端口。

Python 建议沿用板载此前能运行视觉的 Python 环境，先确认：

```bash
python3 -c 'import torch; print(torch.__version__); print(torch.cuda.is_available())'
```

如果上述命令正常，可以在该解释器上创建环境：

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -c 'import torch, cv2, numpy, yaml, pyrealsense2; print("Python dependencies OK")'
python run.py --offline-check
```

如果此前使用 Conda，请先激活此前的视觉环境，再用其中的 `python` 安装依赖、运行 `run.py`，不必额外创建 `.venv`。Python 支持 3.8–3.12。离线自检要实际加载策略和 RHANet 权重，验证金标准观测、70 维接口、编码器 FK、按键优先级、停止/趴下流程及标定变换。

若 `import torch` 失败，需要先安装与板载 CPU/GPU 匹配的 PyTorch。不要在 Jetson 上直接安装 x86_64/CUDA 轮子；此压缩包不附 Python、PyTorch 或系统依赖安装包，首次安装依赖需要板载网络或现有可用环境。本地验证环境是 PyTorch 2.4.1、Python 3.8。`pyrealsense2` 的 USB 权限继续使用板载已配置的 librealsense udev 规则。

## 3. 先看相机，不启动机器人控制

若旧包预览报 `unsupported D435i colour distortion model: distortion.inverse_brown_conrady`，安装相机增量修复，步骤见 `docs/CAMERA_FIX_20261105.md`。新代码使用设备实际模型生成 RGB-D 校正映射，标定工具也读取保存的模型；无需重新编译 C++。

下文用 `eth0` 举例，替换为 `ip -br addr` 中实际连接 B2 的网卡：

```bash
python run.py --camera-preview --interface eth0 --display
```

没有桌面时去掉 `--display`，或者增加 `--seconds 30` 自动结束。控制台给出 `runs/<时间>/`，其中保存 `front_latest.png`、`rear_latest.png`、`rear_raw_for_calibration.png`、`rear_depth_mm.png` 和 `rear_intrinsics.json`。此入口只调用前相机 RPC 与 RealSense，不创建 B2 低层指令发布者，不开启 Z1。

`config/rear_d435i.yaml` 的 `serial` 为空时只自动选择唯一的 D435 系列设备；若有多台，填写 D435i 序列号。D435i 使用 640×480、30 Hz 的 RGB 与深度，并对齐到 RGB、统一去畸变。前 RGB 沿用这台机器人 2026-09-15 的 1920×1080 标定；若前相机位置/分辨率变过，需要重新标定。

## 4. 后相机外参标定

准备 9×6 **内角点**、每格 25 mm 的棋盘格，放在后相机看得到的位置；机器人和棋盘保持静止，不需要开启低层运动程序。

需要实际测量棋盘格在 B2 `base_link` 中的位姿。`T_base_board` 是棋盘坐标到 B2 坐标的 4×4 刚体变换：B2 的 x 向前、y 向左、z 向上；棋盘原点为工具标出的第一个内角点，棋盘 x 沿每行角点，y 沿下一行。不能把棋盘“在地上”替代完整位姿测量，也不能把仿真安装角度填成实测值。

将实测矩阵保存为 `board_pose.json`，格式为 `{"T_base_board": [[...], [...], [...], [0,0,0,1]]}`；不提供虚构数值示例。相机预览运行时每 2 秒更新图片，结束预览后使用同一次静止采样：

```bash
python tools/calibrate_rear.py \
  --image runs/替换为实际目录/rear_raw_for_calibration.png \
  --intrinsics runs/替换为实际目录/rear_intrinsics.json \
  --board-pose board_pose.json \
  --columns 9 --rows 6 --square-m 0.025 \
  --output rear_calibrated.yaml
```

检查输出旁的 `rear_calibrated.corners.png` 中 `origin` 角点与实测棋盘原点一致；无编码棋盘可能有 180° 顺序歧义，错了要重新采样/确定角点顺序。工具计算 `T_base_camera = T_base_board × inverse(T_camera_board)`，拒绝重投影 RMS 大于 1 px 的结果。重投影误差小不代表棋盘的实测坐标准确；换一个已测位置重复一次，对比外参和地面焊缝位置后再安装配置。

```bash
cp -n config/rear_d435i.yaml config/rear_d435i.uncalibrated.yaml
cp rear_calibrated.yaml config/rear_d435i.yaml
```

第二条会用你刚检查的实测标定替换模板；标定文件与图像另行保留。不要只把 `calibrated` 改成 `true`。

## 5. Z1 服务与状态检查

已安装 `vd2_one_terminal_20261106` 更新时，直接在一个终端使用下面命令，自动启动/复用 `z1_ctrl` 并运行状态检查，详细说明见 `docs/ONE_TERMINAL_20261106.md`：

```bash
bash start_real.sh --preflight --interface eth0 --seconds 10
```

手动管理控制器时仍可使用以下方式：

使用板载原有的 `z1_controller`，不要同时开两个控制器或原来的 VA15/其他 `rt/lowcmd` 发布程序。若服务已经运行，不要重复启动。原目录和默认 SDK 模式没有改变时，在单独终端：

```bash
cd /home/unitree/z1_controller/build
./z1_ctrl
```

新入口默认连接本机 `127.0.0.1:8071`，自身 UDP 端口 `8072`，均在 `config/deployment.yaml` 中。它不是直接把 SDK UDP 发给 `192.168.123.110`；由板载 `z1_controller` 连接机械臂。旧程序如果已经占用 8072，要先正常退出它。本包携带 Z1 SDK，不覆盖板载原 `z1_controller` 及其机械臂负载参数。

在部署包目录、已激活 Python 环境的终端：

```bash
python run.py --preflight --interface eth0 --seconds 10
```

这个检查订阅 B2 状态，并与 Z1 交换 **PASSIVE** 请求；不释放 B2 原控制服务、不发送 B2 低层命令，但会将 Z1 请求为 PASSIVE。输出原始足端力、关节位置/速度/力矩、Z1 模式/错误与数据年龄，不把 `1011` 伪造成 `1111`。Z1 `PASSIVE=1`、`LOWCMD=16`，`fresh=true` 才代表两路状态实际有新包。Z1 UDP 只能验证 `z1_controller` 回复新鲜度；无法单靠它证明控制器内部电机报文的真实采样时间。

## 6. 实机操作入口

后置标定完成、相机预览与状态检查正常之后，现场启动：

```bash
cd /home/unitree/rl_sar_b2_z1_VD2
source .venv/bin/activate
bash start_real.sh --interface eth0
```

统一入口需要先安装 `vd2_one_terminal_20261106` 更新，会自动启动/复用 Z1 控制器；原始手动入口 `python run.py --run --interface eth0` 仍可用，但需要自行启动 `z1_ctrl`。

安装 `vd2_visual_windows_20261107` 后，统一入口默认打开前后视觉窗口。
请在板载已登录的桌面终端运行；纯 SSH 且没有 DISPLAY 时提示并继续无窗口运行。
画面显示检测线、帧年龄、耗时和实际控制状态/原因，上限 5 Hz，复用检测结果。
显示进程独立，拥塞丢弃显示帧；仍有少量 CPU 和内存开销。
`--no-display` 可关闭显示；Q/Esc/关窗口只隐藏画面，不停车。
详见 [VISUAL_WINDOWS_20261107.md](docs/VISUAL_WINDOWS_20261107.md)。

使用手柄操作。启动时应是静止趴姿；程序等到新按下 A，才释放 B2 原运动服务并接管。安装 legacy_getup 更新后，A：1 秒预备、2 秒起立、随后 2 秒站立增益过渡并等待实测稳定；Y：6 秒将机械臂送到策略初始位；R1+Y：开始视觉跟踪，再按切换停止；X：停止跟踪；B：先停止、回完整站姿，再收臂、回启动时记录的趴姿；L1+X：阻尼/PASSIVE。摇杆不生成行走指令。

停止时先给策略逐渐降到零的本体指令，保留末端目标；测得关节速度与 IMU 运动降低后，再插值回完整名义站姿，避免直接冻结后腿半蹲姿态。B 请求不会因视觉掉帧丢掉，也不依赖模型足端接触 `1111`。视觉长时间失效会锁存停止，恢复图像后需要再次 R1+Y，不自动重新行走。运行时使用一个 Z1 IO 线程，每次 LOWCMD 都重新填写原 SDK 原始增益，避免状态切换覆盖增益。输入断流/进程卡住由本地 IO 超时处理，不能替代现场急停。

站立与策略归一化参考分开：实机站姿为四髋 0、四大腿 0.8、小腿 -1.5 rad。legacy_getup 更新恢复原实机支撑：预备/起立使用髋/大腿 200/3、小腿 300/3，结束后从 Kd=8 平滑过渡到髋/大腿 Kp=450、小腿 Kp=675、Kd=15。行走策略使用训练的 160/5；进入策略和停止回站立时过渡目标与增益。Z1 沿用原部署 SDK 原始 `K_P=[20,30,30,20,15,10]`、`K_W=[2000,...]`，不把它们说成与仿真 160/5 相同的物理增益。训练枪为零附加质量，实机实际枪重仍需现场核对。

请在正常 B 趴下完成后再退出程序。直接 Ctrl+C 是阻尼/PASSIVE 退出，不是完整趴下动作，也不会自动恢复 B2 原厂运动服务。

## 7. 当前能力与日志

2026-11-06 后视局部支撑与机械臂通信取证更新见
[REAR_OBSERVED_AND_Z1_EVIDENCE_20261106.md](docs/REAR_OBSERVED_AND_Z1_EVIDENCE_20261106.md)。
后视按工作点附近的直接深度支撑判定；停车时保存对应原始 RGB-D 和失败包。
安装更新后可用 `sudo -v`，再运行 `bash start_real.sh --interface eth0 --capture-z1-network`，
同一个终端同时保存机械臂双向 UDP、带时间的控制器原始输出与主程序日志。
这里只对 tcpdump 提权。`z1_reply_age_ms` 仍只是本地控制器回复年龄，实际电机样本年龄未知。
若此前 sudo -v 后仍提示需要密码，终端会话修正见
[CAPTURE_SESSION_20261106.md](docs/CAPTURE_SESSION_20261106.md)；安装该更新后继续使用上述命令。

2026-11-06 的后续视觉耗时候选见 [VISION_DEADLINE_20261106.md](docs/VISION_DEADLINE_20261106.md)：
减少丢弃图像的解码和全图重复扫描，并限制数值库线程争用。离线完整负载复算
100/100 对有效，尚需现场验证真实时效；不更改帧年龄或动作状态机。

末端控制与刚才 MuJoCo 相同：后视分割焊缝与深度平面，生成基座系末端目标，actor 结合编码器执行。**没有视觉识别实际 TCP，也没有新增 IK/精准枪尖视觉闭环**；不能据此宣称已经满足实机地面投影误差 5 cm。真实负载、Z1 增益单位与隐式仿真执行器仍有差异，实机需用目标/编码器响应评估，不能套用 MuJoCo 的关节速度冲量约束。

前后相机保留各自接收时间、设备时间和帧号。前视估计模式使用 `independent_latest`，不要求两路同序号或收包时间差小于 40 ms；每路仍独立检查 1 秒时效。未启用前视估计的旧模式仍使用 40 ms 主机收包时间差门槛，不是已证明的曝光同步。B2 前相机 SDK 不提供曝光时间，程序不会编造设备时间或用推理时间刷新旧帧。没有全局位姿、线速度真值、已知裂缝曲线、模拟接触或未来轨迹进入 actor。

每次日志在 `runs/<时间>/`，包括压缩的 `control.jsonl.gz`（70 维观测、原始动作、目标角、编码器、估计力矩、模式/按键、原始足端力）、`vision.jsonl`（独立两路时效与识别结果）、最近相机图像和运行参数。后续检查实际末端响应、停止姿态或跟踪误差，用这些真实记录定位。
