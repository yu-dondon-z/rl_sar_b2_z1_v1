# D435i inverse Brown-Conrady 兼容性修复

现场日志：`20261105_190225_277052`、`20261105_190247_396141`。网卡 `eth0`，地址 `192.168.123.164/24`。后相机启动被 `unsupported D435i colour distortion model: distortion.inverse_brown_conrady` 拒绝；这不是 USB 断连或未安装 Python 依赖。

原包错误地只允许 none / brown / modified_brown，且将 modified_brown 也按普通 OpenCV Brown 系数处理。修复按设备实际返回的模型，通过安装的 RealSense SDK `rs2_project_point_to_pixel` 建立“输出无畸变像素 → 原始 RGB 像素”查找表。查找表仅在流启动时生成，RGB 与已对齐深度使用同一映射；零深度继续保留为 NaN。

`rear_intrinsics.json` 新增原始传感器 `distortion_model`；`rear_raw_for_calibration.png` 仍保留原始图像。标定工具读取模型并先校正原始图像，再在无畸变像素上解 PnP。不能把逆模型系数直接当成 OpenCV Brown 系数，也不能只在允许列表里加一个枚举。

相机子进程由主进程处理 Ctrl+C 后通知退出；预览退出提示明确说明没有打开机器人控制。策略、增益、本体运动指令、相机外参、原始时间戳规则均未更改。

官方依据：

- [RealSense 投影模型说明](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0)
- [SDK v2.54.2 投影/反投影实现](https://github.com/realsenseai/librealsense/blob/v2.54.2/src/rs.cpp)

## 在现有板载目录安装

先退出相机预览。在开发电脑传输新补丁，不需要重新传输模型或整个包：

```bash
scp /home/dong/rl_sar_b2_z1_VD2_camera_fix_20261105.zip unitree@192.168.123.164:/home/unitree/
```

板载执行：

```bash
cd /home/unitree
unzip -n rl_sar_b2_z1_VD2_camera_fix_20261105.zip
cd /home/unitree/rl_sar_b2_z1_VD2
source .venv/bin/activate
python ../vd2_camera_fix_20261105/install_patch.py --target .
python run.py --offline-check
python run.py --camera-preview --interface eth0 --display
```

无需重新编译 C++。安装前验证更新文件和目标原文件哈希；如遇其他本地修改，拒绝覆盖。原文件及原清单保存在 `patch_backups/camera_fix_<时间>/`，相机配置和已有运行记录不覆盖。补丁同步更新 `MANIFEST.json` 中变更文件的哈希，原有模型哈希验证继续生效。重复安装无变化时直接退出。

没有桌面时，将最后一行换为：

```bash
python run.py --camera-preview --interface eth0 --seconds 30
```

预览正常时会打印 `camera preview; rear calibrated=False`，并显示 `rear_distortion_model=distortion.inverse_brown_conrady`。`calibrated=False` 在预览阶段正常，不能凭此将外参标志改成 true。

## 验证范围

新增 6 项离线用例：零系数逆模型恒等映射；非零 Brown / inverse 的 SDK 反投影回环；inverse / modified 与普通 OpenCV Brown 的区别；原始图像按记录的模型校正后用于标定；未知/非法内参拒收；模拟真实相机接口跑通后视采样，确认 RGB-D、无效深度、原始帧号和设备时间戳保持一致。

这些用例不需要相机，也不创建机器人 IO。仍需现场重新预览，确认设备实际出图；本地没有验证实机跟踪或外参精度。
