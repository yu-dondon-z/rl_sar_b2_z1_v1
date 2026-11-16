# 相机异常配对恢复同步记录

已按授权同步到 unitree@192.168.123.164 的
`/home/unitree/rl_sar_b2_z1_VD2`，未启动相机、控制或机器人运动。

修复文件：python/independent_vision.py；增加两项回归测试和修复说明。
无效后视配对不再终止双路处理；发布无效结果及真实时钟诊断、空几何，
让窗口显示原图并继续等待后续配对。不改 40 ms、1 s 门槛、策略或增益。

补丁：`/home/dong/rl_sar_b2_z1_VD2_camera_pair_recovery_20261109.zip`
SHA256：`8cd526e8521b1dd3d8057aab8b6f8a24f90a3e972c76039557a961911a4af3e1`
板载备份：
`/home/unitree/rl_sar_b2_z1_VD2/patch_backups/camera_pair_recovery_20261109_105911_558415`

完整离线检查：本地 crack_vision 环境 146 项通过，板载 .venv 146 项通过。
本地 base 环境缺 pyrealsense2 的初次失败日志也保留，未忽略失败后宣称通过。
证据：outputs/CAMERA_PAIR_RECOVERY_A4AF5K/。

实机视频采集尚未验证；若仍持续拒绝配对，按 vision.jsonl 新增实际时钟域、
设备时间和时间差字段继续查因，不能降低门槛当作正常图像。
