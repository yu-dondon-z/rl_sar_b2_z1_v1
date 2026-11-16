# VD2 板载 D435i 畸变模型修复记录

用户提供板载 `20261105_190225_277052`、`20261105_190247_396141` 日志：`eth0=192.168.123.164/24`，前视已取到图像，后置相机因 `distortion.inverse_brown_conrady` 被旧包拒绝。Ctrl+C 时子进程的 `KeyboardInterrupt` 及“发送 damping/PASSIVE”提示在只预览场景造成误解。

本地修复在 `deployment/vd2_real/`，仅调整相机几何、标定图像处理和退出提示，新增真实 SDK 几何用例。查找表调用安装的 RealSense SDK 投影函数并在启动时缓存，RGB/对齐深度共用；原始时间戳和帧号不改写。保存原始畸变模型，PnP 前据此校正图像，避免将逆模型系数当成普通 OpenCV Brown 系数。

增量包：`/home/dong/rl_sar_b2_z1_VD2_camera_fix_20261105.zip`，21,315 字节，SHA256 `25095678448fcd37835e2e31efa292861339833b3b7c39f0ff9706e8c7d8f29c`。原始 `/home/dong/rl_sar_b2_z1_VD2.zip` 保留不变。补丁不含策略、权重、C++ 或相机配置；无需重新编译。

验证：将原 ZIP 重新解压到 `outputs/VD2_CAMERA_FIX_20261105_Jvy5u9/` 后，安装实际补丁 ZIP；复用原始已验证的 C++ 编译产物执行 `python run.py --offline-check`，23/23 通过，包括原有 17 项与新增 6 项。重复安装返回“already installed”，没有重复备份。真实相机未连接，未同步板载或启动机器人控制。

安装器先核对目标原文件及补丁文件 SHA256；如本地文件有其他修改则拒绝覆盖。原文件和原清单保存在 `patch_backups/camera_fix_<时间>/`。更新 `MANIFEST.json` 对应条目，保留其余文件（包括模型）哈希。

用户操作步骤见 [补丁说明](../../../deployment/vd2_real/docs/CAMERA_FIX_20261105.md)。下一步是用户手动传输补丁、安装、离线检查、重新相机预览。后置相机完整外参标定仍待现场完成，本补丁不将 `calibrated` 改为 true。
