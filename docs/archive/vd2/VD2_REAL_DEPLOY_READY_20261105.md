# VD2 实机部署更新：外参与初始保持目标

用户提供完整 `T_base_camera` 并要求准备部署。已确认前后相机在应用 `vd2_camera_fix_20261105` 后能预览；板载预检 `fresh:true`、`z1_mode:1`、电机错误为 0。后置外参来源 `rear_camera.json`，来源说明为近似安装外参，不视为精密标定。

正式更新：`/home/dong/rl_sar_b2_z1_VD2_deploy_ready_20261105.zip`，16,500 字节，SHA256 `3a527bddd870859c4db0f796b846c9ee24ccf3296fe08cb38a875dc548c686cc`。依赖前一个已装好的相机修复，无需重新编译 C++。中间验证生成的 `rl_sar_b2_z1_VD2_rear_extrinsics_20261105.zip` 仅是外参配置候选，尚不含下面的准备目标修复；现场使用 deploy_ready 包。

变更：

- 配置完整 optical→base 矩阵，保留 `calibration_quality: provisional_installation_estimate`、来源与已有相机序列号。`calibrated:true` 是现有程序接收完整外参的入口标志，不代表已证明测量精度。
- 用户实测 Z1 joint2=-0.003239 rad、joint3=+0.002053 rad，略超模型零位边界。原状态机会把测量值用作准备时的保持目标，被 native 层已有严格限位拒绝。新增 Python 下发前的目标处理，仅投影不超过 0.01 rad 的微小越界；较大越界仍拒绝。编码器、策略观测及原始趴姿记录不改写。日志记录修正前/后目标。
- 原文件与 MANIFEST 备份到 `patch_backups/rear_extrinsics_<时间>/`，同步更新改动文件哈希。不同的已存在程序/外参不会静默覆盖，重复安装无更改。

验证副本：`outputs/VD2_REAR_EXTRINSICS_20261105_6mR4ry/rl_sar_b2_z1_VD2`。从原 ZIP 重新解压，安装真实相机补丁、外参补丁及最终部署更新，复用原已验证 C++ 编译产物，`python run.py --offline-check` 25/25 通过。新增 2 项验证实测零位偏差导致 native 拒收及修复后接受、较大越界/非有限目标拒绝。没有接入或同步板载，没有初始化实机 IO。

安装与现场手柄指令见 [部署更新说明](../../../deployment/vd2_real/docs/REAR_EXTRINSICS_20261105.md)。接下来由用户手动传输安装，再离线检查、预检、现场 A 站起 / Y 准备验证。实机步态、停止恢复、真实 TCP 误差尚未验证。
