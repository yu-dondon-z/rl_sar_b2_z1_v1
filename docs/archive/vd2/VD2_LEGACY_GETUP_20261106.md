# VD2 实机起立恢复进度 — 2026-11-06

> 最新：用户明确授权直连操作后，已于 10:47 完成板载传包、安装、重新编译和离线检查（2/2 CTest、50/50 Python）。旧源码/build 已备份，未启动控制或运动。详见 [板载部署记录](../../../outputs/VD2_LEGACY_GETUP_BOARD_20261106_syC4nG/BOARD_SYNC_REPORT.md)。下文“未同步”描述的是此前本地阶段。

用户反馈上一版按 A 约 1 秒前摔，已经停程序并关机；确认无枪/软管。已撤下站立位置偏置候选，以 `patches/va15_tcp_priority_v256_20261028/production/fsm_b2_z1_no_gun.hpp` 和保存的实机 `outputs/diagnosis_v256_20261029_100719/board_config.yaml` 为参考恢复起立。不能只参考当前主目录基础 FSM，因为它不含完整实机站立增益覆盖。

实现与板载操作详见 [更新说明](../../../deployment/vd2_real/docs/LEGACY_GETUP_20261106.md)。完整证据见 [验证报告](../../../outputs/VD2_LEGACY_GETUP_20261106_qXOhkd/verification.md)。原候选源文件/压缩包及全部历史日志均保留；没有远程同步或发起控制。

恢复 1 s 线性收腿预备、2 s 起立、2 s 渐变到原髋/大腿 Kp=450、小腿 Kp=675、Kd=15。A 保持收臂、Y 才展开，保留半站 B 回趴修复。接管后重读真实编码器；命令 ABI 增加明确腿增益，必须重新编译 C++，旧库会在控制初始化前被拒绝。

50/50 Python 离线检查与 2/2 CTest 通过。完整无枪自由机身趴姿起立对照最大倾斜从上一版 42.44° 降到 1.99°；50 s 发 B、55 s 回 idle。0.5/2/4 s 三个 B 中断也完成物理对照。这些都不能代替实机验收。

已准备 `/home/dong/rl_sar_b2_z1_VD2_legacy_getup_20261106.zip`，49180 bytes；SHA256 `24c11f1f7a34af78b28b4b31cbf01aef4214b2892ba4c6a7153dba35e4fb5a9f`。以已安装 stand_recovery 的程序为基线，安装器保存旧文件且不会覆盖额外本地修改、相机外参或日志。
