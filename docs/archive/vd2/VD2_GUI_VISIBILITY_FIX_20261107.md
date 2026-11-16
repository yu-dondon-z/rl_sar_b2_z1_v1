# 板载实时视觉窗口提前退出修复

2026-11-07 已修复并同步到 `/home/unitree/rl_sar_b2_z1_VD2`。

根因已在板载 GTK3 桌面实际复现：成功创建并连续 imshow/waitKey 绘制窗口时，`getWindowProperty(WND_PROP_VISIBLE)` 返回 -1。旧代码用 `<1` 判断关闭，在首个循环就正常退出。该返回值不能表示用户关闭窗口。

修改 `python/visual_preview.py`：移除此属性退出条件，持续显示已有相机处理结果；只有 Q/Esc 或主程序关闭结束显示。关闭窗口装饰按钮可能在下一帧重新创建，主动隐藏请按 Q/Esc。没有修改模型、视觉几何、工作点、控制增益或冻结处理。

板载实际桌面 `DISPLAY=:1001`、当前登录用户 XAUTHORITY 下，用明确标注 DISPLAY TEST 的合成画面绘制两个窗口，共20个循环成功；两个可见性属性均返回-1，确认旧代码误判。测试未打开相机、SDK或控制程序。117项离线回归通过。实际相机数据持续显示仍由下次用户启动现有程序观察。

备份：`/home/unitree/rl_sar_b2_z1_VD2/patch_backups/gui_visibility_20261107_143035_202816`。
板载测试日志：`/home/unitree/vd2_gui_fix_k5r1lt7f/offline_check.log`。
补丁：`/home/unitree/vd2_gui_visibility_fix_20261107.zip`，SHA256 `7c7feb66cfc07d100c6b20ee440c6951867e7738fbdf492ea5e830a83edd678d`。

用户从板载桌面终端运行 `bash start_real.sh --interface eth0 --display`，窗口随运行持续刷新；不需要重新编译。SSH 无 DISPLAY 时仍明确提示不可打开桌面，不猜测显示环境。

之前关于缩短工作距离的讨论未产生代码变更。1.02米工作点和策略训练范围仍保持原样；地面法向与IMU存在约15度差异、有效焊缝深度断段的问题仍需站姿对应的实际图像确认，不能用趴姿末帧代替。
