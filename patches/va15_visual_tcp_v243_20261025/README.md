# VA15 V2.4.3：起步准备、停止诊断与视觉实验准备

2026-10-25 独立候选，保留 V2.4.2 和主程序。同步/板载自检状态见 BOARD_SYNC_REPORT.md；本地验证见 PREPARATION_REPORT.md；本次实机无视觉记录见 REAL_BLIND_REVIEW.md。无视觉短程启停已核对，可准备人工视觉实验；不是视觉跟踪、硬件或作业精度验收，不自动起立/行走。

## 本版变化

- 使用同一 VA15 model_5000、70 维观测/18 维动作、alpha=0.2/raw 历史。训练增益、主 YAML、关节路由、足端阈值/极性均不改。
- 机械臂在原有 Y 待机准备阶段提前平滑到 [0,2.24,-1.27,-1.33,0.04,0] rad，减少 R1+Y 后再进行约 19/25 度肘腕调整。移动后仍由同一个全身策略控制 18 个关节，不增加 IK 覆盖或手工机械臂轨迹。
- 保留 V2.4.2 的首帧力矩连续接管，不延长腿部补偿，也不提高全程增益。
- B 支撑捕获、最终四足/高度/速度/姿态/安静时间条件不变；新增每项失败位、接触状态、实际 gyro/quat、quiet_s 等记录。上次首次 B 等待的具体根因尚未确认，不能把新增诊断当成修复已验收。
- 视觉轨迹先发布，再保存诊断图片；图片写盘采用有界后台队列、最多每秒一次覆盖图，积压时丢诊断任务，不阻塞路径发布。后台错误/丢任务会计数，不授予或取消路径资格。
- 400 ms 处理预算、500 ms 图像年龄、首次六帧、同线关联、错误会话/乱序/失线停止均保留；没有把过期图像当新图像，没有自动重新启动。
- 相机仍为已标定的 B2 自带光学 RGB + RHACrackNet，不是 RGB-D。

## 板载人工短验证

只有 BOARD_SYNC_REPORT.md 确认本板构建和指纹校验通过后，在 PC4 的交互终端执行。
保持实际趴稳、旧控制退出、没有焊缝枪/软管、收臂/运动空间清空，并准备可靠防跌和遥控急停。脚本会处理本地 Z1 服务，不需要第二个 Z1 窗口。

无视觉固定 TCP 对照：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_tcp_v243_20261025/run_blind.sh eth0
```

输入大写 RUN → A 起立 → 实际站稳 → Y 准备机械臂/待机 → 等待准备完成 → R1+Y 开始固定 0.15 m/s 命令，y=yaw=0；不推摇杆。
首次只观察约 2～3 秒，X 请求停止，B 请求趴下。实际趴稳且 Passive 后再 Ctrl+C；不能用断电/强退替代站立时的安全停止。
若 B 等待，请保留 [Stop gate V2.4.3] 行，不重复触发行走。fail_mask 是下面各项的按位或：1 来源无效，2 接触未知，4 非四足，8 高度，16 速度，32 倾角，64 gyro，128 关节速度，256 关节漂移，512 源过期，1024 配置无效。这些是诊断，不是额外放行条件。

## 视觉准备（暂不行走）

只看相机，不接管 B2/Z1：

```bash
bash diagnostics/va15_visual_tcp_v243_20261025/run_visual.sh eth0 --preview
```

如需在同一套程序内人工 A/Y 站立观察识别，可用下列 SHADOW 模式。它仍需大写 RUN 并接管站立控制，但始终不放行自动行走：

```bash
bash diagnostics/va15_visual_tcp_v243_20261025/run_visual.sh eth0 --shadow
```

使用 NoMachine 桌面终端（有 DISPLAY）；趴着看不到地面线可能正常。A 站稳、Y 准备后确认黄/青点落在预定胶带而非瓷砖缝；保持不推杆。B 趴下，实际趴稳后 Ctrl+C。相机会随该会话退出，不需要窗口 R。
单纯 preview 用 Q 或 Ctrl+C 关闭；不能在控制模式中用关闭相机窗口代替停止机器人。

## 下一步：人工视觉行走实验

已核对本版无视觉会话 va15_blind_v243_e5tN0O：约 13.6 秒有前进命令，没有安全故障或接管失败；X 后完成支撑捕获、0.5 秒四足安静确认，随后 B 正常 GetDown → Passive，进程 exit=0。操作者确认 X/B 正常、实际趴稳并退出、无枪/软管。本次记录不代表以往所有 B 等待根因已解决，也没有验证真实视觉/TCP 跟踪精度。

保持本版步态/接管代码、原 VA15 权重及相机标定不变，只允许已有 --walk 入口进入人工实验。在 PC4 的 **NoMachine 桌面交互终端**执行（相机和控制同一个入口，Z1 自动处理）：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_tcp_v243_20261025/run_visual.sh eth0 --walk
```

1. 地面平坦，使用较长、连续且易与瓷砖缝区分的黑色胶带；无枪/软管，机器人实际趴稳、旧控制已退出，收臂/运动区域清空，防跌与急停准备就绪。输入大写 RUN。
2. A 起立，实际站稳；Y 进入待机，等机械臂准备完成。不推摇杆。
3. 先在**站稳后的图像**确认黄/青轨迹点落在预定胶带上，连续显示 PATH IN RANGE，且不是瓷砖缝。显示就绪不代表操作者可以忽略线选择或实际姿态；若识别不对，保持不行走并保留图像。
4. R1+Y 开始：前进命令 0.15 m/s，转向由视觉给出，yaw 限幅 ±0.06 rad/s。本次先走约 2～3 秒，由人按 X 停止，检查是否沿胶带及是否重新抖动；正常后再做较长一次，观察 TCP 日志 active=1。
5. 前相机尚未积累到后方 TCP 所在位置的历史轨迹时，TCP 保持原目标，active=0 是等待历史，不伪造后方路径。active=1 后，目标才按已有速度界平滑接入同一全身策略。目标仍使用训练的约 9 cm 离地高度，不改成 1 cm，也不要求未安装的工具实际接地。
6. 再按 R1+Y 或 X 可停止；B 请求支撑停止后趴下。**实际趴稳且 Passive 后 Ctrl+C**，当前会话相机会一并退出。失线会停止，不会因为恢复识别自行重启；完成支撑停止后须明确再按 R1+Y。没有新增窗口 R、额外第二终端或四秒强制结束。

此放行仅表示可由操作者开始受控实验，不表示真实视觉/TCP、工具负载、长距离直线和曲线精度已通过。足端/停止/通信约束未放宽；若再次无法完成 B，请停止后续行走并保留 Stop gate 日志，不能用站立断电代替停止。

## 本地复现

    cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build build -j2
    ctest --test-dir build --output-on-failure
    /home/dong/anaconda3/envs/crack_vision/bin/python -m unittest test_tracker test_long_tracker test_motion_tracker test_session test_v24_camera test_camera_diagnostics
    /home/dong/anaconda3/envs/crack_vision/bin/python check_arm_entry.py
    python3 run_handoff_suite.py 新结果目录

几何检查只适用于现有模型、无枪条件；真实电机/Z1 内环、标定、共同打滑、工具负载与作业精度尚未验收。原 OFFLINE_REPORT/HANDOFF_REPORT/HOTFIX_REPORT 是继承的历史记录，本版状态只看本 README、PREPARATION_REPORT、BOARD_SYNC_REPORT 和 qualification.json。
