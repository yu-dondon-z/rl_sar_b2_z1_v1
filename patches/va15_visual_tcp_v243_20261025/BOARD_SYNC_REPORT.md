# V2.4.3 板载同步记录（2026-10-25）

当前状态：用户完成本版无视觉短走，日志确认 X 停止、B 趴下和正常退出；已重新生成板载 bundle，允许下一次人工视觉实验。真实视觉/TCP 跟踪尚未验收；本轮工具没有启动任何机器人动作。最新状态见文末“无视觉实测后的视觉实验准备”，下面保留初次同步过程。
用户确认 B2 趴稳、控制程序退出、无枪/软管、Z1 收臂路径无遮挡。只在独立 diagnostics/va15_visual_tcp_v243_20261025 目录同步、编译和离线检查；不自动起立、接管或行走，不覆盖 V2.4.2、主程序、策略和基础配置。

初次同步时本地结果见 PREPARATION_REPORT.md；硬件/精度资格保持 false，当时视觉 --walk 尚未放行。以下初次指纹保留以关联实机无视觉日志；文末记录最新 bundle。

## 已核对

- 目录 /home/unitree/rl_sar_b2_z1/diagnostics/va15_visual_tcp_v243_20261025，主机 unitree-b2-pc4，接口 eth0。
- 板载 native --offline-check 通过，VA15 黄金样本最大误差 1.14441e-05，无低层控制初始化。
- 本板单独编译运行 test_blind、test_handoff、test_stop_diagnostics、test_stop_recovery、test_tcp、test_telemetry，6/6 通过；Python 测试 29/29。
- 主二进制保持 696756deac43676046b0073d6c7ee0c0eb1a79f0248f678f9df337013f8d9ba3；base.yaml、VA15/config.yaml 与权重均未改。
- 权重保持 8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098。
- 新 bundle SHA-256：c0e8c2850e301c5e0249d27ed6f06fd8602bd5d04c2847b0eb79fbce5b929728。
- 新独立二进制 SHA-256：131bf69a0e4c4e07ce1169de38ab15ceef7ddd4757f6c15e9dc31726e80b104b。

## 纯相机检查（没有控制 B2/Z1）

仅运行本版 --preview --headless 约 18 秒；它不运行 run_control、不释放厂商控制、不发布 B2 lowcmd、不启动 Z1。已结束并确认没有遗留相机或实机控制进程。
会话 outputs/va15_near_preview_gy3e3h47：收到 1920x1080 图像，末序号 58，末帧采收到发布约 238 ms，诊断队列丢弃计数 0，raw/overlay 均成功保存。
末帧 valid=false，原因 No continuous, sufficiently wide dark near tape；这是趴卧、当前场景下的结果，不能据此宣称站立后的胶带识别已通过。需由操作者站稳后检查正确胶带/裂缝，而非瓷砖缝。
超时工具的退出码 124 是预定结束纯预览，不是控制程序崩溃。仅请求了相机 RPC；没有实机动作。

## 下一步人工操作

在 PC4 交互终端、无枪/软管、运动空间清空及防跌/急停准备好后：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_tcp_v243_20261025/run_blind.sh eth0
```

大写 RUN → A → 实际站稳 → Y。Y 后机械臂会缓慢提前调整到行走工作姿态，等待它停稳/准备完成，再 R1+Y 只走约 2～3 秒，X 停止、B 趴下。实际趴稳且 Passive 后 Ctrl+C。脚本处理 Z1，不需另开窗口。
如 B 等待，不重复触发行走，保留 [Stop gate V2.4.3] 日志；不要在站立时强退/断电。掩码说明见 README。

需要静态看视觉时可用 run_visual.sh eth0 --shadow，同样需 RUN/A/Y，但不放行行走；或者只用 --preview 看相机，不接管机器人。使用 NoMachine 桌面终端显示窗口。
确认本版短程启动与 X/B 停止正常后，再准备正式视觉行走。当前不宣称起步晃动、首次 B 等待或视觉精度已在实机解决。

## 无视觉实测后的视觉实验准备（2026-10-25）

实测会话 outputs/va15_blind_v243_e5tN0O 与初次 bundle 匹配。约 13.6 秒非零前进命令，无 safety_fault/torque_handoff_failed；X 后正常 Decelerate → CatchStand → Settle → ReadyToLower，B 后 GetDown → Passive，exit=0。用户确认实际趴稳、控制已退出、无枪/软管。详见 REAL_BLIND_REVIEW.md。这不是视觉跟踪验收。

只更新独立候选的 qualification.json/README 和说明；production 源文件全部与仿真验证 SHA 相符，编译后二进制 SHA 与本次实测相同。没有更换策略、控制增益、raw/EMA 历史、标定或接触门槛。没有自动接管、起立、行走或启动 Z1 服务。

- 新 bundle：929dce5e744b8f4e0668abe1a45952cca927693c0c8e8d5c700b6520e85450d4。
- 独立二进制仍为：131bf69a0e4c4e07ce1169de38ab15ceef7ddd4757f6c15e9dc31726e80b104b。
- 主二进制仍为：696756deac43676046b0073d6c7ee0c0eb1a79f0248f678f9df337013f8d9ba3。
- base.yaml、VA15/config.yaml、policy.pt SHA 与初次同步及实测保持一致。
- 旧 native_build_visual 已保留为 native_build_visual_blind_only_20261025；旧 README/qualification 保留在 release_backup_blind_only_20261025。旧 bundle 是当时输入快照，不应拿它跳过当前入口检查；回退需成套还原已记录输入并离线复核。
- 本地 C++ 回归重新 13/13，Python 重新 29/29；板载新构建、native golden（误差 1.14441e-05）、run_visual.sh --offline-check 均通过。所有这些检查不初始化机器人接口。
- walk_release_allowed=true 仅用于人工无枪平地实验；hardware_qualified、precision_qualified、visual_tracking_hardware_acceptance 仍为 false。

PC4 NoMachine 交互终端：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_tcp_v243_20261025/run_visual.sh eth0 --walk
```

大写 RUN → A 站稳 → Y 准备 → 确认相机黄/青点跟随预定胶带、连续 PATH IN RANGE，而不是瓷砖缝 → R1+Y 先短走约 2～3 秒 → X 停止 → B 趴下 → 实际趴稳且 Passive 后 Ctrl+C。相机与控制同入口，Z1 由既有脚本处理，不需额外终端或按 R。反复行走必须明确再按 R1+Y，不自动恢复。

若短走正常，再考虑稍长一次以积累前相机到后方 TCP 的真实观测历史；开始 active=0/保持原目标是正常等待，日志 active=1 才代表已有历史目标接入。它仍是单目平地投影、实验性无滑移估计，不是深度相机或绝对定位；未安装枪时没有真实工具接地任务。
