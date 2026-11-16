# V2.4.8 验证与同步记录（2026-10-26）

独立目录：diagnostics/va15_visual_reflection_v248_20261026。

本地已完成 62 项 Python 测试、25 张本次真实图像与 41 张历史图像回放、66 包原生几何校验。production 目录与 V2.4.7 字节一致，原有 16 项 C++ 回归再次通过。

本次真实图像 17/25 → 23/25，6 张导致停车的拒绝图全部恢复，旧有效图无退化；历史图仍 39/41，无退化。仅保存图片回放，不是完整实时会话，也不是实机成功率。

没有改变权重、增益、足端阈值、500 ms 图像时效、受控停止或终点判断。没有新加按键、静态检查、短距离或单次行走限制。

## 板载已完成

已同步到 /home/unitree/rl_sar_b2_z1/diagnostics/va15_visual_reflection_v248_20261026，保留 V2.4.7 及主程序，不替换默认入口。

- PC4 62/62 Python 测试通过。
- 在 PC4 对本次 25 张保存图片再次推理，17 → 23 张有效，要求恢复的图片全部通过，原有效图片无退化。
- PC4 原生解析校验两组共 66 包，62 个有效、4 个仍无效，与 Python 的决定全部一致；原有坐标、锚点、路径长度、包大小和时效格式没有放宽。
- PC4 本次图像新后处理耗时中位数约 9.68 ms、最大约 14.67 ms；仅保存图片的后处理耗时，不是实时端到端延迟验收。
- 独立主程序编译、权重/配置指纹、原生 CPU golden 检查通过，golden 最大误差 1.14441e-05。只运行 --offline-check，不初始化 DDS/UDP/机器人控制。

新 bundle SHA-256：3c515ecdb9d06d356692871281a51146726a8762517795149da8ff684555f674。

独立 va15_forward_trial SHA-256：c4f3feaa9ddf2884b9149eb453c46e7a111c283ff787c4474bb2007c2491951e。

既有 cmake_build_va15 主程序仍为 696756deac43676046b0073d6c7ee0c0eb1a79f0248f678f9df337013f8d9ba3，VA15 权重仍为 8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098，base.yaml 和 VA15/config.yaml 均未修改。

## 人工测试入口

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_reflection_v248_20261026/run_visual.sh eth0 --walk
```

大写 RUN → A → 实际站稳后 Y → 机械臂与支撑就绪、PATH IN RANGE 对应正确胶带 → 一次 R1+Y。保持无枪/软管并人工看护；X 停止、B 趴下，实际趴稳且 Passive 后 Ctrl+C。没有窗口 R 或新静态检查，没有固定短走时限。窗口 recovered 数字表示本帧由 RGB/网络联合恢复的扫描行数。

hardware_qualified 和 precision_qualified 仍 false。没有启动相机、Z1 服务、控制或实机行走，未释放厂商控制。原有 19 组物理结果只作为字节相同控制内核的继承证据，不宣称新视觉已完成实时/实机验收。当前仍需人工验证反光场景的连续行走。
