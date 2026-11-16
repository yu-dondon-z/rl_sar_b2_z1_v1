# V2.4.2 板载同步与离线验证（2026-10-23）

状态：独立源码同步、板载构建与离线检查通过。只准备人工无视觉、固定 TCP 对照；没有自动启动控制，没有实机运动验收。

用户已确认 B2 实际趴稳、控制退出、没有焊缝枪/软管，Z1 收臂路径无遮挡。检查未发现实机控制/相机进程。本轮只同步文件、编译、运行 --offline-check 和内存单元测试，不自动初始化 DDS/UDP、接管 B2、启动 Z1、起立或行走。

## 目录与保留范围

- 主机 unitree@192.168.123.164（unitree-b2-pc4），接口 eth0。
- 新目录 /home/unitree/rl_sar_b2_z1/diagnostics/va15_visual_tcp_v242_20261023。
- V2.4.1、主程序、基础配置和 VA15 权重不覆盖；不复制开发电脑的可执行文件到板载。
- 本版只准备无视觉、固定 TCP 的人工对照；视觉行走仍未放行。

同步前主二进制 SHA-256：696756deac43676046b0073d6c7ee0c0eb1a79f0248f678f9df337013f8d9ba3。
VA15 权重 SHA-256：8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098。

最终 native_build_visual/bundle.json 将绑定该板实际源文件、模型、配置和新二进制。构建成功仍不等于实机运动验收，硬件和精度资格保持 false。

## 板载已完成的检查

- 独立构建成功；实际运行 native --offline-check，没有 DDS/UDP/机器人初始化。
- VA15 70 维观测、18 维动作与黄金样本通过，最大误差 1.14441e-05。
- 本板离线内存回归确认：首帧前保持站立、675→160 力矩等效接管、无相机明确启停、平行收臂趴下、支持模型短缺恢复及真实过期阻止。
- 本板单独编译/执行 test_blind、test_handoff、test_tcp、test_stop_recovery，4/4 通过；27 项 Python 测试通过。
- bash run_blind.sh --offline-check 通过全部源文件/配置/权重/bundle 校验；bash -n 通过。
- 主程序和 base.yaml、VA15/config.yaml、VA15/policy.pt 的 SHA 与同步前一致。

实际板载 bundle SHA-256：2e5a36eca08332c564d3a6e1cf72844ed90184bdd720bc215b56ea86a48755f3。
新独立二进制 SHA-256：f6023c85e47ac1d6642e9523354ec6debdbcf962b706d596eff89463b045ecc3。

## 人工无视觉短距离测试

在板载 PC4 的交互终端执行，保持无枪/软管、平地、运动空间清空，并准备可靠防跌与遥控急停措施：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_tcp_v242_20261023/run_blind.sh eth0
```

输入大写 RUN → A 起立 → 实际站稳 → Y 待机 → 等待 warmup/支撑准备完成 → R1+Y 开始无视觉对照。
固定前向命令 0.15 m/s、横向/转向命令为零，保持原 TCP 观测；不启动相机，不需要第二个 Z1 窗口，不推摇杆。
第一次只观察短距离（约 2～3 秒），按 X 停止；若有明显晃动立即结束测试，不加速或改增益。B 请求停稳后趴下；实际趴稳且 Passive 后再 Ctrl+C。

R1+Y 再按一次也会请求停止，支持停止完成后可明确重新启动。P/L1+X 是移除主动支撑的紧急被动，按既有急停流程使用；不要把断电/重启当作站立时安全停止。

每次会话自动保存至主目录 outputs/va15_blind_v242_XXXXXX，打印实际路径；包含 terminal.log、session.json、commands/control/actor CSV。新列 torque_handoff_* 和 blind_fixed_target 可用于确认切换与持续阶段。

持续高频晃动和真实响应仍待该实机对照；真实视觉时序尚未在本版修改，walk_release_allowed 保持 false。
