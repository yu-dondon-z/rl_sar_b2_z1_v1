# V2.4 板载同步记录（2026-10-23）

用户确认 B2 实际趴稳、控制退出、无枪和软管、Z1 收臂路径无遮挡后执行。全程只做文件同步、编译和离线测试；没有起立、策略接管、相机采集、Z1 启动或 B2 lowcmd 发布。

## 位置与保留版本

- PC4：`unitree@192.168.123.164`，接口 `eth0`。
- 独立候选：`/home/unitree/rl_sar_b2_z1/diagnostics/va15_visual_tcp_v24_20261023`。
- 原板载中间 V2.4 保留为 `diagnostics/va15_visual_tcp_v24_20261023_intermediate_backup_20261023`。
- V2.3、主部署程序及 VA15 权重未覆盖。最终编译 bundle 绑定候选源码、配置、VA15 权重和实际二进制，启动前重新核对。

## 已验证

- PC4 实际编译的 VA15 model_5000：70 维观测、18 维动作；黄金样本最大误差 `1.14441e-05`，在原绝对/相对误差容限内。
- 权重 SHA-256：`8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098`。
- PC4 编译运行 TCP 组件：平地投影、完整姿态变换、历史插值/重置、仅已观察后方点、平滑接入及失效冻结通过。
- PC4 编译运行图像引导组件：左右符号、新鲜度、回放、启动、停止、变化率和会话解析通过。
- PC4 27 项 Python 测试全部通过；开发电脑重新执行 9 项 C++、27 项 Python 测试也全部通过。
- 相机模型权重及候选内标定与上一轮 V2.3 一致。基础仓库的未标定模板不被此候选使用，也未被改写。
- 主二进制 SHA-256 保持 `696756deac43676046b0073d6c7ee0c0eb1a79f0248f678f9df337013f8d9ba3`。

## 操作

在 **PC4 的 NoMachine 桌面终端**执行，运行时保持无枪/软管、平地且运动空间清空：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_tcp_v24_20261023/run_visual.sh --offline-check
bash diagnostics/va15_visual_tcp_v24_20261023/run_visual.sh eth0 --walk
```

输入大写 `RUN` 后，`A` 起立 → 实际站稳 → `Y` 待机 → 确认黄/青曲线确实位于胶带上 → `R1+Y` 开始。

先做短程 `X` 停止，再验证 `B` 趴下。实际趴稳、显示 Passive 后才 `Ctrl+C`。`R1+Y` 可重复启动/停止，不需要窗口 R 或额外四秒审查。正常持续行走直到按键停止或实际数据失效；不会自行重启。

TCP 初始保持原目标，只有走过前方已观察的线、后方点进入工作空间时才 `active=1`；之后平滑趋近训练离地 9 cm。日志 `tcp_reason` 给出等待/不可达原因。不能把 `active=0` 当成已经在跟踪历史路径。

## 尚未验收

`walk_release_allowed=true` 只表示可手动开展本次实验；`hardware_qualified=false`、`precision_qualified=false`。当前仍为光学单目平地投影，不是深度相机；IMU/关节无滑移里程计对共同打滑无独立观测。

离线完整 TCP 位置命令误差约 8 cm，机身中心最大横偏约 16～17 cm，不能用于焊接精度验收；本轮没有带枪/软管实机测试。完整仿真原始结果保留在开发电脑，板载另有摘要副本。
