# V249 板载同步记录（2026-10-26）

目标为 diagnostics/va15_body_tcp_v249_20261026，保留 V248 和原主程序。

本地 19/19 C++、62/62 Python、8/8 长时间 TCP 物理、11/11 终点停车物理及 66/66 保存图片包验证已完成。详见 VALIDATION.md 与 qualification.json。

## 已完成

已同步到 /home/unitree/rl_sar_b2_z1/diagnostics/va15_body_tcp_v249_20261026，并在 PC4 独立编译 native_build_visual/va15_forward_trial。

- 板载 62/62 Python 回归通过。
- 独立 C++ 编译、原生 VA15 70/18 golden、主入口 run_visual.sh --offline-check 通过；最大 golden 误差 1.14441e-05。
- 本地保存板载 bundle 和检查日志：outputs/board_checks/。qualification.json 为板载编译前冻结的本地资格快照，里面的 pending board 字段由本记录补充；不改动已绑定的源码/元数据以冒充重新编译。
- bundle SHA-256：c4a02926fdafe5205126becfc154870d0952f4d2237ba50a7f99390bf088b02c。
- 独立可执行文件 SHA-256：9fd4526fc52b8aa03e6331e1aba78ac2099fd960e122e06f2b4e44774b736864。

同步后再次核对，原主程序、base.yaml、VA15/config.yaml 与 VA15/policy.pt 均未修改：

```text
main    696756deac43676046b0073d6c7ee0c0eb1a79f0248f678f9df337013f8d9ba3
base    909d04f2ce4820effd7a284ccff5955dc6a9c8fba921f0fe42a439627ced7164
config  8faa51628b3704ce3e7e3a384f1c0eb8bb59b7dcf5a0069992069efed8aab8d1
policy  8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098
```

没有启动相机、Z1 服务、控制程序或实机运动，没有释放厂商控制。硬件、实体工具负载与精确三维跟踪仍未验收；保留原 X/B/通信时效及健康检查。

## 人工入口

板载 NoMachine 桌面终端执行：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_body_tcp_v249_20261026/run_visual.sh eth0 --walk
```

确认仍无枪/软管、已趴稳、旧控制退出，输入大写 RUN → A → 实际站稳后 Y → 确认正确胶带 → R1+Y。X 停止、B 趴下，实际趴稳且 Passive 后 Ctrl+C。没有新增静态审查/窗口 R/短距离或一次性限制。

TCP 初始保持默认目标，需前视历史实际到达后方才接入；观察 active=1 与 tcp_reason、target_b 以及 CSV 的 measured/target/error。并非启动即有完整初始后方可见轨迹。
