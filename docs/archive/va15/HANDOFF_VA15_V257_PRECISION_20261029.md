# VA15 V257 本地候选交接（2026-10-29）

用户最终明确的末端指标是：TCP 在地面上的 XY 投影到裂缝中心线的最短距离 ≤ 0.05 m；枪尖高度沿用 VA15 训练配置 0.09 m。本体足底外缘不得进入裂缝带。

## 当前候选

完整候选目录：`patches/va15_body_precision_v257_20261029/`。

本候选未同步板载、未启动控制、未启动机器人运动。V256 板载版本和历史日志保持不动。

## 已完成

- `production/body_path_control.hpp`：仅基于当前和已缓存的实际观测折线控制本体。未观测到后方线段时不向后外推。
- `production/rl_real_b2_z1_no_gun.cpp`：视觉 TCP 会话接入本体控制；保持原速度、偏航、时效和停止门控。新增控制只使用原策略的前进/偏航通道。
- `production/foot_seam_geometry.hpp`：几何足底离线检查，不修改接触信号。
- `production/tcp_fk.hpp`：补充与 MuJoCo `ee_site` 一致的解析位置 Jacobian。
- `precision_tcp_control.hpp` / `planar_tcp_control.hpp`：笛卡尔末端修正实验保留但未接入原生默认路径。直接叠加 IK 的仿真案例出现倾角/足部侵线，故不发布。

## 证据

- `test_precision`：Jacobian 与 MuJoCo 最大误差 `1.89e-15`；足部固定几何偏差 `0.174 mm`；IK 数值残差 `1.21e-7 m`。
- `native_candidate_offline --offline-check`：通过；没有 DDS/UDP、相机、Z1 服务或机器人控制。
- 直线 `body_straight_03`：稳定阶段 TCP 地面投影误差 RMS 约 10–11 mm、95 分位约 20 mm；加载足底边缘间距为正。
- 温和弯线 `body_gentle_left_01`：足底加载边缘间距为正，但进入过渡期间仍有超过 5 cm 的误差，不能宣称全程合格。
- 急弯 `body_left_03`：在当前视觉包络/偏航上限下提前停止，保留为失败证据。

## 下一步

1. 先在仿真中完成温和弯线的减速/进入段验证，要求稳定窗口内投影误差 ≤ 50 mm 且所有加载足底外缘保持正间距。
2. 重新编译完整本地候选并做现场前离线检查。
3. 用户重新确认板载状态和同步授权后，才生成板载同步包；当前禁止同步和运动。

不要将 `qualification.json` 中的 `hardware_qualified`、`board_synced` 或 `offline_full_curve_pass` 改为 true；不能把失败的 IK 实验当作精度修复。
