# B 专用减速对比：离线第一轮

这里只做本机 CPU 无窗口模拟，没有机器人入口，未同步板载程序。
原 `va15_independent_stop_20261021` 版本及其报告保持不变。

6 档 B 减速参数 × 2 档速度 × 4 个 B 时刻：48/48 通过。
单独将 B 后的 XY 命令减速由 0.12 改成 0.30 m/s²，完整回归 30/30。
没有改动普通行走、加速、actor、增益、估计器或支撑/停稳判据。

但更早捕获站姿不代表更早物理停稳。仅改减速时，部分场景捕获后仍
有姿态调整，不能据此直接部署。后续过渡时长对比保留于
`../va15_stop_response_v2_20261021/`，当前离线候选及完整结论见
`../va15_stop_response_v3_20261021/README.md`。

原始记录：`outputs/brake_sweep_v1/summary.json`、
`outputs/brake_030_full_v1/summary.json`、`outputs/comparison_v1.json`。
