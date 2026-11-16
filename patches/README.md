# 历史实验补丁索引

这里保留 46 份 VA15 阶段候选补丁与验证记录，主要日期为 2026-10-20 至 2026-10-29。各候选的 `README.md`、`VALIDATION.md` 或 `*_REPORT.md` 仅说明当时状态。

| 目录名称 | 主要主题 |
| --- | --- |
| `va15_*_20261020` | 初始控制、起立、退出与支撑诊断 |
| `va15_*_20261021` | 停止、反馈、电机契约与恢复 |
| `va15_*_20261022`、`va15_*_20261023` | 视觉行走与起步接管 |
| `va15_visual_tcp_*`、`va15_*_20261026` | 视觉 TCP、端点与曲线进入 |
| `va15_*v25*` | 后续恢复、SDK 所有权、TCP 优先级和精度候选 |

本次保留整个目录及文件布局，原因是存在实际路径依赖：

- [assemble_bundle.py](../deployment/vd2_real/tools/assemble_bundle.py) 从 `va15_tcp_priority_v256_20261028/config/b2_builtin_front_camera.yaml` 读取历史相机标定。
- `.vscode/settings.json` 的 `cmake.sourceDirectory` 指向 `va15_body_precision_v257_20261029`。
- 多份候选的构建和测试使用同目录相对路径，保留布局便于复现。

体积约 10.7 GiB，包含源码副本、构建产物和验证输出。本次只整理索引；后续回收空间需要先区分仍需复现的候选和可重建的产物。当前板载入口见 [部署说明](../deployment/vd2_real/README_板载编译与运行.md)，历史报告见 [文档归档](../docs/archive/README.md)。
