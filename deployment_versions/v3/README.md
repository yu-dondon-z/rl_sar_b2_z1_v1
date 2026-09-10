# B2+Z1 部署版本 v3

保存日期：2026-09-10（Asia/Shanghai）

这是 `/home/dong/rl_sar_b2_z1` 当前可运行状态的可恢复快照，基础提交为
`1fae490143f02f0098cfa90d15b9dd3e679cbd34`。

## 版本内容

- 当前部署策略来自训练任务
  `2026-09-09_16-01-19_rear_vision_v3_gait_finetune_from9600/model_10000.pt`。
- 部署用 TorchScript：`policy/b2_z1_no_gun/robot_lab/policy.pt`。
- TorchScript SHA256：
  `b77e968cdfce021779b4276d067aaf019987e5f26e3e3141b82322dd083cd408`。
- 保留 B2+Z1 后向机械臂、视觉裂缝轨迹、平坦场景速度控制和实机入口。
- 保存当前 MuJoCo/URDF 模型、策略配置、控制源码、Z1 SDK、运行脚本及
  VS Code 调试配置；不包含 VS Code 的 1.6 GB 索引数据库。

## 已验证状态

无视觉平坦场景以 `x=0.30 m/s, y=0, yaw=0` 连续测试约 67 秒：

- 未摔倒，非足端触地为 0；
- 机身高度为 0.51--0.57 m；
- 平均实际前进速度约 0.186 m/s；
- 可以仅靠本体稳定前进，但仍有缓慢向右偏航，尚不能长期严格保持世界直线。

## 恢复

在仓库根目录执行：

```bash
bash deployment_versions/v3/restore_v3.sh
```

也可恢复到另一份 `rl_sar_b2_z1` 仓库：

```bash
bash deployment_versions/v3/restore_v3.sh /path/to/rl_sar_b2_z1
```

恢复脚本只覆盖快照内保存的文件，不会删除目标仓库的其他文件。恢复前若
目标仓库仍有需要保留的修改，请先另行备份。

## 校验

```bash
cd deployment_versions/v3
sha256sum -c SHA256SUMS
```

