# B2+Z1 部署版本 v1

保存日期：2026-08-03（Asia/Shanghai）

这是当前效果较稳定的 B2+Z1 MuJoCo/实机部署快照，基于：

- Git 分支：`main`
- 基础提交：`1fae490143f02f0098cfa90d15b9dd3e679cbd34`
- 策略目录：`policy/b2_z1_no_gun`
- MuJoCo 模型：`src/rl_sar_zoo/b2_z1_no_gun_description`

当前版本包含低速命令映射、高速稳定性限制、安全停车、机械臂抑振和
MuJoCo 机器人跟随视角等部署调整。`files/` 保留了恢复这些功能所需的
源码、配置、模型资源、策略权重、Z1 SDK 和 VS Code 运行调试配置。

## 恢复

在仓库根目录执行：

```bash
bash deployment_versions/v1/restore_v1.sh
```

也可以向脚本传入另一个 `rl_sar_b2_z1` 仓库目录：

```bash
bash deployment_versions/v1/restore_v1.sh /path/to/rl_sar_b2_z1
```

恢复脚本只覆盖快照中保存的路径，不删除目标目录中的其他文件。恢复前若
当前版本仍需保留，请先另行备份。

## 校验

```bash
cd deployment_versions/v1
sha256sum -c SHA256SUMS
```

`tracked_changes.patch` 记录了保存时相对于基础提交的已跟踪文件差异，
`git_status.txt` 记录了当时工作区状态，便于审计。


##部署到实机

方法1 通过8+2转网线口连接到自己电脑，在自己电脑上跑程序
