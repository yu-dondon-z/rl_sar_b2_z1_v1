# B2+Z1 部署版本 v4

保存日期：2026-09-14（Asia/Shanghai）

这是 `/home/dong/rl_sar_b2_z1` 当前可运行状态的可恢复快照，基础提交为
`1fae490143f02f0098cfa90d15b9dd3e679cbd34`。

## 版本内容

- 增加完整的趴下、站起、焊缝枪安装、视觉轨迹跟踪和安全趴下流程。
- 初始状态下 B2 趴卧，Z1 收紧到零位，仿真焊缝枪隐藏并关闭碰撞。
- 按 `0`：B2 先站起，随后 Z1 展开到焊缝枪安装工作位。
- 按 `1`：仿真中安装焊缝枪，并开始策略机械臂的平滑接管。
- 按 `T`：启用视觉裂缝轨迹跟踪。
- 按 `9`：先平滑停止，仿真中拆下焊缝枪，Z1 收紧，再让 B2 趴下；
  实机入口会明确提示先人工拆枪。
- 保存 MuJoCo 可视化入口、终端状态机回归检查、实机视觉桥接、策略配置、
  模型资源和 VS Code 调试配置。
- 不包含编译产物、训练日志、缓存、`deployment_backups/`、其他历史工作副本
  以及 VS Code 索引数据库。

## 策略文件

- 部署策略：`policy/b2_z1_no_gun/robot_lab/policy.pt`
- SHA256：`b77e968cdfce021779b4276d067aaf019987e5f26e3e3141b82322dd083cd408`
- 独立 v4 播放包策略：`policy/b2_z1_no_gun/v4/policy.pt`
- SHA256：`7b2efd8c5355561b47274b976a001d0deca646717bf1542addd8c08d0c3eec02`

## 已验证状态

- `scripts/check_b2_z1_v4.py` 通过。
- `scripts/check_b2_z1_v4_terminal.py` 通过。
- 18 秒无窗口 MuJoCo 生命周期测试通过。
- 实机目标 `rl_real_b2_z1_no_gun` 编译通过。

## 恢复

在仓库根目录执行：

```bash
bash deployment_versions/v4/restore_v4.sh
```

也可恢复到另一份 `rl_sar_b2_z1` 仓库：

```bash
bash deployment_versions/v4/restore_v4.sh /path/to/rl_sar_b2_z1
```

恢复脚本只覆盖快照内保存的文件，不会删除目标仓库中的其他内容。恢复前若
目标仓库仍有需要保留的修改，请先另行备份。

## 校验

```bash
cd deployment_versions/v4
sha256sum -c SHA256SUMS
```
