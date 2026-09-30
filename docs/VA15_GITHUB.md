# VA15 代码版本

此分支包含 MuJoCo VA15 策略、配置、调试入口和 B2+Z1 模型资产。
旧部署快照、测试输出及临时备份未纳入提交。

在另一台电脑还原：

```bash
git clone --branch VA15 --recurse-submodules git@github.com:yu-dondon-z/rl_sar_b2_z1_v1.git
cd rl_sar_b2_z1_v1
bash scripts/setup_b2_z1_assets.sh
```

随后参见 [VA15_MUJOCO.md](VA15_MUJOCO.md) 运行仿真。B2+Z1 描述文件以
`assets/b2_z1_no_gun_description` 保存在主仓库；初始化脚本仅在子仓库内
缺少该目录时复制，不覆盖已有文件。原工作区中的嵌套 Git 仓库没有改动。
