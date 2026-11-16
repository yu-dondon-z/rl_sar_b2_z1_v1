# V2.4.6 板载离线候选（2026-10-26）

状态：本次独立源代码同步、板载离线构建和指纹核对已完成。
没有启动机器人控制、Z1服务、DDS/UDP、相机，也没有实机行走验收。

操作者本轮确认 B2 已连接、实际趴稳、控制程序退出、无枪和软管，
授权同步但不启动。只读检查确认 PC4 在线、没有部署控制进程。

本次只写独立目录：
`/home/unitree/rl_sar_b2_z1/diagnostics/va15_visual_tcp_v246_endpoint_20261025`。
保留旧候选、主二进制、VA15权重、base/profile YAML，不启动控制或Z1服务。

本版 `walk_release_allowed=false`。仿真真值可通过终点停车，但独立位置估计
发现物理终点偏差，不能把局部估计中的到达作为实机精度验收。

仅开放检查 `--offline-check` 和相机 `--preview`；预览只取图，不站立，
不启动策略。禁止使用 `--shadow` 代替只读预览，后者会进入人工控制流程。

## 本版验证

- 本地C++17/17、Python54/54，2005组模型FK对照通过。
- 最终源码对应真值终点16/16；12个正常终点物理平面误差约0.5～2.65cm。
  本体估计终点4/8，只有4类异常停止通过，4个正常终点实际误差约18～21cm。
  新版会报告物理终点未验证，不会冒充 `COMPLETED`。原视觉/TCP回归8/8。
- PC4私有主程序离线黄金对照通过，最大误差1.14441e-05；70观测/18动作和
  VA15 model_5000一致，没有初始化机器人传输。
- PC4新增终点、实测FK、终点包校验3项原生测试全部通过；Python54/54通过。
- `check_bundle.py` 核对所有绑定源文件、策略和配置通过。
- 更新前后检查没有活动控制/相机进程；未触发任何站立、策略、趴下或关闭动作。

## 指纹

- 独立bundle：`9a6a20351b6d99b4c0f11743515a4f5f52d577a27ab520e2172725d3ba71bf45`
- 私有二进制：`5506ddcc4edf53720c87f2fb4046132e56aed931c9b21a8f45b554c0c1d39262`
- 主二进制（未改）：`696756deac43676046b0073d6c7ee0c0eb1a79f0248f678f9df337013f8d9ba3`
- base.yaml（未改）：`909d04f2ce4820effd7a284ccff5955dc6a9c8fba921f0fe42a439627ced7164`
- VA15配置（未改）：`8faa51628b3704ce3e7e3a384f1c0eb8bb59b7dcf5a0069992069efed8aab8d1`
- VA15权重（未改）：`8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098`

本报告是编译后的独立审计附件，不参与bundle自身哈希，不改变行走许可。
`qualification.json` 仍为 `walk_release_allowed=false`、`simulation_pass=false`、
`hardware_qualified=false`、`precision_qualified=false`。

## PC4 NoMachine桌面终端：只预览

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_tcp_v246_endpoint_20261025/run_visual.sh eth0 --preview
```

Q或Ctrl+C关闭本次相机预览。红圈仅表示单帧末端候选，不是任务完成。
若没有DISPLAY，使用NoMachine桌面终端，不猜测或强设显示编号。
不提供本候选的实机行走指令；旧入口保留但没有被本次自动切换或运行。
