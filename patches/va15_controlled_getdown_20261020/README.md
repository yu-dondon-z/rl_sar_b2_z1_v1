# VA15 独立 B 停止／趴下候选（2026-10-20）

已备份并同步 PC4、重新编译，通过原生离线校验、内存状态机测试、
B2-only 只读预检。**不等于行走后实机趴下已验收**。
本轮没有启动低层控制、Z1 控制器或发送运动命令。

当前仅准备无焊缝枪、无软管的零速度 A/Y/B 静态测试。
真实足端标定和低层高度/速度来源仍未验证，实机行走继续锁定：

```yaml
independent_B_stop_candidate: true
real_foot_contact_calibration_validated: false
real_stop_sport_feedback_validated: false
real_kinematics_shadow_only: true
```

不能直接将 validated 改 true；不能用运动学影子诊断或假接触放行。
1 kg 末端负载压力测试仍有 2/8 未完成下降，暂不支持带枪／软管动态测试。

## B 的新流程

```text
B 锁存并清零、锁定运动输入
  → 原策略受限减速
  → 新鲜有效的四足支持窗口（不是 FK 猜接触）
  → 独立控制接管、结束 actor、丢弃排队动作
  → 平滑回到已知固定站姿
  → 持续 0.5 s 原有高度／姿态／速度／四足支撑检查
  → GetDown：先收 Z1 2 s，再降低 B2 2 s
  → Passive
```

与原流程的区别是：不用零速 actor 连续完成静止站立后才停止 actor。
捕获短支持窗口，允许小量残余关节运动；随后用独立控制完成真正静止验证。
捕获不是趴下许可。最终高度 >=0.48 m、倾斜 <=0.15、平面速度 <0.04 m/s、
角速度 <0.18 rad/s、最大腿关节速度 <0.55 rad/s 及四足新鲜接触要求保留。

实现细节：

- 只有起步前已经连续通过 0.5 s 支撑检查时，才记录站立 PD 支撑力矩参考。
  不从运动中的 actor 或缺失反馈中制造参考。
- 捕获时等效转换最后请求的 PD+前馈力矩，保持首帧请求力矩连续。
  这不是实测电机输出力矩连续性的硬件证明。
- 在约 0.1 s 消退捕获速度记忆、0.3 s 消退动态步态力矩，转向站立支撑参考。
  不持续保留某一帧未来抬腿目标，也不无限抵消阻尼。
- 约 2 s 平滑变换至既有站立目标和增益，不提高既有站立或策略增益。
  四足条件短暂不满足时暂停站姿插值，仍执行有界的制动。
- 不要求负载关节误差收敛到零；PD 本就需要位置误差承重。
  最后以有效支撑、机身运动和关节漂移检查静止。
- 高误差步态帧不适合捕获时，保留 B 并等下一窗口，不扩大目标范围。
  缺失参考、缺失/陈旧状态、姿态异常或超时均不授权下降。
- B 后不能用摇杆、A/Y 或排队的 actor 动作重新启动；P/紧急请求优先。
  从未启动行走 actor 的站立／Y 待机仍沿原静态 B 路径趴下。

反馈中途失效时，该候选不承诺自动维持平衡或自动趴下；故障注入只证明
拒绝不具备依据的下降，不是故障状态下的物理防摔验收。

## 验证结果与限制

本机 13 组 CTest 全部通过，包含原生 VA15 70/18 契约、旧安全行为回归、
新的力矩连续性、B 锁存/优先级、缺失来源及 actor 输出隔离等测试。

最终动力学报告：`outputs/suite_v6/summary.json`，源码哈希与本候选相同。

| 范围 | 结果 |
| --- | --- |
| 无枪／无管，前向指令 0.10/0.15/0.20/0.30 m/s，多相位 B | 16/16 完成 |
| 静态 B、起步途中 B、摩擦系数 0.5/1.0、30 s 行走后 B | 8/8 完成 |
| 接触无效、少一脚、速度无效、过期数据、捕获后过期 | 5/5 拒绝下降 |
| 1 kg TCP 代理负载，多速度/相位 | 6/8 完成，2/8 拒绝下降；未通过负载资格 |

0.20 m/s、行走 8 s 的例子：按 B 后约 1.43 s 捕获、4.50 s 进入收臂下降
流程，约 10.51 s 完成趴卧并继续静置 2 s；最大倾斜约 2.15 度。
这些是仿真结果，不是实机响应时间承诺。低速数据只验证停止流程，
不表示该策略在 0.10 m/s 有合格的速度跟踪或步态。

模型为既有 VA15 MuJoCo 场景，Z1 使用其已有隐式位置伺服代理。
B2 显式 PD 按 200 Hz 更新，物理步长 1 ms；这不能等同实机电机内环。
足端、身高与速度使用明确标记的 MuJoCo 真值，不是实机足端标定。
测试的角速度从世界坐标显式转换到 base_link，避免将 MuJoCo 主惯性轴
误用为机身轴；这项修正只在新测试器中，未修改实机 DDS IMU 轴。
未验证视觉、侧移、转向、坡地、软管拉力或带枪收臂碰撞。

带完美真值的旧停止路径在本次参考仿真中也能完成趴下，因此不能据此
声称已修复实机缺失高度／速度及足端信号异常；该问题仍需独立处理。
本候选只解决停止流程对零速 actor 静止行为的依赖，并保留数据边界。

## 复现（本机，无机器人接口）

```bash
cd /home/dong/rl_sar_b2_z1/patches/va15_controlled_getdown_20261020
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 run_simulation_suite.py --output-dir outputs/new_suite
```

`run_simulation_suite.py` 保留并报告负载失败，不会隐藏失败或覆盖旧结果。
当前完整套件返回非零，是因为两项 1 kg 压力测试尚未通过。
单项无枪复现：

```bash
env LD_LIBRARY_PATH=/home/dong/rl_sar_b2_z1_old/library/inference_runtime/libtorch/lib \
  build/simulate_controlled_getdown --speed 0.20 --walk-seconds 8 \
  --output outputs/new_walk020.csv
```

`render_simulation_review.py` 从记录的 qpos 重建检查图；不参与测试动力学。
已查看 `outputs/final_walk020_review.png`，确认收臂后降低及最终趴卧姿态。

## 板载同步与备份

备份：`/home/unitree/rl_sar_b2_z1/deployment_backups/controlled_getdown_20261020_HMsouK/`。
`before.tar.gz` 保存旧 FSM、配置、manifest、检查脚本和二进制；
`before.sha256` 保存更新前身份。未替换主目录、未上传本机二进制或改权重。

| 本目录 | PC4 仓库内位置 |
| --- | --- |
| fsm_b2_z1_no_gun.hpp | src/rl_sar/fsm_robot/fsm_b2_z1_no_gun.hpp |
| va15_controlled_stop.hpp | src/rl_sar/library/core/rl_sdk/va15_controlled_stop.hpp |
| VA15_config.yaml、VA15_manifest.json | policy/b2_z1_no_gun/VA15/config.yaml、manifest.json |
| check_va15_legacy.py | scripts/check_va15_legacy.py |
| test_controlled_stop.cpp | scripts/test_va15_controlled_stop.cpp |
| verify_board_candidate.py | scripts/verify_va15_controlled_stop.py |

源码/配置和 VA15 actor 哈希在启动前检查，未经资格确认的候选还要求两个
真实来源验证位保持 false。板载已重新编译，通过 `--offline-check`、
板载原生内存状态机测试及 `--preflight --disable-z1`，没有接管官方控制。

当前操作者只可复验无枪、无管、零速度 A→B 或 A→Y→B 的静态流程；
不要推动摇杆、按 R1+Y/T 或将这些静态测试当作动态验收。
确认实际趴稳且终端进入 Passive 后再 Ctrl+C；不要站立时直接断电/重启。
动态行走和行走后趴下实测需先完成真实支持/停止反馈来源验证。
