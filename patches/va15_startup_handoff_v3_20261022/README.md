# VA15 起步接管 V3 候选 — 2026-10-22

**结论：保存候选，不替换 V2，不开放候选实机运动。** 起步支撑交接确有改善，但同条件直行侧偏增大。34 个软件场景通过不等于整体行走性能验收通过。

## 最终保留的修改

1. 真正的策略首帧到来之前保持完整旧站立指令，包括 18 关节位置、速度、Kp/Kd 和前馈项。不提前将腿部增益切到运动值。
2. 首帧到来时，按同一时刻的实际 q/dq 匹配旧指令的 PD+FF 请求。之后 1.5 秒按 smoothstep 在**每周期当前状态**下的站立与策略 PD 请求之间过渡，而非保持一个固定首状态位置偏置：
   - tau_stand = Kp_stand * (q_stand - q) + Kd_stand * (dq_stand - dq) + ff_stand
   - tau_policy = Kp_policy * (q_policy - q) + Kd_policy * (dq_policy - dq) + ff_policy
   - tau_join = remaining * tau_stand + (1 - remaining) * tau_policy
   - q_command = q + (tau_join - Kd_join * (dq_policy - dq) - ff_policy) / Kp_policy
3. 不使用 tau_est 闭环，不添加额外重力补偿。上式仅在软件模型的当前采样状态下等效，不是实测电机扭矩或电机确认。
4. 首帧等待超过 0.10 秒（名义控制周期累计）、收到取消、反馈无效、数值异常或目标越过原安全范围时拒绝新目标。首帧前拒绝后持续保留完整站立帧；已运动后的拒绝保留上一帧并请求原受支持停止流程，不强制降低。
5. 增加 startup stage/wait/elapsed/first requested-PD error 日志。

VA15 权重、70/18 布局、关节映射、raw action history、动作 EMA 0.2、静止/运动增益端点、目标范围、原电机/足端/停止门槛和 Z1 滤波均未改。B2 总增益缩放限定为 1。V2 原有 0.15 m/s、单次最多 4 单调墙钟秒的请求预算也未放宽；V3 实机前进则明确拒绝。

## 最终离线证据

- CTest **5/5**：电机契约、诊断记录、试验监督器/来源隔离、首帧及变化状态下的 PD 连续性、native FSM 的等待/取消/X/B/超时/失效反馈。
- Python 诊断解析 **1/1**。
- `outputs/regression_final/summary.json`：**34/34** 无窗口 MuJoCo 场景通过。故障场景的通过含义是正确拒绝捕获/降低，不是忽略故障仍然趴下。
- `outputs/paired_final/summary.json`：5 组同条件对照，共 10 次运行。
- `outputs/review_final.json`：**运动候选不接受，保留 V2。**

默认摩擦 0.8、估计器 torque 输入倍率 1.0：

| 指标 | V2 接管 | V3 候选 |
| --- | ---: | ---: |
| 起步至首帧，同状态最大 PD+FF 请求变化 | 37.3273 N·m | 0.00000429 N·m |
| 前 1.5 秒最低机身高度 | 0.504654 m | 0.523912 m |
| 前 1.5 秒最高机身高度 | 0.551288 m | 0.549386 m |
| 前 1.5 秒上下范围 | 4.66 cm | 2.55 cm |
| 前 1.5 秒最大倾斜 | 0.884° | 0.815° |
| 4 秒请求结束时前进距离 | 0.384298 m | 0.413324 m |
| 请求结束时横向偏移 | -6.42 cm | -11.07 cm |

图：`outputs/paired_final/startup_comparison.png`。

**不能只看起步改善。** 默认条件侧偏增加约 4.6 cm，低摩擦条件也变差。未达到“不退化直行”的离线筛选要求，因此不用于实机前进，也不声称抖动或步态已全部解决。后续需区分机身偏航和机身坐标系横移，再单独处理航向/横向漂移。

MuJoCo 的 Z1 是隐式位置伺服代理，不是 SDK 验证；tau 倍率仅扰动估计器输入，不代表电机强度随机化。上述数字不是实机安全认证或长时间行走验收。

## 试过但未采用的方案

- 固定首帧等效位置偏置释放 1.5 秒：首帧连续，但起步上浮约 4.4 cm。
- 对完整 actor 步态保留高站立 Kp 再下降：离线失稳，34 场景仅 4 个通过；未同步该方案到板载。
- 固定偏置短释放及 0.75/1.0/1.25 秒对照：侧偏没有同时改善。
- `rejected_variants/` 和各历史输出目录保留实验记录；旧结果不代表当前源代码。

## 板载状态

独立目录：`/home/unitree/rl_sar_b2_z1/diagnostics/va15_startup_handoff_v3_20261022`。

最终编译产物：`native_build_v3_final/va15_forward_trial`。仅用于离线检查，native 构造器在接管前明确拒绝真实前进请求。启动器除 `--offline-check` 外全部拒绝。未替换 V2 启动器/二进制，未改 VA15 配置/权重；本轮未初始化机器人 DDS/UDP/相机，也未接管 B2/Z1。

板载终端只读核对命令：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_startup_handoff_v3_20261022/run_trial.sh --offline-check
```

不要直跑 candidate native、不要运行未选中的旧候选产物。原 V2 控制入口仍然是原来的版本，不能用它来判断本候选效果。

## 本机复现

在 `/home/dong/rl_sar_b2_z1`：

```bash
cmake -S patches/va15_startup_handoff_v3_20261022 -B patches/va15_startup_handoff_v3_20261022/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build patches/va15_startup_handoff_v3_20261022/build -j2
ctest --test-dir patches/va15_startup_handoff_v3_20261022/build --output-on-failure
python3 -m unittest discover -s patches/va15_startup_handoff_v3_20261022 -p test_analysis.py -v
```

套件输出目录不能重名，不能覆盖既有报告。绘图使用 crack_vision 的 Python；未修改 base 环境的 NumPy/Matplotlib。
