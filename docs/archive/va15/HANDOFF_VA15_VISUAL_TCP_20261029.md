# VA15 视觉行走与 TCP 跟踪接管文档

> 最新四次实机测试的定位和本地修复见 [重复停车半蹲与跟踪复核](VA15_FOUR_RUN_STOP_TRACKING_20261029.md)。这次新修复尚未同步板载。

> 2026-10-29 续办更新：请先读 [V256 复核与最新未完成项](HANDOFF_VA15_V256_REVIEW_20261029.md)。下文保留交接时的历史状态；其中 0.20 m/s 实际执行速度、后方交叉计数代表线尾到达、板载 production 路径等表述已在新文档中纠正。

> 同日后续：用户授权后已完成停机修复的 [板载同步、编译和离线自检](VA15_V256_BOARD_SYNC_20261029.md)，未启动控制或运动。下文“未同步”的状态属于历史记录。

更新时间：2026-10-29  
工作目录：`/home/dong/rl_sar_b2_z1`

## 1. 当前任务

继续完善 B2+Z1 的完整视觉行走：

1. 视觉网络输出裂缝中心线和 near tracker 轨迹。
2. B2 按视觉轨迹前进。
3. 后方 TCP 跟踪裂缝末端。
4. 后方 TCP 到达线尾后停车。
5. 视觉丢失时立即停止整段视觉行走。

用户最新要求：恢复“视觉丢失就停止”的逻辑；TCP 功能保持开启。

## 2. 代码和策略位置

- 主仓库：`/home/dong/rl_sar_b2_z1`
- 当前策略：`policy/b2_z1_no_gun/VA15`
- 本地候选源代码：
  `patches/va15_tcp_priority_v256_20261028/production/rl_real_b2_z1_no_gun.cpp`
- 本地候选离线构建目录：
  `patches/va15_tcp_priority_v256_20261028/build_v256/native_candidate_offline`
- 候选说明：
  `patches/va15_tcp_priority_v256_20261028/TCP_PRIORITY_REPORT.md`
- 板载仓库：`/home/unitree/rl_sar_b2_z1`
- 板载二进制：
  `/home/unitree/rl_sar_b2_z1/cmake_build_va15/bin/rl_real_b2_z1_no_gun`
- 板载启动脚本：
  `/home/unitree/rl_sar_b2_z1/scripts/run_b2_z1_real_near_visual.sh`
- 板载地址：`unitree@192.168.123.164`
- 网卡：`eth0`

## 3. 已完成的修改

### 3.1 R1+Y 启动逻辑

- 修复 `Armed -> Forward` 卡死：TCP 优先会话在 `Armed` 和 `Forward` 阶段保持正的机身前进请求。
- 机身速度从 `0.08 m/s` 调整为 `0.20 m/s`，因为 `0.08` 只能产生很小的姿态修正，几乎不前进。
- 常量位于候选源文件顶部：`kTcpPriorityBodySpeed = 0.20f`。
- 视觉会话已经启动后，重复按 R1+Y 不再作为停止命令。
- 停止命令为 X、B 或 Space。
- TCP 优先会话期间忽略摇杆横向和偏航漂移，避免人为输入改变中心线跟踪。

### 3.2 最新的本地未同步修改

在 `near_output_ = near_guide_.Tick(...)` 后加入了视觉丢失保护：

```cpp
if (near_output_.stop && tcp_priority_session_)
{
    tcp_priority_session_ = false;
    tcp_start_pending_ = false;
    std::cout << LOGGER::WARNING
              << "[TCP-priority] visual guide stopped; ending TCP/body session: "
              << near_guide_.reason << std::endl;
}
```

含义：near tracker 判定轨迹过期、拒绝或丢失时，立即结束 TCP/机身视觉会话，让统一的受控停止流程接管。TCP 缓存只用于短暂切换延迟，不能在没有视觉时继续盲走。

注意：这段最新修改在本地已编译并通过离线测试，但在本接管文档生成时尚未确认同步到板载。

## 4. 最近一次实机证据

日志目录：
`/home/unitree/rl_sar_b2_z1/outputs/va15_near_visual_20261029_100719`

关键结果：

- `command_rows=5562`
- `control_rows=5316`
- `actor_rows=3064`
- `dropped=0`
- `io_failed=false`
- `hardware_qualified=false`
- `dds_write_is_motor_ack=false`
- R1+Y 已进入 `trial_stage=Forward`，`policy_running=1`，`requested_0=0.20`。
- `tcp_active=1`，`tcp_ever_active=1`。
- 后方 TCP 已到达后端判定区域：`tcp_rear_crossings=12`，`tcp_rear_gap_m≈0.00028`。
- TCP 目标和实测值存在约 8–11 cm 误差：
  - 中段约 `0.111 m`
  - 结束样本约 `0.085 m`
- 这说明机身视觉行走和 TCP 会话确实启动过，但 TCP 末端仍没有紧密跟随目标轨迹。

上一轮 `20261029_094602` 中 `requested_0=0.08`，机身几乎不产生有效位移；因此已经改为 `0.20`。

## 5. 当前未解决问题

### 5.1 TCP 跟踪误差偏大

先不要直接提高增益。下一次运行需要同时比较：

- `tcp_target_b_x/y/z`
- `tcp_measured_b_x/y/z`
- `tcp_error_norm`
- `tcp_active`
- `tcp_rear_crossings`
- `fsm_target_q_policy_12..17`
- 对应的实际机械臂关节角和速度

判断顺序：

1. 目标 TCP 是否在持续更新；
2. 目标是否经过正确的 B2/机身坐标变换；
3. policy 机械臂目标关节是否变化；
4. 实际关节是否跟随目标；
5. 最后才判断是否需要调整增益或限幅。

### 5.2 视觉丢失策略

用户已经明确要求：视觉识别不到有效轨迹就停止。必须验证：

- 视觉有效时可以持续走；
- 视觉失效后日志出现 `[TCP-priority] visual guide stopped...`；
- 机身速度降为 0；
- TCP/机械臂进入受控停止；
- 不会使用缓存轨迹继续盲走。

### 5.3 实机资格状态

当前日志仍显示：

- `hardware_qualified=false`
- `dds_write_is_motor_ack=false`

离线检查、DDS 写返回值和 MuJoCo 结果都不能单独证明实机已验收。接管帐号应继续保留现有人工急停和分阶段测试流程。

## 6. 接管后的第一步：本地验证

```bash
cd /home/dong/rl_sar_b2_z1

cmake --build patches/va15_tcp_priority_v256_20261028/build_v256 -j2

patches/va15_tcp_priority_v256_20261028/build_v256/native_candidate_offline \
  --offline-check

patches/va15_tcp_priority_v256_20261028/build_v256/test_trial
patches/va15_tcp_priority_v256_20261028/build_v256/test_repeatable
```

如果修改了候选源，先确认测试通过，再计算源文件和二进制 SHA-256：

```bash
sha256sum \
  patches/va15_tcp_priority_v256_20261028/production/rl_real_b2_z1_no_gun.cpp \
  patches/va15_tcp_priority_v256_20261028/build_v256/native_candidate_offline
```

## 7. 同步板载前的检查

不要在控制进程运行时覆盖源码或二进制。先在板载执行：

```bash
ssh unitree@192.168.123.164 \
  'pgrep -af "rl_real_b2_z1_no_gun|run_b2_z1_real" || true'
```

确认没有控制进程后，先备份：

```bash
ssh unitree@192.168.123.164 \
  'mkdir -p /home/unitree/rl_sar_b2_z1/deployment_backups/va15_tcp_priority_v256_20261029'

ssh unitree@192.168.123.164 \
  'cp /home/unitree/rl_sar_b2_z1/production/rl_real_b2_z1_no_gun.cpp \
      /home/unitree/rl_sar_b2_z1/deployment_backups/va15_tcp_priority_v256_20261029/rl_real_b2_z1_no_gun.cpp.before'
```

然后再同步候选源、重新编译板载目标，并运行：

```bash
cd /home/unitree/rl_sar_b2_z1
./scripts/run_b2_z1_real.sh --offline-check
```

离线检查通过后再做人工实机测试。启动命令：

```bash
cd /home/unitree/rl_sar_b2_z1
./scripts/run_b2_z1_real_near_visual.sh eth0
```

操作顺序：

1. 输入大写 `RUN`；
2. 按 `A` 起立，确认站稳；
3. 按 `Y` 进入策略待机；
4. 按一次 `R1+Y` 开始视觉行走；
5. 按 `X` 停止，按 `B` 趴下；
6. 视觉丢失时确认程序自动停止；
7. 不要通过断电代替停止流程。

## 8. 日志分析命令

```bash
cd /home/unitree/rl_sar_b2_z1
find outputs -maxdepth 1 -type d -name 'va15_near_visual_*' | sort | tail
```

优先查看：

- `summary.json`
- `control.csv`
- `near_guide.csv`
- `arm.csv`（如果该版本生成）
- `events.jsonl`

重点搜索：

```bash
rg -n "TCP-priority|visual guide stopped|tcp_active|tcp_error_norm|tcp_rear_gap|requested_0|trial_stage|visual_stage" \
  outputs/va15_near_visual_*/
```

## 9. 接管结论

当前最接近目标的是 VA15 TCP-priority 候选版本：机身可以进入视觉行走，后方 TCP 已经激活并到达线尾区域，但 TCP 仍有约 8–11 cm 跟踪误差。最新本地修改已经恢复视觉丢失即停止，下一帐号应先完成离线验证，再决定是否同步板载，最后用一条连续黑线做单次实机测试。

