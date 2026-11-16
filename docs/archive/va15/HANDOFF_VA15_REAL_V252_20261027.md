# B2＋Z1 实机部署交接：VA15 / V251 → V252

更新日期：2026-10-27，Asia/Shanghai。

**帐号切换最新补充（2026-10-27 15:55）：** 请先读 [HANDOFF_VA15_ACCOUNT_SWITCH_20261027.md](HANDOFF_VA15_ACCOUNT_SWITCH_20261027.md)，再按本文读取原证据。新文件汇总本帐号离线工作、未解决项及暂停中断点；本文原始交接内容保留。

本文用于更换 Codex 帐号后继续工作。新的帐号应先读本文，再读其中的本地证据；不要依赖旧帐号的聊天记录或记忆。这里记录的是交接时状态，机器人姿态、连接情况及运行进程必须重新确认。

## 1. 一分钟了解当前进度

目标是让 B2 搭载后向 Z1，使用前置相机识别地面的黑色胶带/裂缝，机身沿轨迹行走，后方虚拟工具 TCP 跟踪已走过的历史轨迹；看见可信线尾后站立停止。最终还要用于实机。

当前集中排查实机部署，不是在训练新策略。使用的是 **VA15 model_5000**，不是 V19，也不是最新 VB 系列。

最近一次 V251 实机测试出现三个连续问题：

1. 按 R1+Y 后高频小幅抖动，前进不明显。
2. 约 10.32 秒后，一帧视觉长前视不合格导致保护停车。
3. 停车恢复一直不能通过右前脚的可信支撑判断；后续 R1+Y 和 B 都收到，但既无法重启，也无法进入趴下。

已经新建 **V252 本地候选**，修改了 B 的视觉抢占、停止恢复参考和诊断。常规回归通过，但本次实机故障尚未针对性验证通过，**不能说抖动或无法趴下已经解决**。

**V252 尚未同步板载，禁止把其当作已发布行走版本。** 没有改主目录默认程序、策略权重、接触门限或实机增益。V251 保留。

## 2. 目录、版本和硬件状态

| 内容 | 交接时位置/状态 |
| --- | --- |
| 实机部署主仓库 | `/home/dong/rl_sar_b2_z1` |
| 训练仓库 | `/home/dong/b2_z1`，本轮未修改训练 |
| 当前 Codex 初始工作目录 | 可能仍为 `/home/dong/legged_amp_lab`，不要因此改错仓库 |
| V251 本地版本 | `/home/dong/rl_sar_b2_z1/patches/va15_curve_entry_v251_20261026` |
| V252 当前候选 | `/home/dong/rl_sar_b2_z1/patches/va15_stop_recovery_v252_20261027` |
| PC4 SSH | `unitree@192.168.123.164` |
| 板载项目 | `/home/unitree/rl_sar_b2_z1` |
| 板载 V251 独立版本 | `/home/unitree/rl_sar_b2_z1/diagnostics/va15_curve_entry_v251_20261026` |
| 本次板载原始记录 | `/home/unitree/rl_sar_b2_z1/outputs/va15_near_walk_yumjt3ry` |
| 已取回的完整诊断 | `/home/dong/rl_sar_b2_z1/outputs/diagnosis_v251_yumjt3ry_20261027` |
| SDK/仿真依赖旧树 | `/home/dong/rl_sar_b2_z1_old`，不是可以随便删除的备份 |

用户最后确认：B2 已实际趴稳，程序已退出，之后曾重启；本轮实验没有焊缝枪和软管。先前多次确认 Z1 放置安全、收臂路径无遮挡。**这些是历史确认，不是新帐号启动运动的授权。**

本轮最后的只读 SSH 能连接 PC4，所查控制进程没有匹配项；没有写板载文件、启动相机/Z1、释放厂商控制或启动机器人运动。若继续，要重新检查当前状态。

本地 Git：`main`，HEAD `68a9e979665197e3ef146ff5b0906ec402840986`。大量实际工作在未跟踪的 `patches/`、`outputs/`、`docs/` 中，不能只看 HEAD 判断版本。`.vscode/settings.json` 有原有修改，未动。不要 reset、clean 或整树覆盖。

磁盘交接时仅约 **6.6 GB 可用，99% 使用率**。优先复用已有 build，不要贸然新建大构建、复制训练日志或清理旧树。

## 3. 必须优先阅读的证据

以下路径均在本机，不需要先连接机器人：

1. 原始事件诊断：
   `/home/dong/rl_sar_b2_z1/outputs/diagnosis_v251_yumjt3ry_20261027/DIAGNOSIS.md`
2. 候选验证总表：
   `/home/dong/rl_sar_b2_z1/patches/va15_stop_recovery_v252_20261027/VALIDATION.md`
3. 发布状态：同候选目录 `qualification.json`、`BOARD_SYNC_REPORT.md`。
4. 更细的报告：同候选目录 `outputs/startup_analysis/` 下：
   - `REPORT.md`：真实关节目标、实测响应、频谱与拟合限制。
   - `TRAINING_CONTRACT_AUDIT.md`：真实训练归档与部署契约核对。
   - `Z1_GAIN_ORDER_AUDIT.md`：SDK 增益重置及板载文件核验。
   - `PLANTED_HIP_REVIEW.md`：新恢复参考的独立复核与反事实计算。
   - `VISUAL_STOP_REVIEW.md`：seq489 的逐项视觉证据、时效、下一候选建议。

原始数据目录含 `terminal.log`、`vision.log`、`control.csv`、`commands.csv`、`actor.csv`、`session.json` 和 `exchange/guide_packets.jsonl`、拒帧图片。三份 CSV 分别有 5127、5612、545 条完整记录。本次没有原终端最后关闭信息，板载 `telemetry_summary.json` 为空；不能声称程序正常关闭了日志。

## 4. 已确认的故障链路，不要重新误诊

### 4.1 腿部抖动与 TCP 目标跳变不是同一个问题

本次活动运行约 10.32 秒：

- actor 输出及实际发送的腿部位置目标约 **5.91 Hz** 往复变化。
- 腿部目标误差 P95 约 0.469 rad，最大约 0.769 rad；估计扭矩峰值约 122.7 Nm。
- 目标累计关节行程约 277 rad，实测约 14.3 rad；这是诊断累计量，不是机器人行走距离，也不是跟踪准确率。
- 首次 actor 冷启动约 40 ms，后续多数约 0.9 ms。CSV 约 47 Hz 是抽样记录，不是电机发送频率。
- 首帧力矩等效接管是连续的；约 1.65 秒接管结束后振荡仍存在，不能只靠加长接管过渡解释或宣称修复。
- `tcp_active`、`tcp_ever_active`、`rear_crossings` 全为 0，TCP 目标一直是机身系 `[-0.775, 0, -0.35]`。因此这次不是动态历史 TCP 开始牵引或目标换点造成的。
- 未发现动作历史串台、明显关节速度单位/符号倒置、持续推理卡死的证据。

用闭环日志拟合的简单一阶执行器模型，12 个关节 **0/12** 通过留出回放检验。不能使用这些拟合出来的延迟/带宽/增益去调实机。

目前只能确认策略请求与真实机械响应明显不匹配，尚不能唯一归因于增益、真实执行器内环、惯量、地面摩擦、安装或控制器竞争。成功 DDS Write 也不等于电机确认、不能排除竞争发布者。

### 4.2 第一次停车是视觉时效，不是正常终点

以首次接受 R1+Y 为 t=0：

- seq488：capture `441484826619`，publish `441647269842`，单位 ns。
- seq489：近处胶带和身份仍好，但完整曲线只有 21 点/138 px；不满足原 25 点/200 px 和前视行范围要求，包被判无效。
- t≈10.31954 s，上一合格帧年龄到 500.082714 ms，触发原 500 ms 时效停车。
- seq490 随即恢复合格，publish `441988525882`，只比停机评估晚约 **3.617 ms**。

相机约 6 fps，capture 到 publish 常约 160 ms，一次拒帧即可耗尽余量。不是固定 2 米/短时间行走限制，也不是正常看见胶带末端。

seq489 的近线证据仍很强：confidence≈0.99866、contrast≈0.64286、宽15 px、RMS≈1.115 px、关联偏差≈1.835 px。不能说整条胶带丢失；但也不能直接把 valid 改 true，包中的远前视占位 u 与真实曲线相差约 7.71 px，且违反多项原契约。

当前代码已经有：运行中单帧关联恢复、反光恢复、不外推的自适应前视、保留仍新鲜上一帧、已观测曲线最近前方点 fallback。不要把重复加同样机制当作新修复。关键在于完整包验证仍耦合近线与长前视，后级 fallback 救不了上游无效包。

### 4.3 B 和再次 R1+Y 都卡在支撑恢复，不是没收到按键

- t≈11.363 s 捕获后 actor 关闭；t≈13.901 s 进入 `VA15ControlledStop::Stage::Settle`，值为3。
- 随后约80秒、3430条记录一直 `stop_stage=3`、`fail_mask=4`，显示顺序 **FL/FR/RL/RR=1011**：右前脚 FR 的模型可信支撑为 false。
- 期间收到 7 次后续 R1+Y、25 次 B 上升沿，均不能完成状态切换。最终四脚 Ready 条件未通过。
- 其余量已较安静：h≈0.512m、倾斜≈0.019、角速度≈0.008rad/s、关节速度≈0.0007rad/s，数据 tick 持续推进。
- 这是模型支撑假设，不是已经证明实际右前脚离地；也不是本次 `lost` 字段或电机故障导致的停车。

注意：显示顺序 FL/FR/RL/RR 与原始 DDS 腿序 FR/FL/RR/RL 不同。历史的 `1101` 左后脚问题不是本次 `1011` 右前脚问题，不能混同。

重要假设：停止时前腿已较宽，实测髋约 FL +0.158 / FR -0.141 rad，但恢复目标仍为0、Kp450，产生较大横向预紧。准静态反算 FR 法向约128N、水平约106N，略超过模型约103N的可信上限。这是有证据的假设，**不是实机足端力真值或已证实的完整因果**。

## 5. V252 改了什么，哪些还不能算解决

候选目录：`/home/dong/rl_sar_b2_z1/patches/va15_stop_recovery_v252_20261027`。

| 文件 | 修改 |
| --- | --- |
| `production/priority_getdown.hpp` | 纯逻辑 B/9/锁存请求抢占辅助，不赋予物理支撑权限 |
| `production/rl_real_b2_z1_no_gun.cpp` | `PrioritizeGetDown()` 在 `UpdateNearGuide()` 之前执行；取消视觉运动、保留 B 意图、跳过近前视历史/投影工作，增加诊断 |
| `production/va15_controlled_stop.hpp` | 在原四足捕获通过时，冻结有界髋参考；事务式捕获提交 |
| `production/va15_independent_state.hpp` | 只增加水平力/可信上限/载荷下界/滞回时间诊断，原判定不改 |
| `production/trial_telemetry.hpp` | CSV 增加 B 锁存、抢占状态、髋参考、足端受力诊断 |

髋参考为 `capture_q + verified_pre_walk_standing_tau / stand_kp`，仅捕获时计算一次，与原名义角度和捕获角度的差均须≤0.15rad。不是每帧追随测量放松刚度，不放宽四足捕获或最终静止门槛。大腿/小腿、增益不变；首帧保持请求力矩连续。

反事实计算提示，新参考可能减少前髋额外水平预载约38～50Nm。但这些数值来自记录和近似的起步前承重参考，不是新控制器已在实机实现的效果。

**B 的软件抢占已实现；“最终一定能趴下”仍未验证。** 支撑模型若继续1011、参考越界或真实通信/姿态失效，候选依旧不会强制降低机身。5.91Hz 实机振荡也尚未解决。

没有改 VA15 权重、观测排列、EMA、实机电机增益、足端门限、500ms视觉时效，没有伪造接触/位姿/路径或让新鲜视觉自动重启。

## 6. 验证结果与不能滥用的测试

| 检查 | 结果 | 候选目录内证据 |
| --- | --- | --- |
| C++/native 离线 | 24/24 | `outputs/ctest_v252.log` |
| Python 相机/跟踪/启动器 | 62/62 | `outputs/python_v252.log` |
| 曲线/TCP/丢线/位姿失效/多摩擦 | 15/15 | `outputs/tcp_v252/summary.json` |
| 线尾/相机失效/伪线尾/B | 11/11 | `outputs/end_v252/summary.json` |
| 人工像素扰动 | 3/3 | `outputs/noise_v252/summary.json` |

物理回归使用真实 actor、MuJoCo、合成标定 RGB 和独立 IMU/关节估计；地面真值用于生成相机/评分，不是实机视觉验收。Z1 是仿真伺服代理，无实体枪/软管。常规仿真通过不能证明真实执行器响应正确。

另做的针对性复现没有成功，务必如实保留：

- `outputs/recorded_probe/`：把单帧实机姿态放入模型后，新旧版本均在捕获前失稳；接触/速度/受力历史不一致，结论不充分。
- `outputs/recorded_continuous1/`：温和连续髋命令偏移，新旧均通过，但脚没真正变成实机宽落脚，不能证明改进。
- `outputs/recorded_stepwide_final/`：4组基线/候选×真值/估计，均在固定PD单脚外展准备阶段侧翻，未到停止恢复阶段。全部 `inconclusive_initial_condition=true`，**不是4组修复通过**。限制见 `LIMITATIONS.md`。

因此 `qualification.json` 仍为：

```json
{
  "simulation_pass": false,
  "walk_release_allowed": false,
  "board_sync": false,
  "hardware_qualified": false,
  "startup_oscillation_resolved": false,
  "real_failure_reproduced_in_physics": false
}
```

`regression_suites_pass=true` 与上述 false 不冲突：前者是常规回归通过，后者是实机失败尚未针对性验收。

## 7. 策略契约：已核对，不要凭注释乱改

实际训练归档：

`/home/dong/b2_z1/logs/rsl_rl/b2_z1_amp/2026-09-28_15-07-31_va15_curriculum_from2500/model_5000.pt`

本轮仿真实际加载的导出 actor：

`/home/dong/rl_sar_b2_z1/patches/va15_stop_feedback_20261020/work/b2_z1_no_gun/VA15/policy.pt`

SHA-256：`8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098`。

- 70维观测 / 18维动作；8个actor参数张量与原 model_5000 完全一致。
- 纯 actor 70→512→256→128→18；训练关闭 empirical actor normalization，没有漏掉 actor normalizer。
- 动作 clip±5、scale0.25、EMA alpha0.20。
- 上次动作观测使用裁剪后的 **raw action**，不是 EMA 后的动作；保存的动作实现注释与实际 manager 路径有冲突，以实现/归档核对为准。
- 仿真 dt0.005s，decimation4，actor周期0.02s；腿部 PD160/5，训练延迟0～2物理步；机械臂是隐式PD160/5。
- 机械臂 policy 索引 `[8,13,14,15,16,17]`。
- IsaacLab 外部依赖未完整存入训练 source.tar.gz；当前安装实现的核对不是训练时依赖哈希证明。

仿真场景引用 `/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml`。该旧场景旁边的 `policy.pt` 不是本轮实际加载的 actor，不要混用。

## 8. 新发现的 Z1 SDK 问题：确认了顺序覆盖，未改执行器

V251 代码先把 SDK 命令缓冲 Kp 乘0.35，再切 `setFsm(LOWCMD)`。已检查的 SDK 在 PASSIVE→LOWCMD 内部调用无参 `setControlGain()`，重置默认增益，因此先前缩放被覆盖。

默认缓冲值 Kp `[20,30,30,20,15,10]`、Kd `[2000,2000,2000,2000,2000,2000]`。**物理单位未确认，不可直接等同于训练160/5，也不可凭数字大小调参。**

板载文件和本地审计 SDK 的 SHA-256 一致：

`ecfbdd917016de721983ad2cc0770adb110832f506cc9ef67b8c37a9f5f3eafd`

板载路径：

`/home/unitree/rl_sar_b2_z1/src/rl_sar/library/thirdparty/z1_sdk/lib/libZ1_SDK_x86_64.so`

只读 `readelf` / `ldd` 确认当前 V251 可执行文件解析到这个文件。不能据此恢复历史进程环境或证明下游固件物理增益。

现有 `kp_scale=.35` 日志仅代表意图，不是实际保留增益。**不要未经单独验证就把增益设置挪到 LOWCMD 之后**，这会真正改变电机命令。

主程序/已审计本地 SDK LOWCMD 没有自动追加逆动力学重力补偿；设置模型 addLoad 本身不等于已施加补偿。下游固件行为未验收，不可因此断言真实电机没有任何补偿。

Y 待机有效臂姿态并非 YAML 看上去的那一组：FSM 默认启用 `va15_prefit_arm_entry`，覆盖为 `[0,2.24,-1.27,-1.33,.04,0]`。本次记录的机械臂跟随该覆盖目标，不能把与 YAML 的差误诊成机械臂跟踪失败。

## 9. 下一帐号建议按这个顺序继续

### 第一步：只读接手，不启动机器人

1. 读本文、第3节列出的报告和 `qualification.json`。
2. 核对 Git 脏状态、磁盘及候选源码，保留所有旧版与原始记录。
3. 如果涉及板载，先重新询问机器人是否实际趴稳、程序退出、枪/软管未安装，以及是否授权同步；不要把本次交接当成同步或运动授权。

### 第二步：先补足停止恢复验证，而非直接放行走路

1. B 的视觉抢占与最终物理趴下是两层；B 应持续保留请求、阻止再启动，但不能用按钮代替足端支撑证据。
2. 检查新髋参考是否能真正消除水平预紧，同时维持高度/倾斜/速度/承重；别重复用“单脚固定PD外展导致准备侧翻”的无效初始化来给控制器评分。
3. 若离线无法可靠复现，要明确说明限制，再设计经用户确认的最小现场观测/测试；不要硬凑一个物理通过结果。
4. 在确认停止链路前，不要再反复让用户实机行走测试，也不要把 Ctrl+C、强杀或断电当作可靠站立停止方式。

### 第三步：单独排查 5.91Hz 的真实执行器闭环

1. 先核对真实加载库、实际保留的增益、SDK单位和控制器/发布者，不盲目提高Kp，不套用失败的一阶拟合。
2. 保持同一权重与观测，分离“待机→首帧→1.65秒后持续振荡”的响应；首帧连续不代表后续力矩温和。
3. 新增真实应用增益/力矩/时间诊断优先于改变控制行为；任何真实增益、补偿或滤波改变应另建候选并独立验证。
4. 不要把本次振荡直接归因于动态 TCP，也不要未经证据就换权重、改 raw-history 或再加多层滤波。

### 第四步：视觉近线与长前视解耦，另建候选

优先读取 `VISUAL_STOP_REVIEW.md`：

- 先用同权重/原始图像离线定位逐行扩展失败，记录概率图、拒绝原因及预处理/推理/后处理/发布/消费延迟。
- 优化真实处理延迟不改变契约，但本例只差3.617ms不代表只省4ms就能稳健解决；要覆盖负载和采样相位。
- 如果设计 `NearOnlyValid`，必须作为明确的新包类型，与 `FullPathValid`、前方可用几何、TCP历史权限和运动权限分开。不能填一个假的远点让解析通过。
- 近线新鲜不能自动刷新远路径或后向TCP历史，不能自动重新启动；允许退化近线继续行走属于新的控制契约，须单独验证。
- 终点识别仍需与普通丢线/遮挡/反光分开。目标是可信裂缝结束后站立停稳，不是短距离/短计时自动停，也不是永不停止。

## 10. 本机离线复查命令（不启动 DDS/UDP/相机/机器人）

在 **dong 本机终端**，不是板载终端：

```bash
cd /home/dong/rl_sar_b2_z1/patches/va15_stop_recovery_v252_20261027 || exit 1

# 已有构建的测试；native --offline-check 分支不初始化机器人接口
ctest --test-dir build --output-on-failure

/home/dong/anaconda3/envs/crack_vision/bin/python \
  -m unittest discover -p 'test_*.py'

./build/native_candidate_offline --offline-check
```

如确有源码修改，复用构建目录；先检查磁盘：

```bash
df -h /home/dong
cd /home/dong/rl_sar_b2_z1/patches/va15_stop_recovery_v252_20261027 || exit 1
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j2
```

重跑物理套件时输出目录必须是新的，避免覆盖交接证据：

```bash
cd /home/dong/rl_sar_b2_z1/patches/va15_stop_recovery_v252_20261027 || exit 1

/home/dong/anaconda3/envs/crack_vision/bin/python \
  run_tcp_suite.py "tcp_recheck_$(date +%Y%m%d_%H%M%S)"

/home/dong/anaconda3/envs/crack_vision/bin/python \
  run_visual_end_suite.py "end_recheck_$(date +%Y%m%d_%H%M%S)"

/home/dong/anaconda3/envs/crack_vision/bin/python \
  run_preview_noise_suite.py "noise_recheck_$(date +%Y%m%d_%H%M%S)"
```

套件运行期间不要再改生产源码；套件会比较前后哈希。这里只给离线命令，**不提供也不授权本候选的实机 --walk/--run**。

常用键位语义（不是启动指令）：A/0 起立，Y/1 策略待机，R1+Y/T 视觉跟踪启停，X/Space 停止请求，B/9 趴下请求，L1+X/P 紧急被动。紧急被动可能失去支撑，不能等同于安全趴下。具体入口必须先核验所选版本。

## 11. 给新帐号直接复制的提示词

> 请先完整阅读 `/home/dong/rl_sar_b2_z1/docs/archive/va15/HANDOFF_VA15_REAL_V252_20261027.md`，再读取其中列出的本地诊断和 V252 验证报告。我要继续 B2＋Z1 的 VA15 实机视觉部署。V251 曾出现 R1+Y 后高频抖动、视觉拒帧停车和右前脚模型支撑1011导致B无法趴下。V252只是本地未发布候选，未同步板载，常规仿真通过不等于本次实机故障修好。请保留现有代码和日志，先从停止恢复验证、真实执行器响应和Z1增益覆盖问题继续，不要盲调增益或伪造接触/帧时效。没有我重新确认状态和授权，不要同步板载、启动控制或机器人运动。请先说明你读到的当前进度和准备执行的第一步。

