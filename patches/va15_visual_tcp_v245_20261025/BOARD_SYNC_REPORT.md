# V2.4.5 板载准备与失败复盘（2026-10-25）

独立目录：/home/unitree/rl_sar_b2_z1/diagnostics/va15_visual_tcp_v245_20261025。已同步和板载离线构建；**未实机行走验收**。操作者确认旧会话实际趴稳、控制退出、无枪/软管，X/B 正常；工具未启动运动、Z1 服务或释放厂商控制。

## V2.4.4 失败会话

outputs/va15_near_walk_zdktsu6h 的 session 明确为 V2.4.4，bundle 7427de9c3697bc09c69ebbf64126bb4079283dd2bf955af7e3c15a5aa8507d2f。日志确认 10 次 R1+Y，4 次短暂启动约 1.94、2.17、0.87、1.18 秒后停止，6 次拒绝；没有形成连续行走。不能称为四次实机成功。

逐帧日志 646 条：行走阶段 48 条，其中 29 条有效、19 条因为连续曲线没覆盖固定前视行而无效。处理耗时中位约 118 ms、最大 159.10 ms，说明 V2.4.4 CPU 提速有效，但没有解决动态观测覆盖问题。例 seq262、264、326、327、447 仍有 200～222 px 连续路径和较高置信度，只因看不见固定 v=820.8 而拒绝；连续拒绝后旧有效帧超 500 ms，于是停止。第三次还存在数据恢复后时效/资格重置的竞争窗口。

没有记录 safety_fault/故障锁定，command/control/actor dropped=0、io_failed=false；X/B 后正常 GetDown → Passive，exit=0，操作者确认实际动作正常。本轮不改物理停止条件，也不能从这次正常退出推断所有历史停止问题已解决。

## 修改

- Python 在当前连续观测内选 look_v，优先 820.8；短一些时使用已观测曲线内部的点，最大 885.6，保留 12 px 观测余量。仍要求至少 200 px、25 点、原宽度/置信度，不外推未知点。
- 原生包校验使用同一前视行范围，身份比较用两条已观测曲线的公共行；缺共同支持则拒绝。近点/转向幅度/斜率限制不变。
- 8 px 等弧长重采样强制保留 near/look 精确锚点。离线首次复测发现一个包的 look 与发出的折线偏差 2.685 px，超过原生 2 px 容差；已从发包几何上修正，而不是放宽容差。最终 41 包全部按预期通过/拒绝。
- 仍用 V2.4.4 CPU 布局/受约束曲线拟合和发布后异步逐帧记录。

production 仅 near_visual_guide.hpp 改变；其 SHA 为 96010bd276d2a6241179cdfca6517ea7159f927f351d83fa00786314534aacfd。主 CPP、FSM、受控停止、首帧力矩接管、反馈/接触判断、TCP 目标生成等源文件与前版相同。policy、YAML、标定、增益、raw/EMA、70/18、9 cm 训练目标、400 ms/500 ms、六帧首次获取未改。

## 验证

- 本地新构建 C++ 回归 14/14，包含有界自适应行、公共行身份、120 帧长短观测交替、换线/缺共同支持拒绝，以及原有接管、停止、通信/故障等。
- 本地和板载 Python 各 39/39，涵盖原薄瓷砖缝、换线、过短曲线、曲线拟合、相位/诊断及新增自适应行、精确锚点。
- 同一批 41 张实际保存 RGB，旧算法 22 张可用，新算法 39 张可用，新增 17 张且原有效图像无退化；两张仍因观测太短拒绝。本地与板载结果一致。**这不是行走成功率**，也不是全视频逐帧网络重放。
- 本地/板载实际输出包：39 支持路径通过原生 Parse/Valid，2 短路径拒绝。使用明确标记的离线保存图像时序夹具，不授予实机六帧独立获取资格。
- 当前源文件对应物理仿真 8/8：曲线摩擦 0.5/0.8/1、直线、交替自适应前视、短失检、真失线和位姿失效停止；每例最终趴下，没有跌倒。合成像素不代表实机识别精度。
- 自适应例约前进 6.25 m（仿真），最大倾角约 1.9°，TCP 历史目标接入，X/B 最终趴下；源 SHA 和可执行 SHA 已记录 outputs/adaptive_final/summary.json，检查期间源文件未改变。
- 本板独立构建 native --offline-check 通过，VA15 黄金误差 1.14441e-05；无 DDS/UDP/机器人控制初始化。

离线保存图像报告：本地 outputs/real_saved_adaptive_anchors.json，板载 outputs/real_saved_adaptive_board.json；两者 .packets.jsonl 为结构校验夹具，不可用于实时交换目录。C++ 日志 outputs/ctest_v245.log，物理仿真 outputs/adaptive_final/summary.json。

## 指纹与保留

- 新 bundle：ec2aca11773be036b6b5bb636b7ff90246c4e545d0bc785585332b5d649a453d。
- 新私有二进制：1848c10ed76c58ee15f230c784795e274a116fd3c3828979e9a9d2e5336e2335。
- 主二进制保持 696756deac43676046b0073d6c7ee0c0eb1a79f0248f678f9df337013f8d9ba3。
- base.yaml 保持 909d04f2ce4820effd7a284ccff5955dc6a9c8fba921f0fe42a439627ced7164。
- VA15/config.yaml 保持 8faa51628b3704ce3e7e3a384f1c0eb8bb59b7dcf5a0069992069efed8aab8d1。
- policy.pt 保持 8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098。
- V2.4.3/V2.4.4 和主程序保留。hardware_qualified、precision_qualified、real_v245_motion_test_passed 仍 false。walk_release_allowed 仅为人工无工具平地实验入口，不是实机验收。

## 人工入口

PC4 NoMachine 桌面交互终端：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_tcp_v245_20261025/run_visual.sh eth0 --walk
```

无枪/软管，实际趴稳、旧控制退出，空场、防跌/急停准备好；大写 RUN → A 实际站稳 → Y 准备完成 → 确认黄/青点在预定胶带、持续 PATH IN RANGE → 一次 R1+Y 短走约 2～3 秒 → X 停止 → B 趴下 → 实际趴稳且 Passive 后 Ctrl+C。不推摇杆，不用反复按键或断电替代停止。若再次马上停止，请保留当前会话日志，不宣称本版一定成功。

前视行改动可见窗口 row=...；继承的本体/停止日志仍可能显示 V2.4.3，session.json 才标记 V2.4.5。相机与 Z1 同入口，不需窗口 R 或第二终端。没有自动恢复行走，真正失线仍停止。
