# V2.4.4 板载视觉修复准备（2026-10-25）

状态：独立目录已同步、板载编译和离线检查通过，纯相机预览完成；可由操作者开始无工具短程实验。**V2.4.4 尚未实机行走验收**。本轮未自动起立、行走、释放厂商控制或启动/停止 Z1；用户确认已实际趴稳、控制退出、无枪和软管。本目录继承的 PREPARATION_REPORT/OFFLINE_REPORT/HANDOFF_REPORT/HOTFIX_REPORT/REAL_BLIND_REVIEW 是旧核心历史，当前操作只看 README、qualification 和本报告。

## 实机日志根因

V2.4.3 会话 outputs/va15_near_walk_1rt8m301，bundle 929dce5e744b8f4e0668abe1a45952cca927693c0c8e8d5c700b6520e85450d4。只读复制在本地前版 outputs/review_visual_20261025 下。

- R1+Y 收到 9 次，其中 3 次启动，约持续 2.75、0.15、14.64 秒；6 次因当前包无效/过期或未积累六帧被拒绝。不是未收到遥控器，也不是一次性会话没有重置。
- 有效图像相邻更新中 24 处前一图像在后一包发布时超过 500 ms。264→265 为 538.754 ms，265→266 为 553.067 ms。299→300 发布时年龄 486.549 ms，原 Inbox 25 ms 轮询/解析再耗时即可越界。越界清空 latest，诊断出现 sequence=0；恢复帧从 qualified=1 重新积累，不能理解成全部是网络未检测到胶带。
- 第三次较长行走结束前，seq405 后出现近段拟合不规则拒绝。保存的 rejection_000406_after.jpg 里真实胶带为曲线；原近处强制直线拟合 RMS=6.754879 px，超过 6 px；同一批点二次模型 RMS=2.593211 px。
- no safety_fault/no latched deployment fault，command/control/actor 无丢记录或写盘错误。X/B 已完成支撑停止、GetDown → Passive、exit=0；用户确认实际动作正常。不能用这次正常停止宣称所有历史 B 等待根因均已解决。

## 修改范围

Python 视觉端的 CPU channels-last 内存布局、受约束二次近段拟合、保留曲线阶数的背景光流、发布后的逐帧异步诊断。详见 README。

production 内所有 C++ 和配置逐文件 SHA 与 V2.4.3 相同；VA15 model_5000、70/18、raw/EMA、PD 增益、Y 姿态、接管、TCP、六帧获取、500 ms 时效、400 ms 处理限额和足端停止约束均不改。不使用 optimize_for_inference/量化/裁切/降分辨率，不切换视觉 checkpoint，不把过期图片时间戳刷成当前时间。

## 已完成验证

1. 相同 production 源文件重跑前版 C++ 回归 13/13；这是源文件完全相同的控制核心复验，不是声称新版全套物理仿真重跑。既有物理验证引用保留前版路径。
2. 本地和 PC4 Python 视觉/诊断测试各 34/34，包含小弯接受、过大弯/锯齿拒绝、薄瓷砖缝拒绝、独立换线拒绝、相机相位和诊断队列测试。
3. 板载 13 张保存图像，原内存布局与 channels-last 的稠密概率最大差约 1.01e-5，近点/远点选择相同；输入/权重未改。最终测量 full-wrapper 中位数 139.18→80.76 ms，重复 40 次 P95 90.06 ms、最大 94.89 ms。这里的同路径对照是同一个新版几何跟踪器下比较 CPU 布局；前后版本几何差异另以第 5 项核对，不能混称整个视觉算法没改。
4. 真实保存图像输出包经原生 Parse/Valid 检查：11 个受支持路径通过、2 个短路径拒绝；使用离线图像时序夹具，不是实机获取资格。报告和包为 outputs/final_saved_frames.json、final_saved_frames.packets.jsonl。
5. rejection_000406_before/after 用同一 LongTracker 顺序输入：前帧直线 RMS=4.4092 px；后帧二次 RMS=2.5932 px，同线关联差 16.001 px，背景光流有效，两帧 long_valid=true。没有靠放宽 6 px 直线残差上限蒙混通过。
6. 本板单独编译、native --offline-check 和包装入口校验通过；actor 黄金误差 1.14441e-05。未初始化机器人控制接口。

## 纯相机预览

只运行本版 --preview --headless 约 18 秒，使用 B2 自带光学 RGB；无 run_control/低层命令/厂商控制释放。会话 outputs/va15_near_preview_437whvlr。定时结束码 124 是预定结束预览，不是控制崩溃；仅关闭自有相机子进程。

- 逐帧记录 149，valid=144；开始五帧为原六帧获取过程。
- 取图收到到轨迹发布：中位 93.93 ms，P95 130.90 ms，最大 150.76 ms。
- 前一有效图像在新包发布时的年龄：P95 236.08 ms，最大 248.93 ms，没有超过 500 ms；诊断丢任务 0。
- 这是趴卧静态、无控制负载的相机检查，不证明站立视野、动态抖动、转向误差、后方 TCP 精度或控制负载下的实时性合格。下一轮必须保留 guide_packets.jsonl 和控制日志一起核对。

## 指纹、保留和人工操作

- 新独立目录：/home/unitree/rl_sar_b2_z1/diagnostics/va15_visual_tcp_v244_20261025。
- bundle：7427de9c3697bc09c69ebbf64126bb4079283dd2bf955af7e3c15a5aa8507d2f。
- 私有二进制：2de506da0d332c66b4354f39b1f3e3eeb5ae60cdb4b98b1ef1d4b155950d0b3f。路径/构建产物不同，不声称二进制与 V2.4.3 字节相同；控制源文件是相同的。
- 主二进制保持：696756deac43676046b0073d6c7ee0c0eb1a79f0248f678f9df337013f8d9ba3。
- policy.pt 保持：8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098。
- 旧 V2.4.3 候选、主 YAML、相机标定和策略均保留。hardware_qualified、precision_qualified、real_v244_motion_test_passed 均 false。

PC4 NoMachine 交互终端：

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_tcp_v244_20261025/run_visual.sh eth0 --walk
```

大写 RUN → A 实际站稳 → Y 机械臂准备完成 → 确认黄/青点确实在目标胶带并持续 PATH IN RANGE → R1+Y 短走约 2～3 秒 → X 停止 → B 趴下 → 实际趴稳、Passive 后 Ctrl+C。先保持无枪/软管，不推摇杆，防跌/急停与空场准备好。

不因启动拒绝而连续重复按 R1+Y；保留拒绝原因，等待六帧、正确线路和停止就绪后明确重试。视觉丢失不会自动恢复行走；短测正常后再延长以观察 cached TCP active=1。不得把本报告当成硬件或作业精度认证。
