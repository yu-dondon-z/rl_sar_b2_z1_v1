# VD2 实机视觉超时后续记录（2026-11-06 13:05）

## 当前结论

用户确认板载联网、控制程序关闭后，本轮仅做日志读取、隔离候选修改、录制图
复算和离线测试。没有启动 z1_ctrl、主控制、SDK 状态通道、相机采集或运动。
板载正式目录 `/home/unitree/rl_sar_b2_z1_VD2` 的程序、配置、动态库和日志未修改。

12:32 会话中的两次 R1+Y 都已经进入 tracking，之后因前视真实接收年龄超过
1 s 锁存停车。ready 的完整识别处理约 548–708 ms、中位数 633 ms，上一包
在等待下一包期间过期。后视多数已覆盖 `x=-1.02 m`；本轮不再修改后视几何
放行规则，不盲调站立/行走/Z1 增益，不延长帧时效。

修正已完成并打包，正式目录尚未安装；下一步是现场选择安装后验证真实 ready
与行走阶段的连续帧年龄、包间隔、X/B 恢复，以及实际 TCP 地面投影误差。
离线加速不等于本次实机故障已经修好，也不等于达到 5 cm 末端误差目标。

## 文件与改动

开发源在 `/home/dong/rl_sar_b2_z1/deployment/vd2_real/`：

- `python/cameras.py`：前视保留最新 JPEG 的独立副本，仅选中配对帧才解码矫正；
  请求上限 30 Hz；原始帧号和独立接收时间不变，包耗时包含解码。
- `thirdparty/rhacracknet/crack_trajectory/centerline.py`：把清理/连通分量/图搜索
  限制在带完整上下文的前景矩形内，保留图像边界、扫描对齐和全图坐标舍入。
- `run.py`、`start_real.sh` 与独立视觉工具：设置 `OMP_WAIT_POLICY=PASSIVE`、
  `OPENBLAS_NUM_THREADS=1`，减少采集、推理和小规模几何计算之间的资源争用。
- `tools/benchmark_recorded_vision.py`：新增只用历史图像的叠加负载工具，actor
  计算结果丢弃，不调用 IO；输出明确区分计算和、采集年龄与未验证实机项目。
- `tests/test_thinning.py`、`tests/test_realtime_vision.py`：新增全路径等价、
  JPEG 矫正/缓冲独立/原始时间保留/坏图与分辨率拒绝测试。

保持 actor、RHANet 权重、360×640 输入、0.90 阈值、内外参、0.20 m/s、增益、
状态转换、native IO、1 s 帧时效和 40 ms 主机配对偏差不变。没有采用神经网络
trace/freeze、固定 CPU 核号或修改系统电源策略；相关实验没有进入候选。

## 证据

本地目录：`outputs/VD2_VISION_DEADLINE_20261106_FaV0ai/`。

- 实机原始副本 `20261106_123235_704827/`，launcher 副本
  `launcher_20261106_123234_011569/`。
- `profile_recorded_visual.py`、`profile_neural_variants.py`、
  `profile_ready_geometry.py`、`compare_recorded_geometry.py` 均不初始化硬件 IO。
- `board_benchmarks/` 保留每次逐包耗时和 summary，包括最初测试夹具工具姿态维数
  错误导致的失败样本 `baseline_load.jsonl`，修正为原 7 维工具位姿后的测试使用
  `baseline_load_v2.jsonl`，没有拿失败样本当作性能结果。
- `verified/rl_sar_b2_z1_VD2/` 是实际增量包安装到上一个正式完整副本后的结果。
  `candidate/` 是较早完整组装的实验副本，不用于下一版补丁的基准。
- `local_verified_offline.log`、`board_verified_offline.log`：实际 ZIP 安装的副本
  本地/板载均 71/71 通过，native contract 不打开网络或运动通道。

录制图复算中的第一次有效图在起立，第一次覆盖工作点图在 stand_settle，
最后保留图在机身仍站立的 arm_lower。没有完整的连续行走图像序列；不要把这些
文件名解释成已经保存了实际稳定焊接跟踪样本。PNG 深度量化到 1 mm。

同一板载，60 次旧版叠加 30 Hz JPEG/矫正及 50 Hz FK/几何/actor/压缩工作：
中位数 491 ms、最大 611 ms、相邻两包计算和最大 1159 ms。
完整候选 100 次：中位数 310 ms、P95 333 ms、最大 382 ms、相邻两包计算和
最大 760 ms，100/100 对检测有效。三个保留图组共 28 个组件的清理掩膜、骨架、
顺序路径、航点及长度与旧版完全一致。测试未包含实际 SDK、深度对齐、采集和
传输延迟，计算和不能标作真实最坏帧年龄。

## 包与板载状态

ZIP：`/home/dong/rl_sar_b2_z1_VD2_vision_deadline_20261106.zip`，50916 字节。
SHA256：`29a6a668b6a4d20dc879b8660208f5516abcc0fd4780cf97461a277738bc091d`。
已复制到板载 `/home/unitree/rl_sar_b2_z1_VD2_vision_deadline_20261106.zip`，哈希一致。

板载正式文件与补丁所有 before 哈希完全一致。真实安装器仅运行在隔离副本：
`/home/unitree/vd2_vision_profile_20261106_G9DnnV/verified_deadline/rl_sar_b2_z1_VD2/`。
其中备份为 `patch_backups/vision_deadline_20261106_130357_622081/`。
不是正式部署目录的备份；不要误认为正式板载已经安装此修正。

ZIP CRC、完整清单、重复安装幂等、本地/板载实际安装检查通过。模型、control.py、
vision_start.py、相机/部署配置及 native 动态库与上一版一致。安装不要求重编译。

现场安装命令和离线/现场验证方法详见
`deployment/vd2_real/docs/VISION_DEADLINE_20261106.md`。先完成安装并离线检查；
机器人后续启动与按键运动由现场决定。后视外参仍为先前近似安装配置，不能把
几何检测有效当作外参精度或实际 TCP 视觉闭环已经验收。

收尾再次读取状态时，开发机 enp5s0 已变为 DOWN，无 192.168.123.99 地址，SSH
转向代理路由而关闭连接。此时 ZIP 已成功传输、板载隔离安装与 71/71 检查已经
完成；没有在正式目录执行安装。下一次继续应先只读核对链路、正式文件哈希与
实际运行进程，再做后续操作，不假设有线连接仍在。
