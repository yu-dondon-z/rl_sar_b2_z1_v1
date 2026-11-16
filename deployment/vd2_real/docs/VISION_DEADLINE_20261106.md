# VD2 完整视觉负载下的帧时效修正候选（2026-11-06）

## 本轮真实故障

`runs/20261106_123235_704827` 和对应 launcher 日志证明：两次 R1+Y 均已进入
tracking，随后因 `front capture is stale or in the future` 锁存停止。ready 的
有效包计算中位数 633 ms，范围约 548–708 ms。后视大部分已经覆盖工作
`x=-1.02 m`，不能再把本次直接停车归结为后视未覆盖或组合键无效。

关键是相邻两次结果之间的旧包年龄。即使每包计算小于 1 s，正在使用的上一包
仍会在下一包到达前超过 1 s。保持真实接收时间及原 1 s 限制，降低完整计算负载。

## 改动

1. 前相机采集线程只保留最新 JPEG 的独立副本、原接收时间和帧号，SDK 请求上限
   30 Hz。仅对实际选中的配对帧解码和原分辨率矫正，避免反复处理最终会丢弃的
   1920×1080 图像。矫正参数、插值与分辨率不变；解码失败不能当作有效图。
2. 中心线的清理、连通分量筛选及图搜索限定到前景包围区域。保留完整形态学
   上下文、真正图像边界与组件扫描对齐，先恢复全图整数坐标再做 float32 平滑。
   掩膜、骨架、顺序路径、航点、长度均与旧实现逐项对比。
3. `OMP_WAIT_POLICY=PASSIVE`，`OPENBLAS_NUM_THREADS=1`。数值库不忙等争抢 CPU；
   小规模几何求解使用一个 BLAS 线程。torch 仍 2 线程，OpenCV 仍 1 线程。
   在 shell、Python 主入口及独立视觉工具中设置，spawn 子进程继承。
4. 视觉包新增 `front_preparation_ms`，`processing_ms` 包含选中 JPEG 的解码矫正。
   既有前后独立接收时间、帧号和日志保存方式保留。

没有更换神经网络/actor 权重、输入 360×640、0.90 分割阈值、相机内外参、
0.20 m/s、站立增益、A/Y/B 与停止状态机、C++ IO。没有用推理时间刷新旧图，
也没有放宽配对偏差 40 ms 或帧年龄 1 s。

## 离线证据与边界

开发机证据：`outputs/VD2_VISION_DEADLINE_20261106_FaV0ai/`。
板载隔离候选：`/home/unitree/vd2_vision_profile_20261106_G9DnnV/deadline_candidate/`。

真实图像来自上述会话的三个保留组。`valid_pair_` 在起立，`working_pair_` 在
站立稳定阶段，`latest_valid_` 在机身仍站立的收臂阶段；不能把它们标作实际连续
行走帧。毫米 PNG 深度仅用于离线复算，未改写原始日志。

| 板载录制图计算实验 | 次数 | 单包中位数 / 最大值 | 相邻两包计算和最大值 |
|---|---:|---:|---:|
| 已安装版本 + 30 Hz 图像工作 + 50 Hz FK/几何/actor/日志负载 | 60 | 491 / 611 ms | 1159 ms |
| 限定中心线区域 | 60 | 400 / 528 ms | 1002 ms |
| 再按需解码，尚未调整数值线程等待 | 60 | 397 / 500 ms | 991 ms |
| 完整候选，PASSIVE 等待 + 单线程 BLAS | 100 | 310 / 382 ms | 760 ms |

最后 100/100 对检测有效。三个真实图像组的 28 个候选组件，完整分辨率的清理
掩膜、骨架、顺序路径、航点及长度与旧实现完全一致。仅调整等待策略、尚未限制
BLAS 的独立 100 次测试仍有 944 ms 的相邻计算和，因此保留完整线程配置。
固定 P 核实验不是最终方案，没有硬编码 CPU 核号或修改系统电源设置。

本地与板载隔离候选的离线检查均为 71/71 通过，原生契约检查不初始化 DDS、
UDP、相机或运动。新增对比覆盖形态学边界/随机形状/锚点、JPEG 矫正像素一致、
坏图与分辨率拒绝、SDK 缓冲副本与接收时间保留。

这些是离线重复图加计算负载，**不包含真实 SDK/深度对齐负载、相机采集延迟、
连续行走、末端精度验收**。760 ms 只是计算和，不是实测最坏帧年龄，不能声称
本次实机故障已经修好。未启动机器人控制或运动。当前生产目录保持原版本，
候选与安装包分开保存。

## 安装与后续验证

增量包：`rl_sar_b2_z1_VD2_vision_deadline_20261106.zip`。在板载，控制程序关闭时：

```bash
cd /home/unitree
unzip -n rl_sar_b2_z1_VD2_vision_deadline_20261106.zip
cd /home/unitree/rl_sar_b2_z1_VD2
.venv/bin/python ../vd2_vision_deadline_20261106/install.py --target .
.venv/bin/python run.py --offline-check
```

安装器逐文件核对当前哈希，遇到意外本地修改拒绝覆盖，原文件和清单保存到
`patch_backups/vision_deadline_*`；不覆盖日志、模型、相机配置或 C++ 动态库，
不需要重新编译。已有控制程序必须退出后再安装；安装本身不会启动控制。

离线复算工具（不打开相机、SDK 通信、运动控制）：

```bash
.venv/bin/python tools/benchmark_recorded_vision.py \
  --session runs/20261106_123235_704827 --prefix latest_valid_ \
  --iterations 100 --load --front-demand --passive-numerics \
  --output /tmp/vd2_recorded_vision_新文件名.jsonl
```

输出文件必须不存在。模拟采集工作只提供 JPEG/矫正负载，actor 输出仅计算后
丢弃，不发送命令。历史图像不会投递给真实控制循环，也不会改成新时间戳。

后续现场仍用 `bash start_real.sh --interface eth0`。A/Y 到 ready 后先观察真实
完整包的 processing_ms、前后 age、后视覆盖及连续包间隔，再由现场决定 R1+Y。
关注 ready/行走阶段持续时效和 X/B 恢复，不能拿趴姿快速拒帧当成识别性能通过。
