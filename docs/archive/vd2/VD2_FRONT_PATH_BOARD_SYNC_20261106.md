# 前视路径估计板载同步记录

2026-11-06，用户授权同步到板载并准备视觉行走。本轮完成安装和不运动检查，
未运行 `start_real.sh`、`run.py --run`、preflight 或任何接管/运动操作。

## 已安装

- 主机：`unitree@192.168.123.164`，`unitree-b2-pc4`。
- 正式目录：`/home/unitree/rl_sar_b2_z1_VD2`。
- 增量包（开发机和板载 `/home/unitree` 均有）：
  `/home/dong/rl_sar_b2_z1_VD2_front_path_20261106.zip`，64031 字节。
- ZIP SHA256：`73b1e9a67a1f2214f4b40c7d1c86fd72349d70637ab47bb9f9bddfefb6d1d85d`。
- 更新 ID：`vd2_front_path_20261106`，19 个文件纳入补丁核对。
- 修改前文件与原 MANIFEST：
  `/home/unitree/rl_sar_b2_z1_VD2/patch_backups/front_path_20261106_143204_347456`。
  旧日志、原标定、虚拟环境、编译产物均保留。
- 隔离检查目录：`/home/unitree/vd2_front_path_check_20261106_zjOOAQ`。
- 开发机证据：`outputs/VD2_FRONT_PATH_BOARD_20261106_tKFQfv/`。

前视估计器及视觉控制器来自已经做过 MuJoCo 120 秒验证的脚本。
板载新增共享 IMU 重力缓冲：主进程按实际 B2 host receipt 时间发布重力方向，
视觉进程选择图像接收之前最近一次且不超过 100 ms 的真实样本。
不发布/积分机身位置、速度、航向，不使用后视深度设定前视平面。
前投影高度配置 `front_nominal_base_height_m=0.55` 是仿真沿用的名义假设，
**尚未现场标定**。不声称 IMU 与相机硬件曝光同步。

一起安装此前已在板载隔离验证的 CPU deadline 修复：按需 JPEG 解码、
中心线包围区域计算、PASSIVE OpenMP 等待与单线程 BLAS。
没有放宽图像年龄 1 s、前后主机接收偏差 40 ms，未伪造接触或时间戳。

政策/IO/姿态准备路径未变。安装前后哈希一致：

| 文件 | SHA256 |
|---|---|
| `assets/policy.pt` | `51eda90b2b77204ef66d949523486250e88c7062aeb1e86f0dc5478924291c78` |
| `python/control.py` | `51ad5abe3a1318cd198ddbddf30d9090c9ac0875c0b4710b6dae5cb4a4c364dd` |
| `native/io.cpp` | `f60151016421f9af5b7515598c3f2c881b2ef766cfe3b27cb5df23dfa885cef8` |
| `config/front_rgb.yaml` | `f75d3b26d7de2260117430addc0d716da73184bf4dea2bc49a8c1a086c8e61ae` |
| `config/rear_d435i.yaml` | `5063518d860faab507c4542337ca0673f0fa433458de2e1fea8a63586b819ce9` |

## 检查结果

- 本地完整候选重新编译及 CTest 通过；Python 离线检查 77/77 通过。
- 板载先在隔离副本安装、自检 77/77 通过，再安装正式目录。
- 正式目录 `python run.py --offline-check`：77/77 通过，4.416 秒。
  原生契约检查不打开 DDS/UDP/相机。
- 正式 MANIFEST 所列 903 个文件全部核对一致，无哈希差异。
- 12 秒双相机预览通过，无控制接管；前 RGB 有持续更新，D435i 序列号
  `242322073586`，逆 Brown-Conrady 校正正常。
- 相机预览数据在隔离目录的
  `rl_sar_b2_z1_VD2/runs/20261106_143036_658836`。
- 最后检查没有 `z1_ctrl`、主控制 Python 或 8071/8072 UDP 服务运行。
  本次不涉及 C++ 修改，板载无需重新编译。

## 真实图像回放：通过项和未解决项

输入均为原正式会话 `runs/20261106_123235_704827` 的保存图像和
`control.jsonl.gz` 中对应 IMU，图像接收时间完整保留。
`replay_front_path.py` 只在历史时钟计算几何有效性，不把旧图当实时帧发送。

| 图像组 | 前后检测 | 前视估计＋控制几何 | 板载完整回放单包 |
|---|---|---|---|
| `latest_valid_` | 有效 | 有效，`front_only_cubic_kalman` | 294.23 / 299.00 / 305.29 ms |
| `working_pair_` | 有效 | 拒绝：外推截距超范围 | 257.41 ms |

`latest_valid_` 使用图像之前 14.90 ms 的真实 IMU 样本。
`working_pair_` 使用之前 14.72 ms 的真实 IMU；可见近端约 1.31 m，
拟合截距 -0.816 m 超过原 0.8 m 上限，前瞻 y=-0.219 m。
因此此次没有以“所有真实图都通过”验收，也没有为了通过而提高阈值。
仅为范围拒绝增加数值原因日志，算法行为不变。

上述计算时长不含实时 SDK/USB 采集、当前主程序 actor 负载及连续行走；
**没有证明旧帧停车已在实机消除，也没有完成末端精度验收。**
原 MuJoCo 120 秒 TCP 投影 P95 约 12.8 cm，尚未满足 5 cm 要求。
后视目前仍保留原末端保持与失效停车联锁，视觉进程仍按前后成对处理。

## 现场命令

在板载终端执行：

```bash
cd /home/unitree/rl_sar_b2_z1_VD2
source .venv/bin/activate
bash start_real.sh --interface eth0
```

A 起立，待站稳后 Y 到准备位，待 ready 且视觉几何有效后 R1+Y 跟踪。
再次 R1+Y 或 X 停止；B 沿原流程收臂、趴下；L1+X 为原紧急停止。
首次观察可选择对齐的较直线段；若不能进入跟踪，保留 `[vision]` 的实际拒绝原因，
不要据此改写帧时间或盲目增大增益。

下一步取新会话中 `vision.jsonl`、`control.jsonl.gz` 和保存图像核验：
前视估计是否接受、实际可见范围/外推距离、IMU接收间隔、真实图像年龄，
以及视觉转向、机身响应与末端目标的对应关系。
