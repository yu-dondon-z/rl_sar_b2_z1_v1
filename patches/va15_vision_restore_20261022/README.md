# VA15 视觉观察版（2026-10-22）

恢复上一版实机视觉模块，但暂不把视觉输出接入执行器。与当前 V2 无视觉
4 秒前进测试是两个独立功能：加入相机和识别，不默认为视觉轨迹行走。
原始无视觉候选目录、原始部署入口、VA15 权重和所有控制参数均保留。

## 恢复来源

从工作站 `rl_sar_b2_z1_old` 原样恢复：

- B2 `FrontVideoClient` 光学 RGB 相机桥接脚本，不是 RealSense 深度。
- RHACrackNet `RHANet_dep` 原训练权重，两端 SHA-256 相同。
- 地面 ROI、底边抑制、近端宽度筛选、连续六帧同路径获取和跨帧路径锁定。
- 这台 B2 2026-09-15 的内外参文件，未使用当前板载的零内参占位文件。
- 标定射线接口 v2、纯几何投影和路径锁定回归测试。

桥接脚本 SHA-256：`d391eb85d1a9eb1354b5f5fa4b35b00bb5e5fe74b5fd54475d8cd412bbad6a5b`。
标定 SHA-256：`525a898c45693a2b00839a6d0bc2cc3e4c9d629f6687ff80dde9a850cbfdb4e0`。
网络权重 SHA-256：`f1eea726c85773c3ddb71649bdae097eff6ca7693c7f36ad488fa9f4643bdb80`。
恢复历史标定不等于重新完成现场标定；相机分辨率、固定安装和地面假设仍需检查。

## 板载操作

在 NoMachine 的板载终端操作。纯观察模式不接管 B2，不启动 Z1 控制器，也不
发布电机命令；读取图像需要前相机服务。单独观察时保持趴稳即可。

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_vision_restore_20261022/run_vision.sh --offline-check
bash diagnostics/va15_vision_restore_20261022/run_vision.sh eth0 --preview
```

按 Q/Esc 关闭图像，或 Ctrl+C 退出纯观察程序。没有 DISPLAY 时可添加
`--headless`，查看输出目录的 `exchange/latest_overlay.png`。

若需站起来看胶带（人工操作，条件同此前静态测试）：

```bash
bash diagnostics/va15_vision_restore_20261022/run_vision.sh eth0 --standby
```

同一个终端先启动视觉窗口，再进入原 V2 的 RUN 确认。趴卧视野 INVALID
不阻止静态起立；A 起立，Y 待机，观察是否一直锁定同一条黑色胶带。B 趴下，
实际趴稳并显示 Passive 后 Ctrl+C。旧 Z1 服务仍由原启动器管理，不另开窗口。
退出控制后只清理本次创建的相机子进程，不杀进程组，不重启/结束 Z1。

可选 `--forward-test` 仅把相机观察窗和原来的无视觉受限短测一起启动。
它仍然只有前进目标 0.15 m/s、每进程一次、最多 4 秒前进请求，不是物理停止
时间保证。松杆/X 请求停止，B 请求停稳后趴下；现场防倾倒和已验证急停要求
与原受限测试完全相同。Q 只关相机，不停止机器人；控制中不能用关窗替代 X/B。

**本版本不支持 R1+Y/T 视觉跟随。** 摄像头的 LOCKED PATH 只说明二维识别已
锁线，不能当成三维轨迹验收或启动许可。每次生成独立目录：
`outputs/vision_observer_20261022_XXXXXX/exchange/`，不复用旧 `/tmp` 交换目录。
新观察器使用 `--preview-only`，不写 `latest_trajectory.json`，不会串入旧控制器。

## 为什么不直接套用旧的 --run --vision

当前 V2 的无视觉验证使用独立的实验状态估计，而旧视觉历史点转换依赖另一套
姿态/里程计来源；原厂 sport 数据在低层接管后不能假设仍然有效。此外，旧视觉
发标定射线 v2，重置后基础程序接收固定平面 XYZ v1。直接拼接会导致拒绝帧、
坐标/高度错误，或绕开当前只前进、不转向、4 秒截止的边界。

下一阶段须把射线和同时间戳的当前姿态/高度进行地面投影验证，再离线验证
路径历史、TCP 目标连续接入、限速转向、丢线停止与 B 趴下；不能通过修改
接触阈值、伪造 sport_fresh 或删除当前截止来“接通”。目前腿部目标/反馈偏差
仍需继续分析，不能把这次短测试当作持续自主跟随的动态验收。

## 离线验证

`--offline-check` 只导入接口、校验文件和加载网络，对合成空白图推理；不初始化
相机/控制接口。合成推理通过不代表现场识别精度或实时性已验收。
摄像头进程单独限制为两个 CPU 线程，不给 native 控制进程设置线程环境变量。

```bash
python3 -m unittest discover -s diagnostics/va15_vision_restore_20261022 -p 'test_session.py' -v
RL_SAR_RHACRACKNET_ROOT=/home/unitree/RHACrackNet/code \
  python3 diagnostics/va15_vision_restore_20261022/scripts/test_b2_builtin_front_path_lock.py
```

几何测试使用纯头文件 `geometry/test_vision_ground_projection.cpp`，覆盖趴卧高度、
上视射线、无效四元数、过期/不同步帧的拒绝；不是实机定位精度证明。
