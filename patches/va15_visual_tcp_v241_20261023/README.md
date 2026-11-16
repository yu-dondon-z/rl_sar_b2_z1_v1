# VA15 视觉 + 后方 TCP V2.4.1（2026-10-23）

本版修复 V2.4 实测中“启动约 85 ms 后误判位姿失效”和“模型短缺使 B 永久锁住”的两个问题。以下原 V2.4 结果只作历史参考；新验证与板载状态见 qualification.json 和 HOTFIX_REPORT.md。

独立实验版本。保留 V2.3；不替换主程序、VA15 model_5000 权重、标定、关节映射或增益。
未做实机运动验收。仿真结果不等于硬件安全认证。

V2.4 板载及仿真记录保留在相邻 `va15_visual_tcp_v24_20261023` 目录。V2.4.1 的本轮实际结果以本目录 qualification.json、native_build_visual/bundle.json 和 [修复报告](HOTFIX_REPORT.md) 为准，不继承旧版实机放行状态。

## 本次更改

- 行走时的同线关联从固定 12 像素，改为依胶带实际 RGB 宽度和背景运动证据使用 24～60 像素范围。保留宽度、对比度、网络广泛支持和曲线连续性；不接受任意瓷砖缝。
- 连续行走中允许一次短暂漏检复用上次仍新鲜的路径，但不会刷新旧图像时间。原 500 ms 数据过期保护、错误会话/乱序/篡改检查保留。
- 起步仍需六帧采集；已在跟随时，只需一帧重新关联到原线的好图像即可恢复有效输出，不再把每次漏检都当成六帧新采集。
- TCP 不再固定：已标定像素曲线通过平面相交得到地面点，按采样时的局部位姿存入缓存；B2 走过后，再把后方历史路径点转换回当前机身坐标，送入原 VA15 的 target_commands 观测。仍是同一个 18 维全身策略，无独立 IK 或关节偏置。
- 本体引导补偿由局部速度估计得到的运动方向/机身朝向偏差（0.8 秒低通、角度限幅 0.30 rad），避免单纯“机头对准前方点”而保留平行偏移；yaw 指令幅值与变化率仍为原值，不新增侧向电机指令。
- 实际引导使用地面点相对“机身中心后方 0.775 m 名义 TCP 参考点”的方向，减少单纯机头追线导致的后方不可达。这优先保证后方 TCP 的可达性，不代表机身中心恰好在胶带上。使用有效米制参考时由它的 ±0.45 rad 范围判断，不再被旧相机中心射线 ±0.40 rad 硬限制误停。相机显示窗口放宽为近/远 ±0.60/0.65 rad。
- 使用完整 IMU 姿态旋转，局部平移由已有 IMU/关节无滑移估计提供。只在本地坐标系保存，不依赖 GPS、全局定位或失效的 sportmodestate。
- 迈步时允许滤波器在协方差、时间戳与源健康合格的条件下，使用最多 250 ms 的短时惯性预测；不要求每个摆腿帧都有足够足端更新，不伪造接触，超出时仍停止。
- 起步保持接管时目标高度；后方历史路径可用后，以最多 0.01 m/s 过渡到训练使用的离地 0.09 m，不在起步时突然压低至 1 cm。按本权重保存的 `env.yaml limit_ranges` 使用 x=[-1.00,-0.55]、y=±0.15、z=[-0.55,-0.30] m，而不是旧部署误用的 z 下限 -0.38。位置指令最大变化率 xyz=[0.04,0.04,0.02] m/s，4 秒平滑接入；来源见 config/va15_training_contract.json。
- 后方初始没有被相机看见，故先保持原目标。需要约走过“相机可见前向距离 + 0.775 m”后才出现 active=1；不反向外推一条未经观察的线。
- 只使用同一帧连续观察到的线段，不把不同帧点随意连起来。缓存上限 192 帧/45 秒；位姿源重置会清掉路径，停止运动，不自动恢复。
- 支持停止已捕获后，四脚模型接触的短暂掉线不再永久卡住站姿恢复。姿态/反馈有效且安静时缓慢完成站姿回归；14 秒资格等待不再不可恢复锁死。初始捕获和最终允许趴下的四脚支撑条件不变，绝不由超时强制趴下。

## 限制

当前是**单目平地投影**，不是 RGB-D：地面假设为局部 z=0。相机沿用本台 B2 已做的光学标定。
IMU/关节无滑移里程计仍为实验性，对共同打滑没有独立观测；瓷砖反光、图像网络延迟、标定误差和打滑会影响精度。
相机时间按 SDK 主机收图时间关联，不宣称硬件曝光同步。TCP 实际跟踪误差尚未在实机验收。
无已观察到的后方点或目标超出训练空间时，冻结目标并明确显示原因，不生成虚假深度/路径。

## 操作（仅本版完整验证并编译后）

PC4 NoMachine 桌面终端，机器人实际趴稳、旧控制退出、无焊缝枪和软管，收臂路径与运动空间清空：

    cd /home/unitree/rl_sar_b2_z1 || exit 1
    bash diagnostics/va15_visual_tcp_v241_20261023/run_visual.sh eth0 --walk

输入大写 RUN → A 起立 → 实际站稳 → Y 待机 → 确认黄/青色曲线在胶带上 → R1+Y 开始。
同键再次停止；X/Space 停止；B/9 请求停稳后趴下。无四秒静态审查，无窗口 R，无一次性机会限制。
起步前仍检查当前反馈和已完成的准备，不缓存危险的提前启动请求。P/L1+X 紧急被动可能失去支撑。
初次先用 X 验证停止，再验证 B；不要把断电/重启当成站立时的安全停止。实际趴稳且 Passive 后再 Ctrl+C。

日志中的 `TCP_flat_floor=1 active=1` 表示正使用缓存路径，`cache_frames` 是缓存数量，`target_b` 是输入策略的机身系目标，`clearance` 最终平滑趋近 0.09 m。
`waiting for front-observed path to reach rear TCP` 是等待真实历史路径，不代表故障。`active=0` 不应被当成已完成末端跟踪。

只看相机，不接管：

    bash diagnostics/va15_visual_tcp_v241_20261023/run_visual.sh eth0 --preview

离线自检，不初始化 DDS/UDP：

    bash diagnostics/va15_visual_tcp_v241_20261023/run_visual.sh --offline-check

## 开发电脑验证

    cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build build -j2
    ctest --test-dir build --output-on-failure
    /home/dong/anaconda3/envs/crack_vision/bin/python -m unittest test_tracker test_long_tracker test_motion_tracker test_session test_v24_camera
    /home/dong/anaconda3/envs/crack_vision/bin/python replay_rejections.py ../va15_visual_walk_v23_20261023/outputs/review_8rv_wx74/exchange
    python3 run_suite.py --output-name 新回归目录
    python3 run_sessions.py 新会话目录
    python3 run_tcp_suite.py 新TCP目录

`simulate_tcp curve 新文件.csv 0.8` 是真值位姿对照；`estimated_curve` 等正式用例只把真值用于生成合成像素和独立评分，控制与坐标转换使用实际独立 IMU/关节估计器。
正式用例覆盖直线、平缓曲线、摩擦 0.5/0.8/1.0、单帧漏检、持续失线和位姿失效，包含 X 停止、B 趴下。
报告的 `steady_lateral_rms_m` 仅指首次 TCP 激活八秒后的路径横向误差，不是三维 TCP 精度；`tcp_xyz_command_rms_m` 单独记录完整位置命令误差。
这些测试不证明实机分割/里程计精度，不是焊接验收。
qualification.json 在验证通过前拒绝 --walk；native_build_visual/bundle.json 绑定板载实际源码、权重和二进制。
