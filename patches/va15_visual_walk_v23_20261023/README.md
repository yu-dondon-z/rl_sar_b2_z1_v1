# VA15 视觉启停 V2.3 — 单键、连续、可重复（2026-10-23）

独立版本，保留 V2.2 和更早版本，不替换 VA15 model_5000 权重、增益、标定或基础配置。
当前只用前光学 RGB 引导本体沿像素曲线行走，18 维全身策略仍控制 B2/Z1；
TCP 目标仍固定，不是深度/米制路径或焊接 TCP 跟踪验收。

## 操作更改

- A 起立，Y 进入策略待机。R1+Y（键盘 T）一次直接开始行走，再按一次停止。
- 取消额外四秒零输出审查、二次确认启动、四秒行走上限及“每进程只能走一次”。
- 行走持续到 R1+Y、X、B、摇杆偏移或实时视觉/健康条件失效；没有无人值守的持续运行保证。
- 正常停稳后，可以再次 R1+Y 启动，不需要退出进程或重新按 Y。若仍在减速/站姿恢复中，请等 restart_ready=1 后重新按键；不缓存启动请求。
- 相机自动处理趴卧到站立/待机的视角变化，不需聚焦窗口按 R。
- 失线或故障后不自动恢复运动。图像恢复且停止完成后，仍需明确 R1+Y。
- X/Space 请求支持停止；B/9 请求停稳后趴下。B 优先，不会被同时的启动请求覆盖。
- P/L1+X 为紧急被动模式，可能失去支撑；按已验证应急流程使用。实际趴稳并显示 Passive 后再 Ctrl+C。

启动命令（PC4 NoMachine 桌面终端）：

    cd /home/unitree/rl_sar_b2_z1 || exit 1
    bash diagnostics/va15_visual_walk_v23_20261023/run_visual.sh eth0 --walk

启动前 B2 实际趴稳、旧控制退出、无焊缝枪和软管，收臂及运动区域清空。
准备可靠防跌倒措施和已验证的遥控急停，人员离开运动范围；首次只走几步便按 X 验证正常停止，再按 B 验证趴下。
输入大写 RUN 后，等 READY → A → 实际站稳 → Y → 相机 PATH IN RANGE → R1+Y。
无需四秒静态审查，但实时反馈、已完成的起立/策略准备、静止支撑和遥控归中检查仍保留。

## 修正起步时的视觉误判

旧版把相邻图像胶带拟合位置变化超过 12 原生像素直接判断为换线。
本版先用周围地面纹理估计全局画面运动，把上一帧胶带预测到当前图像，再使用原来的 12 像素身份残差检查。

- 640×360 上做背景特征 LK 双向光流和 RANSAC 单应性；排除暗胶带及其周边，不用胶带自身来证明“没有换线”。
- 要求至少 20 个内点、足够的二维空间覆盖、双向一致性和小重投影残差；超过 75 原生像素的全局图像移动或过期参考不补偿。
- 背景证据不足时退回严格关联；不会仅因为“应该在走”就接受任意线。
- 保留胶带 RGB 宽度、对比度、广泛网络支持、近/远点和连续曲线检查。
- 原生控制器只对经验证的运动补偿包使用有界的图像角度连续性范围；错误序列、旧时间戳、错误会话和重复包改写仍拒绝。
- 站立自动重获时清除旧趴卧图像参考。准备/停止中和正在行走时不自动切换到另一条线；在安静待机才允许失败后的自动重获。
- 控制状态只由非实时文件线程发布，主控制线程仅更新原子 phase。相机验证 session_id 和一秒新鲜度，过期时不发有效路径。
- 视觉拒绝事件最多保留 20 组原始前/后图和 JSON，方便确认是机身晃动还是实际换线。

## 未修改的边界

- lost 延续 V2.2：仅记录诊断，不单独阻止运动；不解释成已确认的累计掉包数或故障码。
- 真实电机故障、命令超时、源反馈时序、Z1 看门狗、IMU、支撑和停止检查仍保留。
- 连续模式增加 50 ms 控制心跳约束；控制周期停止更新时，策略不得继续发前进请求。
- 前向请求 0.15 m/s、侧向零、yaw ±0.06 rad/s、yaw 变化率 0.04 rad/s²不变。
- 固定 TCP、70/18 观测动作、raw action history、EMA、关节映射和增益不变。
- 支撑估计仍为实验性，不伪造接触，也不将仿真通过当成实机认证。

## 验证和结果

C++ 组件、Python 图像/上下文、旧版回归和新连续会话物理仿真均需通过后才发布 --walk。
新会话仿真包含单次按键直接启动、持续超过四秒、同进程第二次启动、R1+Y/X/B/摇杆停止、失线后不自动续走、故障拒绝，以及低摩擦测试。
合成图像不等于实机识别验收；十秒级回归也不等于长时间持续行走验收。

结果索引 qualification.json。开发电脑 outputs/regression_v23_final、outputs/functionality_v23_final、
outputs/continuous_v23_final 保存源指纹和测试结果。早期 continuous_v23 保留了过早请求重启被正确拒绝的测试记录。

板载 native_build_visual/bundle.json 绑定实际代码、配置、模型及二进制；
运行 Session records 目录保存原始 CSV、终端、相机日志和拒绝原图。
visual_session_count 用于区分多次启停；命令写入成功不代表电机确认。

仅画面预览，不接管机器人：

    bash diagnostics/va15_visual_walk_v23_20261023/run_visual.sh eth0 --preview

离线自检，不初始化 DDS/UDP：

    bash diagnostics/va15_visual_walk_v23_20261023/run_visual.sh --offline-check

开发电脑离线重跑：

    cd /home/dong/rl_sar_b2_z1/patches/va15_visual_walk_v23_20261023
    cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build build -j2
    ctest --test-dir build --output-on-failure
    /home/dong/anaconda3/envs/crack_vision/bin/python -m unittest test_tracker test_long_tracker test_motion_tracker test_session
    python3 run_suite.py --output-name 新回归目录
    python3 run_functions.py 新功能目录
    python3 run_sessions.py 新连续会话目录
