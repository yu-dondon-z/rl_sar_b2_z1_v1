# VA15 视觉短程试验 V2.2 — lost 仅诊断（2026-10-23）

独立候选版本，保留 V2、V2.1，不替换 VA15 model_5000 权重、配置或旧可执行文件。
本轮只测试身体沿图像曲线短程行走；不是完整的米制轨迹/TCP 焊接跟踪验收。

## 本轮更改

- 按用户要求，在本候选中显式启用 LostPolicy::DiagnosticOnly。原始 lost 全量保留在 commands.csv；它的增加、减少、回零和持续变化不再单独阻止 A/Y、撤销视觉批准或中断行走。
- 不把 lost=5 解释成“五次掉包”或“故障码5”。当前固件上该字段的精确定义尚未核实。
- 保留最初健康帧的 lost 参考向量，并新增 lost_diagnostic_only、lost_change_events。后者是本程序观察到数值变化的帧次数，不是设备丢包数；没有清零、改写传感器原始数据。
- 启动仍需连续三秒当前健康反馈。电机 mode/reserve 故障、bit_flag 超时/故障、源 tick/新鲜度、接收时间、IMU、Z1 看门狗、支撑/停止检查均保留。
- Strict 默认模式仍可用；旧版本未修改。没有 lost 自动重置/重新接管的暗门，也不需要额外的 lost 恢复按键。
- RHACrackNet/相机标定、V2.1 长像素曲线识别、近点 ±0.40 rad/前瞻 ±0.45 rad、固定 TCP、70/18 观测动作、EMA、关节映射和增益均不变。
- 正向请求 0.15 m/s，侧向零，转速限制 ±0.06 rad/s，转速变化率 0.04 rad/s²；每进程最多一次四秒正向请求。没有开放长时间无人值守循迹。

## 板载终端启动

使用 PC4 NoMachine 桌面终端；不要同时运行其他控制程序。开始前 B2 应实际趴稳、旧控制退出、无焊缝枪和软管，收臂路径及行走区域清空；准备已验证的遥控急停和可靠防跌倒措施，人员离开运动范围。

    cd /home/unitree/rl_sar_b2_z1 || exit 1
    bash diagnostics/va15_visual_walk_v22_20261023/run_visual.sh eth0 --walk

1. 相机打开后，在控制终端输入大写 RUN。趴卧视角无有效轨迹，不阻止静态起立。
2. 等 READY，按 A 起立；实际站稳后按 Y 进入待机。终端会显示 lost_DIAGNOSTIC_ONLY=1。保持摇杆归中。
3. 站稳后如锁到了旧线，聚焦相机窗口按 R 重新选线；确认黄色近点、青色远点在目标胶带而非瓷砖缝，等 PATH IN RANGE。行走中不要按 R 换线。
4. 第一次 R1+Y：只做四秒 ZERO OUTPUT REVIEW，不应行走。等待终端 SHADOW PASSED。松开按键。
5. 30 秒内第二次 R1+Y：才开始一次最多四秒的前进和小幅纠偏。单独 lost 变化不会再撤销该批准；其他健康、视觉、静止支撑条件仍需满足。
6. X/Space 请求停稳；B/9 请求停稳后趴下。确认实际趴稳、终端 Passive 后再 Ctrl+C。
7. 四秒请求结束不会自动再走；需要新进程做下一次实验。发生其他故障时不要连续按键强行接管。

键盘对应：0 起立、1 待机、T 对应 R1+Y、9 趴下。P/L1+X 是紧急被动模式，可能失去支撑，不等同正常站立停车。站立时不要强退/断电停车。
四秒是请求时长上限，不保证实际停止时间和距离。Z1 独立服务不随会话强杀；会话只清理自己启动的相机。

只读画面或静态零输出：

    bash diagnostics/va15_visual_walk_v22_20261023/run_visual.sh eth0 --preview
    bash diagnostics/va15_visual_walk_v22_20261023/run_visual.sh eth0 --shadow
    bash diagnostics/va15_visual_walk_v22_20261023/run_visual.sh --offline-check

## 本轮离线验证

- C++ 5 项通过。新增 lost 增减/回零/溢出边界/持续变化，以及同时出现电机故障、超时、时序异常等测试；这些真实故障仍被拒绝。
- 独立支撑估计/策略/停止物理仿真 39/39；包含 lost 单独变化仍能运动、变化同时有真实故障仍受限、X/B、低摩擦、估计扰动。
- 功能仿真 27/27；包含完整视觉静态审查/第二次请求、行走前 lost 变化、行走中变化及持续变化、失线/错误图/写入时序/超角度拒绝。
- Python 相机/曲线/会话 18 项通过。板载原生可执行文件另跑 VA15 黄金输出和 lost 仅诊断自检，不初始化 DDS/UDP。
- 仿真通过不等于实机验收；合成相机不代表真实识别精度。当前反馈估计仍为实验性，未伪装成硬件标定通过。
- 策略后退/左右平移和 TCP 精度仍有已知限制，本候选只开放上述短程正向试验。

开发电脑结果：outputs/regression_v22_diagnostic/summary.json、outputs/functionality_v22_diagnostic/summary.json。
qualification.json 保存测试结果和生产源指纹；板载 native_build_visual/bundle.json 绑定实际源码、配置、模型和二进制。
每次运行 Session records 目录保存命令、控制、策略 CSV、相机原图和终端日志。源 DDS Write 返回不代表电机确认。

## 开发电脑离线重跑

    cd /home/dong/rl_sar_b2_z1/patches/va15_visual_walk_v22_20261023
    cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build build -j2
    ctest --test-dir build --output-on-failure
    /home/dong/anaconda3/envs/crack_vision/bin/python -m unittest test_tracker test_long_tracker test_session
    python3 run_suite.py --output-name 新回归目录
    python3 run_functions.py 新功能目录
