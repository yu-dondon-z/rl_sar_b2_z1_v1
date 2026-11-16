# VA15 V2.4.2：力矩连续接管与无视觉固定 TCP 对照

独立候选，保留相邻 V2.4.1 和主程序。本轮已重新连接 PC4；用户确认实际趴稳、控制退出、无枪和软管。仅同步独立源码并做板载构建/离线检查，不自动运行实机。最终板载状态见 BOARD_SYNC_REPORT.md。
不是实机运动、精度或焊接作业验收。最终状态见 qualification.json 与 HANDOFF_REPORT.md；不继承旧版本的放行声明。

## 修复内容

- 首个新 actor 结果返回前，完整保留最后的站立指令，不能提前从 675 切到 160 而丢失支撑。
- 首个 actor 结果到达时，立即使用训练使用的腿部 Kp=160/Kd=5，通过等效目标保持该时刻的请求 PD 力矩；补偿偏置在 1.5 秒内平滑释放，保持策略后续的动作变化。
- 计算使用已请求的旧 PD 指令与同帧实测 q/dq，不把 tau_est 当作前馈，也不宣称 DDS 写成功等于电机确认。位置目标不一定连续，但瞬时请求力矩连续。
- 不在策略迈步期间保留 450/675 的站立高增益；闭环测试已排除这个变体。释放完毕后完全回到原策略增益与输出。
- 目标/力矩超出原有范围时不发送部分更新，保留最后有界命令并请求既有支持停止；不强制接触，不自动重启。
- 保留 V2.4.1 的同帧时钟、支持停止、B 趴下与有限退出修复。保持权重、70/18 观测/动作、关节映射、alpha=0.2、raw 上一动作以及足端阈值/极性不变。
- 新增真实“无相机、固定 TCP”入口，不构造假视觉包，不替换定位/足端证据。仍由同一 VA15 全身策略输出 18 维动作。
- CSV 新增 torque_handoff_active/pending/failed/elapsed_s、blind_fixed_target，便于下轮实机确认。

## 本地验证

    cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build build -j2
    ctest --test-dir build --output-on-failure
    /home/dong/anaconda3/envs/crack_vision/bin/python -m unittest test_tracker test_long_tracker test_motion_tracker test_session test_v24_camera
    python3 run_handoff_suite.py 一个新的结果目录

run_handoff_suite.py 是不含机器人 SDK 客户端的闭环 MuJoCo 对照：固定 TCP、无视觉，比较旧版与新版；摩擦 0.5/0.8/1.0、速度 0.15/0.20 m/s，以及起步中途 X/B。真实 Z1 内环与电机动态仍未辨识；仿真不能证明持续实机微振消除。

final_baseline_v242、final_sessions_v242、final_tcp_v242 是最终干净编译后的旧功能回归；最终报告绑定实际源文件和可执行文件 SHA。早期 outputs/handoff_first 与非 final 的输出只供开发对照，不作为当前版本放行依据。

## 重新连接后才能准备板载版本

确认实际趴稳、控制退出、没有焊缝枪/软管，再把本目录复制为 PC4 独立目录 diagnostics/va15_visual_tcp_v242_20261023，排除 build/outputs。只能通过 build_board_candidate.py 构建及 --offline-check 检查，不自动运行控制。
build_board_candidate.py 使用本机既有板载依赖，并生成绑定该板实际源代码、模型、配置和二进制的 native_build_visual/bundle.json；不要复制开发电脑的可执行文件代替板载构建。

只有 BOARD_SYNC_REPORT.md 记录板载构建、自检和 bundle 校验成功，且满足人工测试条件后，先用无视觉版本：

    cd /home/unitree/rl_sar_b2_z1 || exit 1
    bash diagnostics/va15_visual_tcp_v242_20261023/run_blind.sh eth0

该入口不开相机，原 TCP 观测保持 [-0.775,0,-0.35,0,-0.707107,0,0.707107]。按 R1+Y/T 后固定 x=0.15 m/s、y=yaw=0。仍保留反馈健康、准备完成、摇杆回中、支持停止和真实故障保护。

在清空空间、可靠防跌措施与遥控急停准备好后，输入大写 RUN → A 起立 → 站稳 → Y 待机 → 等待 warmup/支撑准备 → R1+Y 短距离测试 → X 停止 → B 趴下。重复启动必须再次明确按 R1+Y，视觉恢复/摇杆保持均不会自动启动。
不直接长走。再次抖动应结束测试，不加速、不增益、不切视觉硬试。P/L1+X 会移除主动支撑，按既有急停流程操作；实际趴稳且 Passive 后再 Ctrl+C。

视觉流水线的延迟与重新获取问题尚未在本次修改；本候选的视觉行走保持未放行，先完成无视觉实机对照。旧版本不受影响。
