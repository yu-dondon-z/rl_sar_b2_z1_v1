# VA15 长像素曲线视觉短程试验 V2.1 — 2026-10-23

独立版本，不替换无视觉 V2、视觉 V2 程序、VA15 model_5000 权重或配置。
这是实验性身体循迹试验，不是已验收的焊接 TCP/高精度/全向实机部署。

## 修正内容

- RHACrackNet 权重不变。RGB 胶带区域 + 广泛网络支持锁住近端，再按连续性、透视宽度范围、网络支持向远处追踪。
- 保留曲线形状，每约 8 图像像素一点，最多 128 点；不外推、不补画断线、不创建虚假深度或世界轨迹。
- 近点固定 v=972，前瞻点固定 v=820.8；连续轨迹覆盖两处，跨度至少 200 像素、至少 25 点。近端置信度仍至少 0.95，前瞻支持至少 0.90。
- 修复本次已查明的拒绝条件：近点从 ±0.18 扩大至 ±0.40 rad（约23°），前瞻点从 ±0.30 扩大至 ±0.45 rad（约26°）。当前记录约18°/21°的胶带不再因角度被拒绝。没有关闭所有视觉准入条件，也没有放宽足端/支撑/电机/通信或停止检查。
- 画面明确区分 PATH IN RANGE 与 PATH OUTSIDE ANGLE RANGE；终端 Visual gate 列出实际角度、限值、曲线就绪、静止支撑、遥控归中和健康状态，拒绝时给出具体类别。
- 转速目标为 clip(1.2*(0.6*near_bearing+0.4*lookahead_bearing), ±0.06 rad/s)，低通时间常数 0.4 秒，变化率最多 0.04 rad/s²。
- 前向请求仍为 0.15 m/s、侧向为零、每进程一次最多 4 秒正向请求。策略加减速、EMA、70/18布局、关节映射和增益不改。
- 使用 B2 前光学 RGB，不是深度；本轮 TCP 目标不变。原 18 维全身策略仍控制四足和机械臂。
- 原足端、电机、支撑估计和停止门槛保留；不伪造接触，不把实验性估计当成硬件认证。

## PC4 一次启动，两个明确按键

在 PC4 的 NoMachine 桌面终端执行，不同时运行旧控制，不另开 Z1 窗口：

    cd /home/unitree/rl_sar_b2_z1 || exit 1
    bash diagnostics/va15_visual_walk_v21_20261023/run_visual.sh eth0 --walk

开始条件：B2 已实际趴稳、旧控制退出，无枪和软管，手柄归中，周围和收臂路径无遮挡，具有可靠防跌倒保护及已验证的遥控急停方案，人员离开运动/跌倒范围。

1. 相机打开后输入大写 RUN；趴卧视角无效不妨碍静态起立。
2. 等电机契约 READY，按 A 起立，站稳后按 Y 待机，等待暖机/静态支撑就绪。
3. 聚焦相机窗口按 R 重新选线。确认黄色近点、青色远点沿目标胶带而不是瓷砖缝，等待 PATH IN RANGE。这只表示视觉条件合格，尚需当前控制状态检查。R 会立即发布无效帧，行走中按 R 会停止，不能无缝换线。
4. 第一次 R1+Y（或聚焦控制终端按 T）：四秒 ZERO OUTPUT REVIEW，不应行走；检查当前相机、路径关联、支撑、电机和控制/写入时序。
5. 等终端明确显示 SHADOW PASSED: still ZERO OUTPUT; press T AGAIN。松开按键，30 秒内第二次 R1+Y，才执行一次最多四秒前进/小幅纠偏。没有自动转入行走。
6. 摇杆保持归中。异常按 X/Space 请求支持停止；B/9 请求停稳后趴下。失线、换线、过期、遥控偏移或故障均撤销请求，不自动续走。
7. 四秒结束后按 B，确认实际趴稳且终端 Passive，再 Ctrl+C。相机随会话退出；不结束独立 Z1 服务。

审查失败不允许走，可显式重试静态审查，不消耗实际一次行走请求。批准后若路径过期/改变、状态异常或不再静止，批准失效，必须重新明确审查。
四秒约束的是正向请求，不保证实际停稳时间/距离。站立时不要强退、重启、断电停车。P/L1+X 是可能失去支撑的紧急被动模式，按已验证的应急流程使用。

## 只预览 / 零输出

    bash diagnostics/va15_visual_walk_v21_20261023/run_visual.sh eth0 --preview
    bash diagnostics/va15_visual_walk_v21_20261023/run_visual.sh eth0 --shadow

preview 不启动运动控制，Q 关闭窗口。shadow 仍须 RUN/A/Y，但再按 T 也不开放行走。每会话独立随机 nonce 和交换目录，不读取上次残留路径。

## 验证与边界

- 停止/故障/摩擦/估计扰动回归 34 项；功能仿真 24 项，新增接近实际18°/21°偏角的左右捕获、低摩擦捕获和超大偏角拒绝；仍覆盖完整双按键、失线、无效图、X、写入时序故障。
- C++ 4 项和 Python 18 项；原生 VA15 黄金输出检查另见板载 native_build_visual/offline_check.log。增加实际记录偏角的完整零输出检查/第二次明确请求测试。
- 本次站立原图回放提取 47 点、348 像素跨度；原始运行记录中有61点、390像素跨度。算法未更换识别网络或降低置信度。重复同一原图不等于六帧独立锁定或实时验收。
- 当前仅做四秒渐进纠偏，不保证一次就回到胶带中心，不据此开放长时间无人值守跟随。
- 画面是否跟对胶带仍需操作者确认；同图重复、合成相机和 DDS Write 返回均不代表实机效果或电机 ACK。
- 合成直线四秒仍有厘米级横向偏差，TCP RMS 约 0.10 m，不能作为焊接精度验收。固定当前 TCP 时后退/平移有效位移不足，本轮不开放。
- 相机时间戳为主机收到 JPEG 后的单调时钟，不是硬件曝光同步；透视宽度启发仅约束图像区域，不测量地面深度。

结果索引 qualification.json；开发电脑详见 outputs/regression_v21、outputs/functionality_v21_final、outputs/current_tape_replay。
运行时 Session records: 指向 control/commands/actor CSV、终端日志、vision.log、未标记原图与审查状态。

## 开发电脑离线重跑

    cd /home/dong/rl_sar_b2_z1/patches/va15_visual_walk_v21_20261023
    cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build build -j2
    ctest --test-dir build --output-on-failure
    /home/dong/anaconda3/envs/crack_vision/bin/python -m unittest test_tracker test_long_tracker test_session
    python3 run_suite.py --output-name 新回归目录
    python3 run_functions.py 新功能目录
