# VA15 V2.4.6：实际后方 TCP 的有限终点跟踪候选

独立候选，2026-10-25建立，2026-10-26继续验证。继承 V2.4.5，不覆盖旧版本或主程序。
**当前开发/离线验证，不是实机行走放行。** 以本版 `qualification.json`
和 `ENDPOINT_REPORT.md` 为准。复制的其他旧报告、旧计数与旧指纹均只作历史参考。

## 用户要求

机身可以先通过终点；后方焊缝 TCP 走到裂缝真实终点并站稳后才完成。
不能把“前摄像头看不到线”“B2 机身越过末端”或“目标点已到终点”当成完成。
自动完成只站稳，不自动趴下。B 仍是操作者明确要求的支撑停车、趴下流程。

## 实现逻辑

1. 保留原 RGB 网络、标定、近处关联、自适应前视、长度与置信度检查。
   新增同一条已观测路径的末端证据：RGB 确实结束、前方存在清晰地板，
   而不是 ROI 截断、反光间隙、仅神经网络掉点或可见遮挡。
2. 控制端用新鲜图像与对应的局部姿态投影，多帧确认同一米制终点。
   至少六个不同图像、至少0.5秒，位置须一致；一帧红点不代表确认。
3. 只缓存真正看见、相互有连续重叠的有限路径。不连接断裂片段，不外推。
   必须从实际后方 TCP 到终点都有已观测覆盖，才进入尾段。
4. 进入尾段后，前摄像头可以自然越过已确认的终点。仍须持续收到新图像，
   且局部姿态、遥控、B2/Z1 状态有效；不允许相机停更或定位失效后盲走。
   尾段具有时间、长度、行程预算，并限制速度和转向变化。
5. 用实测六关节 FK 得到实际虚拟焊缝 TCP，按尾段距离提前减速。
   减速限幅贯穿 FSM 和 actor 输入，不被原固定0.15m/s测试命令覆盖。
6. 接入原支撑停车控制器。停车后再次检查实际 TCP，而不是目标坐标；
   仅原停车控制器确认站稳、实际位置/高度/速度合格才显示 `COMPLETED`。
   安全站稳但精度不合格显示 `SAFE STAND / ENDPOINT NOT COMPLETE`。
   原支撑站稳后还需连续1秒通过位置、9cm目标高度和低速复核。

状态：`Observing → Confirmed → Tail → AwaitStand → Completed`。
任何未知失线、矛盾终点、越界、姿态重置或操作者停止都不是任务完成。
不会自动重试行走，也不会因为恢复图像而自行启动。

## 模型与边界

- VA15 model_5000、70观测/18动作、raw/EMA、关节路由、增益和足端阈值不变。
- 无枪时测的是模型中的“虚拟焊缝 TCP”，不是物理法兰；真实装枪必须另行工具标定。
- TCP 目标保留训练约9cm离地高度，不改成接地或1cm。不是 IK 外挂。
- 仍为单目平地投影和 IMU/关节无滑移局部估计，不是深度或绝对定位。
- 若初始后方路径从未被前摄像头看过，过近的终点可能无法获得完整覆盖；
  此时停止、报告覆盖不足，不能靠伪造初始后方路径完成。
- RGB 无法区分与地板完全同色且遮住全部后续线段的遮挡物与真正末端；
  合成测试并非实际胶带/裂缝的识别准确率。
- `COMPLETED` 是有容差的实验状态，不承诺实枪精确接地、接触力或软管负载能力。
- **已知未解决项**：单目末端缓存依赖实验局部位置估计。切换成 IMU/关节估计后，
  已发现估计内完成但物理真值仍有约18～21cm偏差的场景；因此本版尚不放行实机行走。
  关节 FK 精确不等于机身位置精确，也不能把真值位置测试当作部署通过。
- 实机尚无独立终点位置验证，`terminal_metric_verified=false`，局部估计内停车
  只显示 `LOCAL ESTIMATED END / PHYSICAL TERMINAL UNVERIFIED`，不会冒充任务完成。

## 离线验证

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j2
ctest --test-dir build --output-on-failure
/home/dong/anaconda3/envs/crack_vision/bin/python -m unittest discover -p 'test_*.py'
/home/dong/anaconda3/envs/crack_vision/bin/python check_tcp_fk.py --fk-binary build/test_tcp_fk --samples 2000
python3 run_endpoint_suite.py NEW_OUTPUT_NAME full
python3 run_endpoint_suite.py ANOTHER_NEW_OUTPUT_NAME estimated
```

以上不初始化任何机器人接口。物理仿真使用实际 VA15/FSM，但合成像素和
仿真真值姿态只存在于测试器，不构成实机验收。详细结果见本版报告。

## 实机

本版在完成验证前 `walk_release_allowed=false`，不提供可绕过的实机启动步骤。
经人工批准后才单独同步、板载离线编译；不会自动站立、接管或运行策略。
保留熟悉键位 A起立、Y待机、R1+Y/T启停、X停车、B支撑趴下。
P/L1+X 会撤去主动支撑，不是日常停车。实际趴稳且Passive后才能Ctrl+C。
