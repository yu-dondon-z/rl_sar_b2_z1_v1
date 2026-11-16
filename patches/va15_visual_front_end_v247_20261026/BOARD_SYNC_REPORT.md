# V2.4.7 验证与板载同步（2026-10-26）

状态：本地离线验证通过，已同步 PC4 并完成独立编译和离线检查；未启动相机、Z1 服务、控制或行走，没有释放厂商控制。

目标为前视确认裂缝结束后支撑站稳，不要求后方 TCP 到达物理终点。不启用 V2.4.6 不可见尾段行走。

## 测试结果

- 本地 C++ 16/16：端点确认、包一致性、重复启停、原有停止、首帧力矩连续、日志列对齐、无传感器的原生 actor golden 等。
- Python 本地和板载均 54/54：真实 RGB 延续而网络漏检不当作结束；短线、ROI 截断、明显遮挡、长断口后延续和旁线不当作正常终点。
- 新合成 RGB + 独立 IMU/关节估计 + 实际 VA15/FSM 物理仿真 11/11：直线/曲线各在摩擦 0.5、0.8、1.0 下确认线尾后站稳；无线尾持续到人工 X；普通失线/相机断流保护停车而不宣称正常结束；单帧假线尾不停；人工 B 能回到趴卧 Passive。
- 原有 TCP 物理回归 8/8：独立估计、曲线/直线、自适应行、单帧丢失、长失线、位姿丢失。
- PC4 新端点确认和包校验两个 C++ 独立测试通过。原生离线 golden 最大误差 1.14441e-05；没有初始化机器人传输。

报告：outputs/ctest_v247.log、outputs/python_unittest_v247.log、outputs/front_end_v247_run2/summary.json、outputs/tcp_regression_v247/summary.json。各物理套件记录生产源文件和执行文件 SHA-256，测试过程中源文件未改变。

终点仿真中物理线尾 x=5 m，确认停车后机身 x 约 2.60～2.67 m。这是**前视看见终点即停**的预期范围，不是后方 TCP 已到线尾；不得把这些数字当成实机定位或端点精度。合成像素测试不代表真实网络终点识别率。

## 板载位置与指纹

独立目录：/home/unitree/rl_sar_b2_z1/diagnostics/va15_visual_front_end_v247_20261026。

- native_build_visual/bundle.json SHA-256：a15d84d8f3f85ebcd34a87a55a644945662d5e35397a011fc360a7a51a30c549
- 独立 va15_forward_trial SHA-256：9ad08a93ca490cd0ae5ded8a513792731f126e5d017c094c311538a87a7ea105
- 既有 cmake_build_va15 主程序未替换：696756deac43676046b0073d6c7ee0c0eb1a79f0248f678f9df337013f8d9ba3
- 既有 cmake_build 主程序未替换：f0c6ff7dee15490de6a4a92f24f664042e880d6f446fbfd5ff7cb9fb111ffec6
- VA15 权重未更换：8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098
- base.yaml 未修改：909d04f2ce4820effd7a284ccff5955dc6a9c8fba921f0fe42a439627ced7164
- VA15/config.yaml 未修改：8faa51628b3704ce3e7e3a384f1c0eb8bb59b7dcf5a0069992069efed8aab8d1

V2.4.5 的 FSM/停止控制器字节一致；新增内容仅相机终止证据、前视终点停止请求与相应日志和验证。没有改动策略增益、足端阈值或真实故障检查。V2.4.6 独立候选及其未放行记录保留。

## 人工测试入口

```bash
cd /home/unitree/rl_sar_b2_z1 || exit 1
bash diagnostics/va15_visual_front_end_v247_20261026/run_visual.sh eth0 --walk
```

大写 RUN → A → 实际站稳后 Y → 等待支撑和机械臂就绪 → 确认 PATH IN RANGE 对应正确胶带 → 一次 R1+Y。连续运动直到确认线尾、再次 R1+Y/X/B 或保护停车；无固定短距离/时长上限，无自动重启/趴下。X 支撑停止；B 停止并趴下；实际趴稳且 Passive 后 Ctrl+C。

仅放行有人看护、无枪/软管、平地、防跌和遥控急停准备好的人工实验入口，不是实机验收。hardware_qualified、precision_qualified、visual_tracking_hardware_acceptance 仍 false。相机暂时丢失或遮挡只算保护停车，不算正常完成。
