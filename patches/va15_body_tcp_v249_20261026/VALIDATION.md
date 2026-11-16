# V249 本地验证（2026-10-26）

## 问题依据

V248 实测录像与日志对应记录在 outputs/diagnosis_v248_gaj1dt1p_20261026/DIAGNOSIS.md：在部分相机近/远点均位于右侧的帧中，减去无滑动里程计的侧滑角会把视觉修正翻成相反方向。周期日志中后向 TCP 始终 inactive。以上不是从视频单独推测执行器方向。

## 最终通过

- outputs/ctest_final.log：19/19 C++ 测试；含抗串台、停止/交接、包解析、历史缓存、工作范围、纯几何纠偏、SDK-free 原生 golden。虚拟焊缝 TCP FK 与 MuJoCo ee_site 对比 1000 姿态，最大几何差 1.37e-15 m；不是实物工具标定精度。
- outputs/python_unittest_final.log：62/62 Python 视觉回归。near_tracker.py、long_tracker.py、endpoint_vision.py 与 V248 字节相同。
- 重新编译 validate_saved_packets.cpp 校验 V248 的真实图片包：25 包/23 有效 + 41 包/39 有效，全部 66 个有效性判断相同。仅保存图片与包检查，不是完整相机实时链路验收。
- outputs/tcp_v249_integral/summary.json：8/8 长时间实际 actor + 私有 FSM + IMU/关节独立位姿估计的 MuJoCo 物理测试。正常前进约 100 秒/15.4～16.2 m；另外测试短暂丢帧、持续丢线、位姿失效后的停止及 B 趴下。
- outputs/end_v249_integral/summary.json：11/11 终点/停车测试，包括直线/轻微曲线及不同摩擦、无线尾、视觉/相机丢失、单帧伪线尾、B 停止并趴下。看见线尾后站立停车，不自动趴下。

正常长走案例中：初始机身横向最大偏差 11.0～12.1 cm；70～106 s 的后期 RMS 为 0.35～0.74 cm。TCP 接入 33.46～39.69 s，随后实际稳定跟踪约 65～72 s；稳定阶段 TCP 横向 RMS 为 0.58～0.86 cm。首次接入需等待前视历史走到后方且处于原工作范围，不会伪造初始后方轨迹。

**不能据此宣称三维精确焊接。** 稳定阶段实际 TCP 对已应用目标的三维误差 RMS 仍约 6.8 cm，主要为前后方向。初次接入和降低高度阶段误差更大。场景为无枪/软管，Z1 使用已有 MuJoCo 隐式位置伺服代理；相机坐标为按标定合成，控制只用本体估计，真值仅渲染/评分。不是实机 Z1 内环、真实分割、深度、全局里程计、强曲率或负载的验收。

终点测试中，5 m 的物理线尾在机身约 x=2.7～2.8 m 时已被前视确认，随即站立停车。此行为仍沿用前一版约定，不代表后方枪尖走完最后一段。

## 保留失败证据

outputs/tcp_v249、straight_body_refined、direct_probes 和 lateral_probes 为开发期间失败/诊断结果，不是释放证据。早期仅改比例项时机身斜走造成后向目标超范围，TCP 不接入；不通过扩大工作范围、强制 active 或放宽停止条件修饰结果。最终通过版本用视觉积分去除持续偏差，并仅在原工作范围内沿已观测段分配目标。后续正式结果不能继承旧版本字节相同控制内核的结论。

板载编译和同步状态另见 BOARD_SYNC_REPORT.md；hardware_qualified、precision_qualified 始终为 false。
