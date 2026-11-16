# V254 最终离线验证

2026-10-27。本轮无 SSH、无板载更新、无实际机器人控制。

## 证据

| 检查 | 最终结果 | 文件 |
| --- | --- | --- |
| CTest | 31/31 | outputs/ctest_final.log |
| Python | 67/67 | outputs/python_final.log |
| 归档训练目标分配契约 | PASS | outputs/training_contract.json |
| 后方 TCP 物理闭环 | 19/19 | outputs/rear_endpoint_final_v2/summary.json |
| 物理误差分析 | 12 正常组均到达 | outputs/rear_endpoint_final_v2/diagnosis.json |
| V253/V254 轨迹对照 | 已生成 | outputs/rear_endpoint_final_v2/comparison.png |

12 个正常组：直线/缓曲线，摩擦系数 0.5/0.8/1.0，以及直线 μ=0.7；另有 2/3/8 m 直线和 3 px 像素噪声直线/曲线。**不是长度、摩擦、噪声的完整笛卡尔积。**

7 个异常/人工停止组：未见线尾、未知丢线、相机中断、单帧假线尾、途中按 B、线尾接近时按 B、线尾接近时位姿失效。两个 B 用例实际进入仿真趴卧/Passive。

## 独立评分

- 正常组最终虚拟 TCP 水平误差：最小 0.00406329 m，中位数 0.05183545 m，最大 0.0623229 m。
- 最大每控制周期目标位置步长约 0.0002 m。
- 独立物理验收容差未变：请求制动时水平误差不超过 0.10 m，停车后不超过 0.12 m。
- 控制器局部到达条件未变：水平/高度各 0.05 m 持续 0.1 s；随后满足原四足停车资格，并在水平 0.07 m、高度 0.06 m 内安静 0.5 s。
- TCP 目标离地高度保持归档训练值 0.09 m。没有另改为 1 cm。

到达误差始终相对**未经工作区裁剪的真实观测线尾**计算，不以参考限幅假装完成。

## 物理测试边界

使用 VA15 model_5000、MuJoCo、合成标定图像观测、IMU/关节局部估计。真值只生成相机观测和独立评分，不用于控制。相机检测点是合成输入，**没有把真实裂缝网络接入该物理闭环**。未安装实体枪/软管，仅评分虚拟焊缝 TCP；本结果不是负载验收，也不是实机精度承诺。

Python 67 项覆盖视觉协议和既有识别等回归；保存 JPEG 的真实图像几何回归仍引用不可变 V253 输出。不能与合成相机物理检查混称为真实视频全链路验收。

## 修复依据

### 训练/部署不一致

实际 checkpoint 的 source.tar.gz 中，目标位置先按训练工作区裁剪。原部署严格拒绝无完整可达段的目标，导致 V253 两组正常物理用例 TCP 激活为零。

V254 优先精确可达段；对已实际看到且已经位于后工作区附近的有限线段，显式启用受限恢复参考。真实观测、裁剪参考、分配误差分别保留。禁止用前方点外推虚构后方裂缝。

### 线尾投影与里程计误差

早期修复虽能激活 TCP，局部估计却把实际约 14～16 cm 的误差误当到达。现在可见期间用时空一致真实线尾继续刷新，并从静止线尾的多帧相对位移估计沿路径尺度，有限补偿离开视野后的增量。尺度只在充分视差和拟合质量满足时使用，不提升接触/支撑资格。

### 支撑停车参考边界

扩展测试查到四脚支撑已满足，但规划髋关节参考为 0.150204 rad，略超原 0.15 rad 包络而被拒。现在仅对原边界附近最多 0.02 rad 的规划参考做受限投影；仍需原接触、姿态、速度、补偿力矩条件。首次捕获命令保持力矩连续；大幅越界仍拒绝。

## 可复现命令（电脑离线，不连接机器人）

在本目录执行；需要现有 libtorch、MuJoCo 和历史依赖路径：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' -Wno-dev
cmake --build build -j2
ctest --test-dir build --output-on-failure

/home/dong/anaconda3/envs/crack_vision/bin/python verify_training_contract.py
/home/dong/anaconda3/envs/crack_vision/bin/python run_rear_completion_suite.py rear_endpoint_recheck
```

物理输出目录须使用尚不存在的新名字，避免覆盖证据。单测断言通过 -UNDEBUG 保持启用。

## 未通过记录保留

- V253 outputs/rear_endpoint_final：5/7，正常到达 0/2。
- V254 outputs/initial_recovery、refreshed_endpoint：早期误差超限。
- V254 outputs/rear_endpoint_final：15/19，含短线尺度估计和支撑捕获失败；其 tested_sources.tar.gz 保存对应旧源。
- **最终为 outputs/rear_endpoint_final_v2，19/19**，不以最终成功覆盖早期失败。

## 实机仍未放行

历史实机约 5.9 Hz 振荡没有在本物理测试中复现，不能声称已修复。真实 Z1 SDK 模式所有权/通信线程、板载时序、实际相机与标定连续性仍需单独验收。原 SDK 无 I/O 测试及有限协调器原型，不等于已经整改生产线程。
