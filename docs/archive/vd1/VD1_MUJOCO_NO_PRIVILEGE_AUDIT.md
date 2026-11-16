> 历史归档：内容和命令对应原记录日期，不是当前VD13操作指令。当前状态见[文档入口](../../README.md)。

# VD1 MuJoCo：观测边界与执行器契约审计

日期：2026-11-02，Asia/Shanghai。审查采用本地源代码、实际编译的 MuJoCo
模型及原导出 actor；没有连接、同步或启动实机。这里的审计结果不代替动态
行走、TCP 投影误差、足底距线或实际硬件验收。

结论：下述代码快照中，外部环境信息仅来自前 RGB 和后 RGBD。actor 另使用
机载可获得的 IMU、编码器和上一动作。未发现真实机身位置、真实 TCP、
真实接触或解析裂缝路径回流到控制命令及 actor。控制接口反事实测试在真实
MuJoCo 模型与原 TorchScript actor 上得到完全一致的 70 维输入和 18 维动作。

## 审查快照

部署仓库根目录：`/home/dong/rl_sar_b2_z1`。

| 文件 | SHA-256 |
|---|---|
| `scripts/play_vd1_mujoco.py` | `974eae993d21925975e6806968a83f36e064c7fe2f3cb8fb6233a0a0fdd19d1c` |
| `scripts/run_vd1_mujoco.py` | `3c09ce3b8a4a31e45e35c458fe76f615ecfbfcc4cbed5f46b41e6d420121acc6` |
| `scripts/vd1_mujoco_contract.py` | `d03dfc3b22a5121b1c3537be5ab80690b02b0d74af6abdf7b94e1e4c104444c8` |
| `scripts/vd1_mujoco_scene.py` | `9c7b93c39486c60b3b3382d4349c8d5148048ab5e987b4dafc4b6625e944355c` |
| `tests/test_vd1_mujoco_contract.py` | `0d75f964e11a7f878e56a9ffc6348206bd42e9615d60daeddadf9f51aaa61770` |

复用的训练仓库根目录：`/home/dong/b2_z1`。

| 文件 | SHA-256 |
|---|---|
| `scripts/vision/vd1_control.py` | `167b7e793971052e45aae5d7daee9fb1e29462671ce228929a781088c751b01f` |
| `scripts/vision/vd1_frames.py` | `8799c41ed43843afcd4eba13c8ede05d80c744776425a282b142dda5dc36c475` |
| `scripts/vision/dual_camera_perception.py` | `3cb8ed1f4e689d56bf5c7008aed0269b4997255b7e83d6f1753cfe754a914e2d` |
| `scripts/vision/run_dual_camera_bridge.py` | `f87f37c550b4d454dda85bc54e8a44591c799e78d5b06b803891d756d372d888` |

Actor SHA-256 为
`86be6c2d1c7a0e56fe4bd93c2d62622469c030b0cdc6d10db7af001eae3f889a`。
源 URDF SHA-256 为
`14ae3abccfa4d7e5f74f69347ced43f7b1ec5bed728376972260d8b18130c9c6`。
后续实质修改应重新核对接线；会话 `session.json` 也保留启动及结束时文件哈希。

上表为本地 VD1 MuJoCo 当前接线的哈希快照。下方仍保留更早的实验候选记录，
其旧哈希只用于追溯，不代表当前文件内容。

## 数据从哪里进入控制

`run_vd1_mujoco.py:191` 的 `capture()` 每次显式指定两台固定机载相机。
先渲染前 RGB，再渲染后 RGB 和同一后相机的对齐深度。没有读取前深度、
语义标签、物体 ID、真实 TCP 点或解析路径作为感知输入。MuJoCo 3.2.3
`Renderer.render()` 将图形深度转换为米制光学 Z；publisher 再按毫米量化
写入 PNG，并给出 `depth_unit_m=.001`。无效深度不会通过真实几何补齐。

`run_vd1_mujoco.py:177` 发布的安装位置、ROS 光学四元数及内参来自固定
标定常量，不读取相机或机身的实时世界位姿。`vd1_frames.py` 输出图像文件、
固定标定、帧身份和时间；没有输出机身世界位置/方向或真实 TCP。

后 RGBD 的地面和工具标记通过 `dual_camera_perception.py:444` 的深度
反投影、地面拟合及已知半径彩球表面拟合测得。前 RGB 通过 `:363` 的像素
射线与同步后深度测得平面相交。彩球是枪刚体下的固定可见几何，局部位置和
尺寸属于工具标定，不会根据每帧真实 TCP 或目标点重新定位。调试 TCP site
透明，不作为视觉标记；场景不绘制未来目标点或真实路径 overlay。

旧通用感知桥仍有兼容性代码，可以透传 metadata 中已有的 `base_position_w`
等字段；本入口的 publisher 不产生这些字段，且在结果进入控制前递归拒绝
包含这些字段的 payload。这项结论针对本 VD1 接线，不能推广为旧桥在任何
上游输入下都无特权信息。

`run_vd1_mujoco.py:51` 先经 publisher 私有捕获历史验证返回包的代际、
序号和捕获时间，再由 `:274` 只以 packet、单调时间、`dt=.02` 调用纯视觉
控制器。结果几何保留在捕获时机身坐标系，没有用仿真世界位姿补偿延迟。
真实过期帧按原时间保持过期；没有接触或真值路径兜底。

## IMU、编码器和 actor

`vd1_mujoco_contract.py:133` 起，运行时只从 `data.sensordata` 读取具名
关节位置、关节速度、`imu_gyro` 及 `imu_orientation`。构造时验证每个
sensor 的类型、尺寸及关联关节/site；不能只靠容易错位的数组下标。
actor 顺序独立于 MuJoCo 的深度优先关节顺序。

姿态使用 MuJoCo `framequat` 模拟理想 IMU 姿态输出，计算
`R(q).T @ [0,0,-1]` 后仅返回单位投影重力；不返回 yaw、世界位置或基座
线速度。测试验证纯 yaw 改变不会影响重力，roll 改变会正确影响重力。
这是理想姿态传感器假设，没有实现或证明真实 IMU 的融合、漂移和动态噪声。

初始 TCP 仅由 `run_vd1_mujoco.py:131` 的 URDF 正运动学从六个编码器角
计算。没有用 `data.site_xpos` 或 `data.xpos` 初始化末端控制目标。

`run_vd1_mujoco.py:276` 显式构造下列 70 维观测；只加载原导出 actor，
不加载 critic、AMP 判别器或旧 FSM：

| 零起始下标 | 内容 | 处理 |
|---|---|---|
| 0–2 | 视觉产生的本体速度指令 | 裁剪至 ±100 |
| 3–9 | 末端目标 xyz 与 wxyz | 裁剪至 ±100 |
| 10–12 | IMU 机身角速度 | 先裁剪至 ±100，再乘 0.2 |
| 13–15 | IMU 投影重力 | 裁剪至 ±100 |
| 16–33 | 编码器角减训练默认角 | 裁剪至 ±100 |
| 34–51 | 编码器角速度 | 先裁剪至 ±100，再乘 0.05 |
| 52–69 | 上一次原始 actor 动作 | 裁剪至 ±100 |

`:284` 使用 `clip(default_q + .25 * raw_action, -100, 100)`。
`:301` 记录的是 raw action，没有改成关节目标或增量。每个 actor 周期执行
4 个 5 ms PD 周期，每个 PD 周期执行 5 个 1 ms 积分子步。仿真时间 actor
频率为 50 Hz，PD 更新为 200 Hz；渲染和感知较慢时墙钟运行可慢于实时。

## 真值读取为何不回流

`EvaluationRecorder`（`run_vd1_mujoco.py:60`）读取真实机身位置、TCP、
足端位置、接触力和物理道路中心线，仅写 `evaluation_truth.jsonl.gz` 及
退出摘要。`write()` 返回 `None`，调用位于当次动作和物理步进之后
（`:300`）。控制器和 adapter 没有 evaluation 引用，没有从评价文件读回
目标、姿态或指令。解析 `curvature_max` 只用于场景和评价，不交给控制器。

`:216` 和 `:306` 的真实机身位置只更新展示相机的 `lookat`。截图使用独立
临时展示相机，写到 `viewport_robot.png`；下次 `capture()` 会重新显式指定
`front_rgb` 和 `rear_d435i`，所以展示相机不会被当作机载传感器。

这不意味着渲染器不能读取真实场景：渲染是生成仿真传感器图像所必需的。
审计边界是经过图像成像及机载传感器接口获得信息，禁止绕过这些接口将
真实几何状态直接交给控制器。

## 执行器与跨引擎边界

腿采用原 `160*(target-q)-5*qd` 显式 PD；髋/大腿限 ±200 Nm，小腿限
±320 Nm。Z1 保持原生位置执行器 `kp=160, kv=5`，合并 PD 力矩限
±120 Nm；使用 MuJoCo `implicitfast`。没有把 Z1 统一改成显式高增益力矩。
adapter 构造时拒绝额外被动 damping、armature 或重复 URDF 原摩擦；此场景
使用 `frictionloss=.01`。这些参数映射并不构成 PhysX 与 MuJoCo 动力学严格
等价的证明，特别是接触求解器、摩擦模型和积分方式仍有差异。

原 PhysX 速度约束为腿 23/14 rad/s、臂 3.1415 rad/s。MuJoCo 没有这里可
直接对应的求解器关节速度上限，因此只记录越限及最大比例；没有剪裁实际
`qvel`、改小目标变化或改增益来伪装等效。跟踪性能需实测，不把这一差异
解释为已经复现训练执行器。

`actuator_diagnostics.effort18` 中腿项是本次命令力矩，臂项是瞬时 PD 估计，
通过 `arm_effort_is_estimate=True` 标明。它不等于 MuJoCo 隐式积分后的
原生执行器实际力矩；当前独立评价主要记录足端力，没有因此证明真实执行器
力矩响应符合训练模型。

## 已实际执行的验证

```bash
cd /home/dong/rl_sar_b2_z1
/home/dong/anaconda3/envs/crack_vision/bin/python -m unittest discover \
  -s tests -p test_vd1_mujoco_contract.py -v

/home/dong/anaconda3/envs/crack_vision/bin/python -m unittest discover \
  -s /home/dong/b2_z1/scripts/vision -p 'test_vd1_*.py' -v
```

结果为 11 项 MuJoCo 契约测试和 36 项复用控制/帧/几何测试通过。
MuJoCo 契约测试实际生成并编译原 VD1 模型、加载原 actor 后检查：

- 固定传感器和视觉输入，改变真实根位置、根方向、根速度，执行 `mj_forward`
  后确认真实 site 位置和接触数都变化，再恢复固定 sensor 缓冲；两次 70 维
  observation 和 18 维 actor action 逐元素完全相同。
- 仅提供 `sensordata` 和 `ctrl` 的代理对象，对其他属性读取立即报错；
  仍可完成观测读取、原 actor 推理及设置执行器目标。
- 实际关节深度优先排列与 actor 顺序不同，但编码器和 actuator 经名称映射
  后对应正确。错绑编码器对象、盲改机械臂增益、叠加被动阻尼均被拒绝。
- 无效姿态/非有限 sensor 输入被拒绝；腿力矩饱和，机械臂 ctrl 保持原目标角；
  速度越限只报告；设置目标不会步进、篡改 qpos/qvel 或传感器缓冲。

反事实只覆盖明确测试到的数据边界，不是对 MuJoCo、神经网络感知及整个
进程空间的形式化证明。图像噪声、标定误差、延迟、遮挡、完整曲线和实机
安全性都需另行验证。`--static-only` 当前冻结物理步进，仅检查相机/模型，
不能当作站立、行走或跟踪通过；动态成绩应以本次新运行日志为准。

## 显式速度约束候选附录

后续 runner 增加 `--velocity-constraint off/implicit_brake/native_velocity`。
本次复核时默认仍为 `off`；两个近似均不得因单项测试通过就默认为已解决
动态问题。此前连续启用 native velocity 的版本存在“力为零但隐式导数不为零”
的错误行为，其动态记录必须保留并标为失败，不能把阻滞关节误报成限速成功。

| 复核文件 | SHA-256 |
|---|---|
| `scripts/run_vd1_mujoco.py` | `9cf3a1f7b67860697cebfbbcbbb3415c65b5e87e77a9ae17816cd902b083f13c` |
| `scripts/vd1_mujoco_contract.py` | `3dfa7c174b2ad2baf870e4edee7e175c08105c03ef0ffd8659b8f4c6d7a87634` |
| `scripts/vd1_mujoco_scene.py` | `39a204837aee2e1757ef63a35d0d7accf9deb98ede2649853da991a0ca0b7134` |
| `scripts/vd1_mujoco_velocity_solver.py` | `18ebe50157f20e41d60f3811d63e7fc423f7f5f7363faf9e782fcfa6aec65dbf` |
| `tests/test_vd1_mujoco_velocity_solver.py` | `8921f9924d14b828a9afff1889324881ed121d5cf11d8c0697f490a1a870829e` |

`native_velocity` 为每个原 hinge 添加上下限各一个单边 velocity actuator，
原18个robot motor保持不变，额外36个只作为模拟器约束反力。adapter只写
原18个具名motor；`nu=54` 时构造器额外验证这36个的关节、gear、传递方式、
单边 force 范围和 velocity actuator 参数，不能将其误当作54维actor。
默认 `off` 场景仍为18个motor。

约束阻尼由初始化时有效质量除以正则时间得到，属于有限柔性速度约束，
不改原机器人 `kp/kd`。helper 每个1 ms步从实际物理 `qvel` 判断约束是否
激活：未超上/下界的对应 actuator gain 和 bias 同时归零，阻止隐式积分器
将零力项的导数错误地当作额外惯性。实际超限才启用对应单边项，另一方向
保持关闭；不从预测速度提前启用未超限项。

helper 的真实状态读取仅用于模拟器物理层制动，返回统计供记录；runner
没有将这些数据交给视觉控制器或70维观测构造器。没有直接写 `qpos/qvel`、
裁剪编码器、改 actor 动作或重写关节目标。原机载观测边界未扩大。

该方法不是硬上限：当前在界内、下一1 ms刚越界时，制动尚未激活，会有
首跨超速。summary 明确 `hard_bound_guaranteed=False`，记录步后的最大
超速比例。反力 cap 独立于原120/200/320 Nm电机限制，不将约束反力冒充
电机真实输出。记录的瞬时力和功来自隐式线性化点，不能当作精确步内做功。

新增9项测试和原11项契约测试共20项通过，包括：

- 单DOF在界内施加非零驱动力，带关闭制动器和完全无制动器的 `qpos/qvel`
  逐元素相同。具体回归为球惯量`.004`、`dt=.001`、`v0=1`、外力矩`1`，
  步后速度严格为`1.25`，不会出现旧错误版本的`1.009615`。
- 正负超速时反力方向耗散，单自由度动能下降；自由基座对内部制动产生
  角速度响应，总角动量守恒。
- 原VD1真实模型54个actuator编译并通过质量/惯量验证；原18个motor的
  gains/bias/default姿态不变；界内反力为零、界外功率方向正确。
- 首跨超速被测试和报告；源代码没有直接向状态和sensor缓冲赋值。

```bash
cd /home/dong/rl_sar_b2_z1
/home/dong/anaconda3/envs/crack_vision/bin/python -m unittest discover \
  -s tests -p 'test_vd1_mujoco*.py' -v
```

这些测试证明候选没有上述已知的零力导数阻滞和直接状态裁剪，不证明原策略
已能在MuJoCo站立、行走或完成跟踪。动态候选的成功/失败另按新会话记录。

## 后续冲量求解器及位置积分次序

连续隐式制动及修正版原生单边制动在完整机器人动态实验中仍未解决跌倒。
保留这些实现和日志，不将其测试成绩当作动态修复。新的显式
`--velocity-constraint impulse` 采用完整质量矩阵下的物理约束冲量候选。
本次复核 runner SHA-256 为
`66b66d3152547cf93ee7b212cfd559cdb79d727e1d78285ff0613b5096a564fa`，
入口支持`--physics-dt .001/.005`，仍保持50 Hz actor、200 Hz原PD。

| 文件 | SHA-256 |
|---|---|
| `scripts/vd1_mujoco_velocity_impulse.py` | `f5e1229e83f0ab7a2eab49495d80bfc53472f34ce74e098a3e88436de912e072` |
| `tests/test_vd1_mujoco_velocity_impulse.py` | `eb1603924c73a4631dab44b4092110174eeb969268dfd950a0976cbafe3c47d7` |

每个物理步先保存`q_n`及`M(q_n)`，由MuJoCo求自由速度`v_free`，再求解
`min .5*(v_plus-v_free)^T M(q_n) (v_plus-v_free)`，约束18个关节速度
位于原训练上下限内。PGS求解36个单边乘子；关节内部冲量通过完整逆质量
耦合改变全部24个广义速度，包括浮动基座反应。这不等于只对18个编码器
速度独立clip。

实现**明确存在物理状态写入**：收敛及整体动能不增检查通过后，唯一的
`data.qvel[:] += coupled_delta_v`应用完整耦合冲量；再使用
`mj_integratePos`从原`q_n`按受限`v_plus`重积分，唯一一次写回`data.qpos`。
不能声称该候选完全不写速度或位置状态。这些写入属于新增物理约束的
离散积分，不属于传感器或观测补丁；随后调用`mj_forward`让模拟IMU和
编码器来自最终真实物理状态。代码没有手工写`sensordata`、伪造接触、改
actor动作、末端目标或原PD参数。

最初“MuJoCo积分位置后只投影速度”的候选保留失败记录；其`qpos`已按未
受限速度走过过大位移，不能只靠最终速度达标判定正确。当前次序修复后，
1 ms与5 ms的自由hinge回归均验证实际位移不超过`limit*dt`。

它仍不是PhysX/TGS求解器的精确复现：接触与关节速度约束采用operator
splitting，并未在同一约束系统联合迭代。物理内部速度和质量仅供求解器，
不返回控制器当作额外输入。跟踪、接触和真实机器人响应仍需独立验证。

新增7项验证包含原VD1五组全系统随机超限状态的PGS/KKT收敛、整体动能
不增加；`M*delta_v`仅在内部关节DOF有冲量但基座速度仍反应；自由基座
总角动量及线动量守恒；无超限`project()`为identity；独立`project()`不改
位置；完整`step()`在1/5 ms按受限速度积分；模拟编码器由引擎重算且等于
物理关节速度；源代码只有明确标记的速度增量和位置积分写入，没有sensor
缓冲赋值。

Summary包含速度残差、KKT残差、冲量、等效约束反力、投影动能变化，以及
`state_velocity_impulse_applied=True`、`independent_component_clip=False`、
`position_integrated_from_constrained_velocity=True`、
`exact_physx_solver=False`。测试通过不代替本候选的新动态部署记录。

### 源增益不变时的隐式位置弹簧离散转换

该项是单独、默认关闭的数值兼容实验，不是对源控制增益的搜索。PhysX
官方[隐式关节弹簧推导](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/_downloads/6acf3afb8f69452757e0e766b5a22978/implicitDrives.pdf)
在步末位置及速度计算弹簧阻尼力；MuJoCo 3.2.3 的
[implicitfast 积分](https://mujoco.readthedocs.io/en/3.2.3/computation/index.html#numerical-integration)
只隐式处理速度导数，不包含位置刚度导数。对无饱和、单自由度、固定惯量
的线性位置驱动，PhysX 所述隐式 Euler 一步满足
`v_next=(I*v+h*kp*(q_target-q))/(I+h*kd+h*h*kp)`。因此在本版本
MuJoCo 的原生 position actuator 中，仅用于积分的速度系数可写为
`kd_discrete=kd+h*kp`，使此受限条件下的一步与解析式相同。

实际 MuJoCo 3.2.3 的单关节测试取 `I=.004`、`q=.1`、`v=.2`、
`q_target=.2`、源 `kp=160,kd=5`：

| 步长 | 原生 kv=5 的下一步速度 | 解析隐式弹簧速度 | 转换后的 kv | 转换后速度 |
| --- | ---: | ---: | ---: | ---: |
| 5 ms | 2.786206896551724 | 2.448484848484849 | 5.8 | 2.448484848484849 |
| 1 ms | 1.866666666666667 | 1.834061135371179 | 5.16 | 1.834061135371179 |

`VD1MujocoAdapter(model, physx_implicit_position=False)` 保持默认源
160/5 行为。显式开启后，构造函数先验证模型原始关节、传感器、原生驱动
160/5 完整契约，再只改变六个机械臂驱动的速度 bias 系数为
`-(5+model.opt.timestep*160)`。腿部显式 PD、原生位置 gain、策略输出、
目标角及所有传感器均不改变。适配器公开 `source_kp18`、`source_kd18`、
`arm_discrete_kd18` 及源 PD 未限幅诊断，便于记录来源及有效离散系数。

新增 `tests/test_vd1_mujoco_implicit_position.py` 两项测试覆盖两个惯量、
1/5 ms 解析一步等效及实际模型只改变六轴的预期系数。该结果不证明完整
机器人、力饱和、接触、速度约束和 PhysX/TGS 子步在两个引擎之间等效，
也不证明此前的摔倒由此项导致。道路碰撞几何异常另行修复验证，动态
试验应分别记录道路修正与此可选转换，避免混合归因。

此版适配器 SHA256 为
`d03dfc3b22a5121b1c3537be5ab80690b02b0d74af6abdf7b94e1e4c104444c8`。
本地命令 `python -m unittest discover -s tests -p 'test_vd1_mujoco*.py' -v`
在 `crack_vision` 环境通过全部29项；另4项场景碰撞测试和36项复用控制/帧/
几何测试也通过。它们不涉及真实机器人，也不替代当前场景修正后的动态运行结果。

### 本次闭环结果与边界

2026-11-02 的30秒直线候选使用新路面、RHANet、固定0.20 m/s、`impulse`、
5 ms物理步及显式开启的 PhysX 隐式位置驱动离散转换。共1500个 actor 步，
1416步有有效命令，仿真未触发运行时错误或倾斜停止。独立评价得到 TCP 地面
投影误差 P95=7.51 cm、最大13.41 cm，足端边缘净空最小=-4.15 cm；后5秒
P95约9.60 cm，故不满足此前5 cm及足端避线目标。该速度约束会在物理积分
内部写入耦合速度增量并按受限速度重积分位置；它不是 PhysX 精确求解器，
也不意味着实机执行器有相同行为。

250步 GUI 运行使用相同物理参数，前 RGB 与后 RGBD 均产生有效配对观测，
仿真窗口及视觉叠加窗口可启动；这只确认可视化链路和短时运行，不代表稳态
跟踪精度。30秒运行的 RHANet 推理中位数约148 ms、P95约219 ms；控制器
接收测量的采集年龄中位数约293 ms、P95约496 ms，最大约797 ms。延迟可能
影响闭环质量，但现有数据不足以将误差唯一归因于时延；控制路径不使用世界
位姿或真值进行延迟补偿。

短时和30秒记录位于 `/dev/shm/vd1_mujoco_gui` 与 `/dev/shm/vd1_mujoco_tests`，
这是临时内存盘，机器重启后记录可能消失。仿真、视觉子进程由本机启动器
创建并关闭，`hardware_started=false`；没有同步板载、连接实机或触发机器人动作。
