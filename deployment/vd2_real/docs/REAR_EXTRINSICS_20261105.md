# 后置 D435i 外参部署

本更新使用 `rear_camera.json` 中给出的完整变换：

```text
T_base_camera =
[[ 0,        0.770435, -0.637519, -0.434510],
 [ 1,        0,         0,         0       ],
 [ 0,       -0.637519, -0.770435,  0.026751],
 [ 0,        0,         0,         1       ]]
```

相机轴为右、下、前；B2 机身轴为前、左、上。矩阵旋转部分正交、行列式为 +1。来源文档将它描述为“近似安装外参”，所以配置标记为 `calibration_quality: provisional_installation_estimate`。它可以用于当前部署验证，但不能据此承诺 TCP 投影精度。

该更新安装 `config/rear_d435i.yaml`，并修复准备/收臂时微小编码器零位越界导致目标被拒收的问题。用户预检 joint2=-0.003239 rad、joint3=+0.002053 rad，分别略低于 0 下限和略高于 0 上限；原程序把原始读数当作保持目标，站起时会被已有 C++ 限位拒绝。现在仅将不超过 0.01 rad 的目标越界限制回物理范围，原始编码器、策略观测与趴姿记录不改写，较大越界仍拒绝；日志记录修正前目标与实际目标。策略权重、C++ 和增益不变。

它依赖先安装 `vd2_camera_fix_20261105`，因为后置 D435i 的 inverse Brown-Conrady 处理在相机修复中。

## 板载操作

在开发电脑传输：

```bash
scp /home/dong/rl_sar_b2_z1_VD2_deploy_ready_20261105.zip unitree@192.168.123.164:/home/unitree/
```

板载退出相机预览后执行：

```bash
cd /home/unitree
unzip -n rl_sar_b2_z1_VD2_deploy_ready_20261105.zip
cd /home/unitree/rl_sar_b2_z1_VD2
source .venv/bin/activate
python ../vd2_deploy_ready_20261105/tools/install_extrinsics.py --target .
python run.py --offline-check
```

安装器会备份原配置到 `patch_backups/rear_extrinsics_<时间>/`，遇到已有不同修改时不会静默覆盖。无需重新编译 C++。

## 重新确认相机和状态

先预览：

```bash
python run.py --camera-preview --interface eth0 --display
```

再退出预览并运行：

```bash
python run.py --preflight --interface eth0 --seconds 10
```

预检中的 `rear_calibrated` 应为 `true`，且 `fresh: true`、`z1_mode: 1`。预检不接管 B2 行走。正式 `--run` 前仍应确认相机安装未移动，并把第一次运行的 `runs/<时间>/` 日志保留。

## 现场操作

确认现场机器人静止趴着，退出预览/预检、保持 `z1_ctrl` 在独立终端运行后，由现场人员启动：

```bash
python run.py --run --interface eth0
```

手柄 A 站起，等 `standing`；Y 机械臂准备，等 `ready`；R1+Y 开始视觉跟踪，再按切换停止；X 停止；B 完成停止、收臂、趴下；L1+X 请求阻尼/PASSIVE。按键每次松开后重新按，启动时按住的键不触发动作。行走指令为 0.20 m/s，转向来自前 RGB，不预设实机曲率。

相机近似外参可能带来枪尖目标偏移，且当前后视生成焊缝目标，没有识别实际 TCP 的独立视觉闭环。上述准备只通过离线验证，不代表实机步态/停止和 5 cm 投影精度已验证。
