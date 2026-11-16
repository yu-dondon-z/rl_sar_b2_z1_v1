# 部署回归测试

现有测试覆盖视觉、路径缓存、模型/执行器契约、步态和 actor 归因。它们仍验证现有脚本和模块，按原目录保留。

原 `scripts/test_vd1_mujoco_scene.py` 已移入本目录，使物理接触回归检查也能被统一发现。它与其他 MuJoCo 测试需要对应 Python 环境：

```bash
python -m unittest discover -s tests -p 'test_vd1_mujoco_scene.py' -v
python -m unittest discover -s tests -p 'test_vd6_actor_feedback.py' -v
```

VD12双相机视觉入口、时效／来源校验、新枪近面遮挡及VD5兼容回归（使用现有视觉环境，无机器人I/O）：

```bash
cd /home/dong/rl_sar_b2_z1
env PYTHONPATH="$PWD/scripts/vision:$PWD/scripts/vision/rhacracknet" \
  /home/dong/anaconda3/envs/crack_vision/bin/python -B -m unittest discover -s tests -p 'test_vd12*.py' -q
env PYTHONPATH="$PWD/scripts/vision:$PWD/scripts/vision/rhacracknet" \
  /home/dong/anaconda3/envs/crack_vision/bin/python -B -m unittest discover -s tests -p 'test_vd5*.py' -q
```

这些测试通过只表示对应回归通过，不代表连续视觉闭环或实机验收通过。实际图像实跑及失败原因见[VD12视觉部署记录](../docs/reports/vd12/VD12_VISION_DEPLOYMENT_20261114.md)。

`test_rear_mount_geometry.py`包含75cm夹持候选诊断的6项纯CPU投影/地面范围/不跨间隙检查，不代表候选动力学或视觉部署通过；见[静态核对](../docs/reports/vd12/REAR_MOUNT_CANDIDATE_20261115.md)。

`test_vd13_mount75_model.py`核对独立75cm模型的MuJoCo编译质量、COM、完整惯量、碰撞体和TCP／几何守护器一致性（3项）。同样不能代替策略或双视觉验收；[训练决定](../docs/reports/vd13/VD13_MOUNT75_DECISION_20261115.md)。

`test_vd13_play_entries.py`核对VD13两端入口、固定最终检查点、旧版本隔离和安装／课程／工作距离拒绝逻辑（11项）。与模型测试合计14项；使用`crack_vision`环境运行`python -B -m unittest discover -s tests -p 'test_vd13*.py' -v`。真实视觉闭环仍未通过，见[实际运行记录](../docs/reports/vd13/VD13_PLAY_MUJOCO_20261116.md)。

部署包内部的 `deployment/vd2_real/tests/` 随包分发，保持原布局；`patches/` 内测试属于对应旧候选，参见 [补丁索引](../patches/README.md)。一次性分析放在 [scripts/diagnostics/](../scripts/diagnostics/README.md)。

`test_vd14_play_entries.py`有9项：固定最终5999、实际姿态奖励／profile、旧策略拒绝、安装和课程一致性、默认RHANet／传感器补偿及保护参数。`test_vd14_motion_compensation.py`另有14项：局部编码器里程计、刚体变换、原始帧身份／时效／特权字段拒绝、预测50ms上限、真实touch传感器、FK／Jacobian和添加site不改变动力学。`test_vd14_visual_evidence.py`有12项：暗脊证据／原掩码相交、真实误选帧、缺口保留、拍摄目标延后验证仍受控制端支持限制、候选时刻语义／诚实标记、20cm工作点跳变拒绝、近π四元数符号与训练纯转换数值对照。共35项。使用`crack_vision`环境运行`python -B -m unittest discover -s tests -p 'test_vd14*.py' -v`。回归通过不等于[双视觉动态验收](../docs/reports/vd14/VD14_PLAY_MUJOCO_20261116.md)。

`test_vd14_deployment_runtime.py`有7项：VD14默认双窗口／持续运行和关闭开关、可选定时时长超过30秒、流式无限计数、旧入口默认不变、天空不添加几何／改变动力学、前后RGB及depth逐像素不变、独立观看模型250m与相机6m裁剪隔离且显示修改不回馈源状态。VD14现共42项；渲染测试可设置`MUJOCO_GL=egl`无窗口运行。持续运行被用户停止不是采样验收通过，原视觉／物理保护仍保留。

`test_vd14_support_wait.py`新增10项：后视外推支持不足不再终止，仅零行走等待／恢复；5cm限值不放宽，旧命令按传感器运动传输、来源和年龄不刷新，缺测不冒充有效观测，等待不放宽恢复跳变边界；过期、换代及篡改帧仍锁止，首次启动及旧版本不凭空生成目标。VD14合计52项。运行命令同上，不能替代[实际60秒等待／恢复样本](../docs/reports/vd14/VD14_PLAY_MUJOCO_20261116.md)。
