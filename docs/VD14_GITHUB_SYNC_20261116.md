# VD14 GitHub训练与部署快照

2026-11-16。本仓库 [VD14分支](https://github.com/yu-dondon-z/rl_sar_b2_z1_v1/tree/VD14)
保存当前MuJoCo双视觉部署、真值诊断回放、测试及文档；
配套训练仓库为 [yu-dondon-z/b2_z1，VD14分支](https://github.com/yu-dondon-z/b2_z1/tree/VD14)。
保留旧分支，不修改默认actor，不同步板载、不启动机器人。

## 模型与依赖

- RL模型：训练仓库 `test_ckpt/b2_z1_vd14_posture_amp/2026-11-16_10-41-45_vd14_posture_long_20261116/model_5999.pt`，
  同目录含必须配套的 `params/` 与校验清单。
  SHA256：`49e41e91c9e82e178a7e8ea9f2f835a6b8e440a131ebaeae3c2911f68f5d3372`。
- 原有RHANet检测权重已收至
  [`policy/vision/rhanet_dep_camcrack789_b2_stage1/`](../policy/vision/rhanet_dep_camcrack789_b2_stage1/)，未训练或转换。
  SHA256：`f1eea726c85773c3ddb71649bdae097eff6ca7693c7f36ad488fa9f4643bdb80`。
- 原来依赖旧工作区的共用视觉模块，现按原字节收至
  [`scripts/vision/`](../scripts/vision/README.md)，附来源和哈希，避免漏传。
- RHANet网络、`crack_inference`、`crack_trajectory` 所需20份源码已从本地
  [RHACrackNet](https://github.com/yu-dondon-z/RHACrackNet)收至
  [`scripts/vision/rhacracknet/`](../scripts/vision/rhacracknet/)，保留MIT许可和文件校验。
  当前视觉入口使用附带副本，无需再依赖第三个本地工作区；旧诊断脚本仍可能保留历史路径。

当前入口沿用开发电脑目录 `/home/dong/b2_z1_VD2` 与 `/home/dong/rl_sar_b2_z1`，
MuJoCo环境为 `/home/dong/anaconda3/envs/crack_vision/bin/python`，需能导入
`mujoco, torch, cv2, numpy, yaml`。Isaac训练/Play另用 `env_isaaclab`。
换机需准备上述依赖、训练源码/资产和目录映射；不是任意路径解压即用的独立安装包。

## 当前电脑直接看双视觉部署

```bash
cd /home/dong/rl_sar_b2_z1
bash scripts/deploy_vd14_vision.sh
```

默认打开MuJoCo与识别窗口、持续运行，无30秒自动停止。
如从GitHub只恢复了附带模型而没有本机训练日志，明确指定模型位置：

```bash
cd /home/dong/rl_sar_b2_z1
bash scripts/deploy_vd14_vision.sh \
  --checkpoint /home/dong/b2_z1_VD2/test_ckpt/b2_z1_vd14_posture_amp/2026-11-16_10-41-45_vd14_posture_long_20261116/model_5999.pt
```

纯动作回放不含视觉，入口为 `bash scripts/play_vd14_mujoco_truth.sh --viewer`，
同样可用 `--checkpoint` 指定上述检查点。

## 状态和范围

后视目标支持不足只对VD14指定情况进入零行走WAIT，窗口/物理继续，合格连续目标恢复后继续；
不把旧命令冒充新观测。相机时效、其他视觉错误和物理保护仍保留。
已有60秒样本包含8次等待和恢复，`visual_sample_passed=false`；
仍是仿真候选，不是长时跟踪或实机验收。详细证据和未解决项见
[实跑记录](reports/vd14/VD14_PLAY_MUJOCO_20261116.md)。

保存当前源码、模型、测试和报告；不上传运行日志/视频、历史补丁里的实拍帧/遥测、
构建产物、缓存或本机IDE设置。历史板载包和补丁源码保留原状态，**不是VD14实机部署包**。
报告里的本机 `outputs/` 链接仅供原工作区追溯，不表示原始数据已上传。

## 打包验证

训练侧VD14共23项、部署侧VD14共52项、共用视觉44项回归通过；
另有VD12 82项、VD13 14项兼容检查通过。检查点、参数、视觉源码和检测权重的哈希均已核对，
附带RHANet源码/权重实际CPU推理成功。打包仅调整源码/权重查找路径，不改算法或参数，
没有因此重复训练或重跑动力学验收；原运行日志仍对应各自manifest中的源码哈希。
