# 共用视觉源码快照

这些文件从当前仿真实际使用的 `/home/dong/b2_z1/scripts/vision` 原样复制；
来源与逐文件SHA256见 [SOURCE.json](SOURCE.json)。原工作区保留。
VD12/13/14视觉入口现在直接使用本目录，不再要求旧训练工作区提供这些模块。
算法、控制参数和相机时效没有改变。

`crack_inference`、`crack_trajectory` 和RHANet网络定义取自
[RHACrackNet仓库](https://github.com/yu-dondon-z/RHACrackNet)的当前本地版本，
已将必要的20份Python源码收至 [rhacracknet/](rhacracknet/)，保留MIT许可证和独立哈希清单。
来源commit尚未在远程分支确认，因此当前部署直接使用附带副本，不依赖该commit在线可下载。
检测权重已保存到本仓库 `policy/vision/rhanet_dep_camcrack789_b2_stage1/`。
