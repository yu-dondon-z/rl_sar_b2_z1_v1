# VD2 单终端启动更新交接

用户现场 `20261106_091151_495675` 主程序在 8 秒内未等到 B2/Z1 新鲜状态，Z1 SDK 输出 `UDP loses connection`；没有接管 B2。随后用户要求一个终端同时启动 `z1_ctrl` 与主程序。

新增 `deployment/vd2_real/start_real.sh`、`tools/launch_real.py`。默认使用包内 `.venv/bin/python`，控制器工作目录默认为 `/home/unitree/z1_controller/build`，等待 UDP 8071 绑定后启动原 `run.py --run`。已有单个真实 `z1_ctrl` 进程会复用；多个控制器、未知 8071 占用或 8072 被 SDK 占用时拒绝启动。目录级锁防止同目录第二个启动器并发启动。

Ctrl+C 转发给主程序；其退出关闭原有 IO 后，停止本启动器自行创建的 Z1 控制器。复用外部控制器时不停止它。启动失败、主程序失败、控制器退出有对应收尾。端口绑定不代表实际反馈新鲜，原主程序的时效、手柄接管和停止状态机不改。无增益、策略、外参或控制代码修改。

新增 `tests/test_one_terminal.py` 六项测试：自动启动/工作目录/退出顺序及日志；复用已有控制器；8072 冲突；控制器启动失败；主程序错误码；Ctrl+C 顺序。测试仅使用临时伪进程和本机回环 UDP，不连真实机器人。

更新包 `/home/dong/rl_sar_b2_z1_VD2_one_terminal_20261106.zip`，12,379 字节，SHA256 `6916f990a7724fa4a35321ec89682aae35a5999fac6440b47f7632086fa767ab`。安装器只添加新入口、工具、文档和测试，更新/备份清单；保留现有配置和日志。没有同步板载或启动实机程序。

验证副本 `outputs/VD2_ONE_TERMINAL_20261106_x1cXLd/rl_sar_b2_z1_VD2`，在已应用相机修复及 deploy_ready 更新的副本上解压安装最终补丁，`python run.py --offline-check` 31/31 通过。Shell 语法和 `bash start_real.sh --help` 通过；重复安装无修改。

操作说明见 [单终端说明](../../../deployment/vd2_real/docs/ONE_TERMINAL_20261106.md)。现场入口 `bash start_real.sh --interface eth0`，状态检查入口 `bash start_real.sh --preflight --interface eth0 --seconds 10`。仍由现场新按 A 站起、Y 准备、R1+Y 跟踪，正常 B 趴下完成后 Ctrl+C。实机状态恢复、运动与精度需要实际日志确认。
