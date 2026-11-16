# VA15 实机 Ctrl+C 退出修复（2026-10-20）

状态：已在 PC4 备份、同步、原生编译及离线检查；**未由助手启动实机
控制、起立、行走或 Z1 运动**。需要操作者先在趴卧状态验证退出。

## 实机证据和原因

用户先按 B，日志已进入 `RLFSMStatePassive`，随后 Ctrl+C 出现所有
`real_*` 控制循环结束，但继续打印 Z1 UDP 超时、不返回终端。
检查发现主进程 PID 60596 仍在，子进程 60620 (`z1_ctrl`) 已是 zombie；
二者 PGID 都是 60596。主线程在 nanosleep，SDK 通信线程仍在 select。

主程序析构在停止 B2 循环后调用 SDK `setFsm(PASSIVE)`。本地和板载
配套 SDK 的反汇编显示：该函数反复等待 `recvState.state` 相等，没有
超时，也不检查 `setWait(false)`。对端已退出，因而无法结束等待。
`nohup` 也不能隔离终端进程组的 SIGINT，SDK 子进程安装自己的信号
处理器后仍可能与主程序一起退出。

完整线程栈因 PC4 ptrace 权限限制未取得；没有为此修改系统安全设置。
操作者明确确认 B2 趴稳、Z1 收紧且失去力矩不会碰撞后，只结束了这一个
卡住的 PID（SIGTERM 未结束，随后 SIGKILL）；没有重启机器或其他服务。

## 改动范围

当前本机完整阶段源码：`../va15_stop_feedback_20261020/work/`。
本机根目录仍是较早的基线，不以整目录覆盖板载。

板载仅更新：

- `scripts/local_z1_controller.sh`：新 Z1 控制器以 `nohup setsid` 脱离
  前台终端进程组，保留自动启动/复用，不需要第二个 Z1 控制终端。
  不复用已经退出的 zombie；不随意结束外部控制器。
- `src/rl_sar/src/rl_real_b2_z1_no_gun.cpp`：析构先停止所有原控制循环和
  SDK 通信线程，然后用退出专用、有限次数的 PASSIVE 报文交换代替
  无限等待的 `setFsm`。
- `src/rl_sar/library/core/rl_sdk/va15_z1_exit.hpp`：仅供退出调用，最多
  10 次、每次接收超时 20 ms，250 ms 级时间预算；报文运动字段全零。
  不伪造 `recvState`，不创建会访问已销毁对象的 detached 超时线程。
  没有收到 PASSIVE 回复时明确报告 `reply=0`，仍结束进程。

协议没有请求序号；收到的 PASSIVE 回复不等于已经证明机械臂物理安全。
250 ms 预算只针对此报文交换，不是对所有操作系统/SDK 线程 join 的
硬实时保证。诊断构造退出不新增 PASSIVE 报文。正常初始化、LOWCMD
切换和实时控制路径未改；未改 SDK 库文件。

板载备份：
`deployment_backups/z1_exit_20261020_onW0ZD/`，含旧主程序源码、启动
脚本及旧二进制。若要回退，只能在实际趴稳、控制已退出后操作。

## 验证

- 本机完整原有+新增 CTest：12/12 通过。
- PC4 独立退出回归：3/3 通过，目录
  `diagnostics/z1_exit_20261020_AHfGty/`。
- 两端测试包括 5 组假传输状态（正确回复、失联、错误状态、错误头、
  截断包）、3 组真实 SDK UDP 的临时 `127.0.0.1` 回环测试，以及
  15 组启动器假进程测试（包含 SIGINT 和独立 session）。不使用真实
  8071/8072，不导入 B2 SDK 或启动实际 `z1_ctrl`。
- 板载 `run_b2_z1_real.sh --offline-check` 通过，golden 最大差
  `1.14441e-05`；没有初始化 DDS/UDP/机器人。
- VA15 权重、配置及 FSM 的 SHA256 保持不变，动态运动仍锁定。

当前板载固定的三个校验值：

```text
8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098  VA15/policy.pt
8faa51628b3704ce3e7e3a384f1c0eb8bb59b7dcf5a0069992069efed8aab8d1  VA15/config.yaml
435b18eddc2381c53ce7e31ab93386eaa86393e5c9eeb37841a942649e131cf3  fsm_b2_z1_no_gun.hpp
```

复现无硬件回归：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j1
ctest --test-dir build --output-on-failure
```

## 操作者下一步

保持 B2 趴稳、Z1 安全放置，在满足启动器现场安全要求后启动原命令，
输入 RUN，**不按 A/Y/R1+Y、不推动摇杆**，先直接 Ctrl+C 验证能够退出。
最终应出现 `[Shutdown] Z1 PASSIVE reply=...` 和
`B2+Z1 real deployment stopped` 并回到 shell。这不是行走验收。
以后的站立实验仍需先 B 趴稳、确认 Passive 后才 Ctrl+C；不能把本次
退出修复当成站立时可直接失去力矩或断电的保证。
