# 16:37 抓包 sudo 授权失败修正

用户在原交互终端先 sudo -v，抓包进程仍提示需要密码。
会话 `runs/launcher_20261106_163705_705399/capture.log` 已核对；该次未新启控制器或主程序。

原因：LoggedProcess 一律 start_new_session=True，抓包子进程 setsid 后失去原控制终端。
sudo 默认按终端会话缓存授权，没有控制终端时按父进程匹配，无法复用原交互终端授权。
修正仅让抓包保留原 session/控制终端，通过独立进程组隔离信号；
使用兼容 Python 3.10 的 exec 包装，不在多线程父进程中使用 preexec_fn。
主程序和控制器保持原启动方式；没有给控制程序提权、修改 sudoers 或系统永久权限。

本地启动器 12/12 用例通过，新增真实控制 PTY 回归验证相同 session ID、
可打开 /dev/tty、独立进程组。最终补丁在板载隔离副本完整离线 94/94 通过。
测试没有执行 sudo、tcpdump 或机器人 IO；实际授权复用仍需用户原交互终端确认。

16:43 已安装板载正式目录 `/home/unitree/rl_sar_b2_z1_VD2`。
备份 `patch_backups/capture_session_20261106_164347_085980`。
906 个清单文件哈希全部一致，run.py、control.py、native/io.cpp、策略和相机配置均与安装前相同。
安装后未运行控制器、主程序或抓包。没有启动机器人运动。

补丁：`/home/dong/rl_sar_b2_z1_VD2_capture_session_20261106.zip`，板载 /home/unitree 下同名。
SHA256：`5baa84618b630ffbf5444483ca226ec363c18d8b17bf20fdd8fb4fb33bf8e752`。
验证：`outputs/VD2_CAPTURE_SESSION_20261106_VHFtVg/board_offline_check.log`。
隔离副本 `/home/unitree/vd2_capture_session_check_20261106_mgADy1`。

现场继续在同一个交互终端运行原命令，无需编译：

```bash
cd /home/unitree/rl_sar_b2_z1_VD2
source .venv/bin/activate
sudo -v
bash start_real.sh --interface eth0 --capture-z1-network
```

此修正只解决抓包启动的授权身份，Z1 电机链路中断和动态后视仍待现场取证。
