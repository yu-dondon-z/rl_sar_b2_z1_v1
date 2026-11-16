# 抓包无法复用 sudo 授权：终端会话修正

用户在同一终端先执行 sudo -v，随后启动抓包仍收到 `sudo: a password is required`。
故障会话：`runs/launcher_20261106_163705_705399`。该次没有新启控制器或主程序。

原因是 LoggedProcess 原来一律使用 start_new_session=True。底层 setsid 创建新会话并脱离
原控制终端，而 sudo 默认认证缓存按终端会话区分；失去终端后会改按父进程匹配。
因此用户刚刚在交互终端获得的授权，不能被该脱离终端的抓包进程复用。
此前模拟进程验证未覆盖真实控制终端身份，遗漏了这个问题。

修正只针对抓包子进程：保留启动终端及 session ID，子进程进入独立进程组，
再 exec 原 sudo -n / tcpdump 命令。没有把主程序提权，也没有修改 sudoers 或持久系统权限。
独立 exec 包装兼容板载 Python 3.10，避免在多线程父进程中使用 preexec_fn。
主程序和控制器仍各自创建独立会话，原日志、过滤器和退出顺序保留。

新增回归在真实控制 PTY 下启动同一 LoggedProcess 路径，验证抓包进程能够打开 /dev/tty、
session ID 与启动终端相同，同时进程组单独隔离。测试不运行 sudo 或抓包，不初始化机器人 IO。
原有伪进程测试继续覆盖抓包启动失败不新启控制器、正常退出先主程序后控制器再抓包。

更新安装后无需编译。在板载的同一个交互终端执行：

```bash
cd /home/unitree/rl_sar_b2_z1_VD2
source .venv/bin/activate
sudo -v
bash start_real.sh --interface eth0 --capture-z1-network
```

sudo -v 若要求密码，在终端输入板载账户密码即可。不要用 sudo 启动 start_real.sh。
开发机只安装和离线验证，不代替用户启动现场控制。
真实 sudo 缓存复用与 tcpdump 启动仍需用户在原交互终端确认，不能把 PTY 测试称作真实抓包验收。
