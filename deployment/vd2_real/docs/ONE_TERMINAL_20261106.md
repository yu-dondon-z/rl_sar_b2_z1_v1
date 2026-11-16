# 单终端启动 Z1 与 VD2 主程序

新增统一入口 `start_real.sh`。无需重新编译 C++，无需修改现有策略、相机外参或控制增益，也不需要另外打开一个终端启动 Z1。

## 安装

在开发电脑（`dong@dong-MP`）执行传输：

```bash
scp /home/dong/rl_sar_b2_z1_VD2_one_terminal_20261106.zip unitree@192.168.123.164:/home/unitree/
```

在板载（`unitree@unitree-b2-pc4`）执行，先正常退出已有 `run.py`/预检程序：

```bash
cd /home/unitree
unzip -n rl_sar_b2_z1_VD2_one_terminal_20261106.zip
cd /home/unitree/rl_sar_b2_z1_VD2
source .venv/bin/activate
python ../vd2_one_terminal_20261106/install.py --target .
python run.py --offline-check
```

安装器核对新文件 SHA256，保留现有配置、日志和代码修改；清单备份至 `patch_backups/one_terminal_<时间>/`，新文件被列入 `MANIFEST.json`。重复安装会直接提示已安装。此前相机及部署准备补丁继续有效。

## 一个终端启动

现场机器人保持静止趴姿，在板载部署目录执行：

```bash
bash start_real.sh --interface eth0
```

脚本自动使用本目录 `.venv/bin/python`，切换到 `/home/unitree/z1_controller/build` 后启动 `./z1_ctrl`，等其 UDP 8071 绑定，再启动 `run.py --run --interface eth0`。绑定端口只代表服务入口已建立，真实 B2/Z1 状态新鲜度仍由主程序检查，不会用端口代替反馈。

已经检测到一个 `z1_ctrl` 时会复用它，退出时保留它；多个控制器、未知程序占用 8071、SDK 客户端占用 8072 时会明确报错，不自动杀掉现有程序。若控制器路径不同：

```bash
bash start_real.sh --interface eth0 --z1-dir /实际目录/z1_controller/build
```

也可以在同一入口做状态检查（读取 B2，向 Z1 请求 PASSIVE，不接管 B2）：

```bash
bash start_real.sh --preflight --interface eth0 --seconds 10
```

## 操作和退出

主程序仍等待新按下手柄 A 才接管 B2：A 站起，Y 准备机械臂，R1+Y 开始/停止跟踪，X 停止，B 完成停止、收臂和趴下，L1+X 请求阻尼/PASSIVE。脚本不自动按键，不自动恢复中断的行走。

正常用 B 趴下完成后，再 Ctrl+C 退出。Ctrl+C 本身仅请求主程序按原逻辑阻尼/PASSIVE 退出，不是完整趴下动作。随后脚本关闭本次自行启动的 Z1 控制器；复用的已有控制器继续运行。主程序启动失败或异常退出时也会收尾本次启动的控制器。若控制器运行期间退出，脚本会通知主程序退出。

同一终端输出带 `[z1]` / `[main]` 前缀。新启动控制器和主程序的完整输出保存在 `runs/launcher_<时间>/z1.log`、`main.log`，启动进程及退出状态在 `launcher.json`；主程序原有相机、控制日志仍在其独立 `runs/<时间>/` 中。复用控制器的输出保留在其原终端/服务日志中。

## 验证范围

新增 6 项本地进程测试，使用临时伪控制器、伪主程序与本机回环 UDP，不调用真实 SDK 或机器人：自动启动且使用正确工作目录；主程序先退出再关闭自启控制器；复用外部控制器不关闭；8072 被占用时拒绝启动；控制器启动失败不运行主程序；主程序错误码保留；Ctrl+C 依次关闭主程序及自启控制器。实际板载执行与机器人响应仍需现场验证。
