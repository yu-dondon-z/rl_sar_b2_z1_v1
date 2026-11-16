# 第二轮 lost 原始字段只读核查（2026-10-21）

操作者确认已安全趴卧并退出；SSH 确认无主控制进程。本工具只订阅
rt/lowstate，不发布 B2/Z1 命令、不释放官方服务、不安装新控制代码。
固定输出 control_authorized=false、hardware_qualified=false。

## 新增核查与结果

板载输出：`outputs/lost_audit_v2_K7LpWR/raw.csv`、`raw.json`；
本地副本：本目录 `outputs/prone_K7LpWR/`。

- 约 30 秒、2951 帧：右前大腿 motor[1].lost 始终为 5，其余 11 关节为 0。
- 无 tick 重复/倒退/>50 ms 间隔；无源年龄 >100 ms，末帧源年龄约 0.95 ms。
- 新增逐通道变化检查：该关节 q 有 6 个不同值、dq 有 2951 个、tau 有 26 个。
  这排除了观察窗口内这些数值整组恒定，但**不是逐电机新鲜度认证**。
- q_raw、dq_raw、ddq_raw 全为零，不能作为替代的新鲜度证据。
- 固件原始 version=[0,0]、bit_flag=5、motor[1].reserve=[0,535]；只保存，
  未自行定义这些字段含义。crc 也只保存，不声称已通过 CRC 校验。
- 2951 帧基本电机检查均因 lost=5 拒绝；没有清零原值或改为可用。

只读工具 6/6 单测通过。可以离线复验，不会导入机器人 SDK：

```bash
cd /home/dong/rl_sar_b2_z1/patches/va15_lost_audit_v2_20261021
python3 -m unittest discover -s . -p 'test_*.py' -v
```

## 再次查阅官方资料

[宇树官方 MotorState.msg](https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/MotorState.msg)
仅定义 uint32 lost，未给当前 B2 固件累计/清零规则。官方 Basic_services
动态页面本次仍未取得有效字段说明；G1 失联判断不作为 B2 字段语义证据。

**仍不能认定 lost=5 只是历史计数或当前通信已合格。** 需要厂商确认
当前 B2 固件语义和独立逐电机时效/故障证据，再设计新候选。生产
ValidateSample、lost 拒绝规则、足端阈值、validated 位和行走限制均未改。
不通过重启来消除该数字，也不再反复要求操作者起立试走。

同轮另有无工具并行趴下候选，见 `../va15_stop_parallel_20261021/`；
那是独立的 GetDown 逻辑修改，不能当作 lost 问题已解决。

## 操作者确认趴下正常后的复核

新增只读记录 `outputs/followup_UjAM3a/`：约 9.990 秒、985 帧，姿态
标签 unknown；motor[1] lost 仍全程 5，其他关节 0，无 tick 异常或
源年龄 >100 ms，正常退出。没有改生产文件。生产起步验证位与独立
估计拒绝规则是两条不同的链，详见 `../../docs/B2_MOTION_READINESS_20261021.md`。
