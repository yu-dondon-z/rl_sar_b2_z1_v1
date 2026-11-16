# 目录维护工具

[archive_legacy_20261115.py](archive_legacy_20261115.py) 是2026-11-15两仓整理的一次性机械迁移工具。它不启动训练、仿真、视觉或硬件，不清理日志／检查点。整理已经执行，不要重复`--apply`。

默认仅显示41条移动计划；`--apply`先写原文备份与SHA256清单，再归档17篇文档、24个shell并建立兼容链接，拒绝覆盖已有归档；`--verify`只校验并更新维护清单。

```bash
cd /home/dong/rl_sar_b2_z1
/home/dong/anaconda3/bin/python -B scripts/maintenance/archive_legacy_20261115.py --verify
```

维护记录与恢复注意事项见 [docs/maintenance/README.md](../../docs/maintenance/README.md)。历史维护清单不追随以后的正常代码改动；以后源码发生变化时应与对应时间点比较，不要为满足旧哈希而回退当前源码。

`--verify`检查299个固定文件、文档链接、24个兼容链接、shell语法／权限，并与原文快照比较是否只做了预定机械改动。IDE的`browse.vc.db*`自动索引文件不作固定源码校验，原始哈希仍保留在本次清单，工具不会删除或回退这些缓存。
