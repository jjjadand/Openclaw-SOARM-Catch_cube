---
name: soarm-control
description: >
  控制 SOARM101 机械臂。当用户提到 catch/grab/pick/place/cube/block/抓取/放下/物块/方块/盒子 时，
  直接执行: LD_PRELOAD=$HOME/miniconda3/envs/sk-test66/lib/libstdc++.so.6 conda run -n sk-test66 python ~/.openclaw/skills/soarm-control/scripts/soarm_pick_and_place.py --speed 20
---

# SOARM Control Skill

所有脚本都在 conda 环境 `sk-test66` 中运行。不需要安装任何东西。

## 抓取/放置任务

当用户要求抓取、搬运、放置物块时，直接执行：

```bash
LD_PRELOAD=$HOME/miniconda3/envs/sk-test66/lib/libstdc++.so.6 conda run -n sk-test66 python ~/.openclaw/skills/soarm-control/scripts/soarm_pick_and_place.py --speed 20
```

## 其他功能

```bash
# XYZ 坐标控制
LD_PRELOAD=$HOME/miniconda3/envs/sk-test66/lib/libstdc++.so.6 conda run -n sk-test66 python ~/.openclaw/skills/soarm-control/scripts/pinocchio_xyz_drive.py --x 0.2 --y 0.0 --z 0.1

# 关节角度控制
LD_PRELOAD=$HOME/miniconda3/envs/sk-test66/lib/libstdc++.so.6 conda run -n sk-test66 python ~/.openclaw/skills/soarm-control/scripts/soarm_set_joints.py \
    --shoulder-pan 1.6 --shoulder-lift -104.1 --elbow-flex 97.5 --wrist-flex 77.7 --wrist-roll -95.1

# 查看状态
LD_PRELOAD=$HOME/miniconda3/envs/sk-test66/lib/libstdc++.so.6 conda run -n sk-test66 python ~/.openclaw/skills/soarm-control/scripts/soarm_status.py

# 失能机械臂
LD_PRELOAD=$HOME/miniconda3/envs/sk-test66/lib/libstdc++.so.6 conda run -n sk-test66 python ~/.openclaw/skills/soarm-control/scripts/soarm_disable.py

# home 位置
LD_PRELOAD=$HOME/miniconda3/envs/sk-test66/lib/libstdc++.so.6 conda run -n sk-test66 python ~/.openclaw/skills/soarm-control/scripts/soarm_set_joints.py \
    --shoulder-pan 1.626 --shoulder-lift -104.088 --elbow-flex 97.495 --wrist-flex 77.714 --wrist-roll -95.077

# look_desktop 位置
LD_PRELOAD=$HOME/miniconda3/envs/sk-test66/lib/libstdc++.so.6 conda run -n sk-test66 python ~/.openclaw/skills/soarm-control/scripts/soarm_set_joints.py \
    --shoulder-pan 1.626 --shoulder-lift -42.110 --elbow-flex 32.088 --wrist-flex 78.242 --wrist-roll -95.077
```
