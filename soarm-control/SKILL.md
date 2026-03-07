---
name: soarm-control
description: 控制 SOARM101 机械臂的运动。使用场景：(1) 控制机械臂移动到指定XYZ坐标 (2) 精确控制关节角度 (3) 查看机械臂状态
---

# SOARM Control Skill

机械臂控制 skill，用于控制 SOARM101 机械臂的运动。

> **技能目录**：`~/.openclaw/workspace/skills/soarm-control/`


## 控制方式

### 1. 使用 pinocchio_xyz_drive.py（首选，推荐）

通过 XYZ 坐标控制机械臂末端位置，内部使用逆运动学计算关节角度。

```bash
cd ~/.openclaw/workspace/skills/soarm-control
~/anaconda3/bin/conda run -n lerobot python scripts/pinocchio_xyz_drive.py --x 0.2 --y 0.0 --z 0.1
```

**参数：**
| 参数 | 说明 | 默认值 |
|------|------|--------|
| --x | X 坐标（前后，前为正） | 必填 |
| --y | Y 坐标（左右，左为正） | 必填 |
| --z | Z 坐标（上下，上为正） | 必填 |
| --max-joint-speed-deg | 移动速度 (度/秒) | 30 |
| --open-gripper | 打开夹爪 | 关闭 |
| --gripper-open-value | 夹爪开度 (0-100) | 100 |
| --keep-connected | 移动后保持连接，不断开 | 关闭 |
| --port | 串口设备 | /dev/ttyACM0 |

### 2. 使用 soarm_set_joints.py（仅在移动到预设位置时使用）

直接设置每个关节的角度，用于移动到精确的预设位置。

```bash
cd ~/.openclaw/workspace/skills/soarm-control
~/anaconda3/bin/conda run -n lerobot python scripts/soarm_set_joints.py \
    --shoulder-pan 1.6 \
    --shoulder-lift -104.1 \
    --elbow-flex 97.5 \
    --wrist-flex 77.7 \
    --wrist-roll -95.1
```

**参数：**
| 参数 | 说明 |
|------|------|
| --shoulder-pan | 肩部旋转 (度) |
| --shoulder-lift | 肩部抬起 (度) |
| --elbow-flex | 肘部弯曲 (度) |
| --wrist-flex | 腕部弯曲 (度) |
| --wrist-roll | 腕部旋转 (度) |
| --gripper | 夹爪 (0-100，可选) |
| --speed | 速度 (度/秒，默认30) |
| --keep-connected | 移动后保持连接，不断开 |

## 坐标系说明

- **X**：前后方向，前为正
- **Y**：左右方向，左为正
- **Z**：上下方向，上为正

## 预设位置

| 预设位置 | shoulder_pan | shoulder_lift | elbow_flex | wrist_flex | wrist_roll | gripper | 说明 |
|------|------|------|------|------|------|------|------|
| home | 1.626° | -104.088° | 97.495° | 77.714° | -95.077° | 0.8 | 初始位置 |
| look_desktop | 1.626° | -42.110° | 32.088° | 78.242° | -95.077° | 0.8 | 查看桌面 |

### 使用预设位置

```bash
# home - 初始位置
~/anaconda3/bin/conda run -n lerobot python scripts/soarm_set_joints.py \
    --shoulder-pan 1.626 --shoulder-lift -104.088 --elbow-flex 97.495 \
    --wrist-flex 77.714 --wrist-roll -95.077

# look_desktop - 查看桌面
~/anaconda3/bin/conda run -n lerobot python scripts/soarm_set_joints.py \
    --shoulder-pan 1.626 --shoulder-lift -42.110 --elbow-flex 32.088 \
    --wrist-flex 78.242 --wrist-roll -95.077
```


## 其他脚本

```bash
# 查看状态
python scripts/soarm_status.py

# 失能机械臂
python scripts/soarm_disable.py
```

## 依赖

### 1. Conda 环境

```bash
# 创建并激活 lerobot 环境
conda create -n lerobot python=3.10
conda activate lerobot

# 安装 lerobot
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e .

# 安装 pinocchio
conda install pinocchio -c conda-forge
```

### 2. 机械臂校准文件

首次使用需要配置校准文件：

```bash
# 创建目录
mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/so_follower

# 拷贝校准文件
cp ~/.openclaw/workspace/skills/soarm-control/references/openclaw_soarm.json \
   ~/.cache/huggingface/lerobot/calibration/robots/so_follower/openclaw_soarm.json
```

校准文件路径：`~/.cache/huggingface/lerobot/calibration/robots/so_follower/openclaw_soarm.json`

## 保持连接（重要）

连续移动机械臂时，建议使用 `--keep-connected` 参数，避免每次移动后断开连接又重新连接：
- 减少连接延迟
- 避免频繁连接导致的设备状态不稳定

```bash
# 连续移动示例
python scripts/pinocchio_xyz_drive.py --x 0.15 --y 0.0 --z 0.2 --keep-connected
python scripts/pinocchio_xyz_drive.py --x 0.25 --y 0.0 --z 0.3 --keep-connected
# 完成后如需释放，使用 soarm_disable.py
```

## 安全提示

- 首次使用建议低速 (`--max-joint-speed-deg 10`)
- 确保运动范围内无障碍物
- 紧急停止：断开机械臂电源

