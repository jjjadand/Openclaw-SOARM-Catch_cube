# Openclaw-SOARM-Catch_cube

使用 Openclaw 控制 SO-ARM101 机械臂抓取 cube 的 Skills 工程。

## 项目结构

```
├── setup.sh                    # 一键安装脚本
├── soarm-control/              # 核心：机械臂控制 (必装)
│   ├── scripts/
│   │   ├── soarm_pick_and_place.py  # 抓取 cube 完整序列 (主功能)
│   │   ├── pinocchio_xyz_drive.py   # XYZ 坐标控制 (IK)
│   │   ├── soarm_set_joints.py      # 关节角度控制
│   │   ├── soarm_status.py          # 读取机械臂状态
│   │   ├── soarm_get_position.py    # 读取末端 XYZ 坐标
│   │   ├── soarm_enable.py          # 使能机械臂
│   │   ├── soarm_disable.py         # 失能机械臂
│   │   └── soarm_move.sh            # 移动命令封装
│   └── references/
│       ├── openclaw_soarm.json      # 校准文件
│       └── so101_new_calib.urdf     # URDF 模型
├── lerobot-env-setup/          # Jetson 平台专用环境配置
│   └── scripts/
│       └── install_lerobot.sh       # LeRobot 分阶段安装
└── pinocchio-install/          # Pinocchio 逆运动学库安装
    └── scripts/
        └── install_pinocchio.sh     # 支持 conda/pip/源码编译
```

## 硬件要求

- SO-ARM101 机械臂 (Feetech 舵机)
- USB 串口连接 (默认 `/dev/ttyACM0`)
- NVIDIA Jetson (JetPack 6.0+) 或 x86_64 Linux 桌面

## 快速安装

```bash
git clone <repo-url> ~/Openclaw-SOARM-Catch_cube
cd ~/Openclaw-SOARM-Catch_cube

# Jetson 平台 (默认)
bash setup.sh

# x86_64 桌面
bash setup.sh --platform desktop

# 自定义环境名
bash setup.sh --env myenv

# 跳过已安装的组件
bash setup.sh --skip-lerobot --skip-pinocchio
```

安装脚本会自动完成：
1. LeRobot 环境配置 (Jetson 使用 GPU wheel，桌面使用 pip)
2. Pinocchio 逆运动学库安装
3. 校准文件部署 + 串口权限配置

## 使用方法

所有脚本在 conda 环境中运行。Jetson 上需要预加载 `libstdc++`：

```bash
# Jetson 环境变量 (建议写入 ~/.bashrc)
export LD_PRELOAD=$HOME/miniconda3/envs/lerobot/lib/libstdc++.so.6
conda activate lerobot
```

### 抓取 cube (主功能)

执行完整的抓取-搬运-放置-复位序列：

```bash
python soarm-control/scripts/soarm_pick_and_place.py --speed 20
```

动作序列共 6 步：
1. 移动到准备位置，张开夹爪
2. 靠近物块，收紧夹爪抓取
3. 抬起物块
4. 移动到放置位置
5. 放下物块，张开夹爪
6. 复位到 home 位置

参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--speed` | 移动速度 (度/秒) | 20 |
| `--pause` | 每步之间停顿时间 (秒) | 0.5 |
| `--port` | 串口设备 | /dev/ttyACM0 |
| `--robot-id` | 机器人 ID | openclaw_soarm |

### XYZ 坐标控制

通过逆运动学计算，直接指定末端目标位置：

```bash
# 移动到指定坐标
python soarm-control/scripts/pinocchio_xyz_drive.py --x 0.2 --y 0.0 --z 0.1

# 慢速移动 + 打开夹爪
python soarm-control/scripts/pinocchio_xyz_drive.py \
    --x 0.15 --y 0.05 --z 0.08 \
    --max-joint-speed-deg 10 \
    --open-gripper --gripper-open-value 80

# 连续移动 (保持连接，减少延迟)
python soarm-control/scripts/pinocchio_xyz_drive.py --x 0.2 --y 0.0 --z 0.15 --keep-connected
python soarm-control/scripts/pinocchio_xyz_drive.py --x 0.15 --y 0.05 --z 0.08 --keep-connected
```

坐标系：X 前后(前+)，Y 左右(左+)，Z 上下(上+)

### 关节角度控制

直接设置各关节角度，适用于预设位置：

```bash
# home 位置
python soarm-control/scripts/soarm_set_joints.py \
    --shoulder-pan 1.626 --shoulder-lift -104.088 --elbow-flex 97.495 \
    --wrist-flex 77.714 --wrist-roll -95.077

# 查看桌面位置
python soarm-control/scripts/soarm_set_joints.py \
    --shoulder-pan 1.626 --shoulder-lift -42.110 --elbow-flex 32.088 \
    --wrist-flex 78.242 --wrist-roll -95.077
```

### 状态与辅助

```bash
# 查看机械臂状态
python soarm-control/scripts/soarm_status.py

# 读取当前末端 XYZ 坐标
python soarm-control/scripts/soarm_get_position.py

# 失能机械臂 (释放舵机)
python soarm-control/scripts/soarm_disable.py
```

## Skills 说明

| Skill | 说明 | 必装 |
|-------|------|------|
| soarm-control | 机械臂控制核心，包含抓取序列、IK 求解、关节控制、状态读取 | ✅ |
| lerobot-env-setup | Jetson 平台 LeRobot 环境配置，解决 torch GPU wheel 兼容性问题 | Jetson 必装 |
| pinocchio-install | Pinocchio 刚体动力学库安装，支持 conda/pip/源码三种方式 | ✅ |

## 故障排查

| 问题 | 解决方案 |
|------|----------|
| `/dev/ttyACM0` 权限不足 | `sudo usermod -aG dialout $USER` 后重新登录 |
| 串口被 brltty 占用 | `sudo apt remove brltty` |
| Jetson 上 `GLIBCXX` 版本错误 | 设置 `LD_PRELOAD=$HOME/miniconda3/envs/lerobot/lib/libstdc++.so.6` |
| Jetson 上 `torch.cuda.is_available()` 为 False | 删除 `~/wheels/*.whl` 后重新运行 `bash setup.sh` |
| `import pinocchio` 失败 (aarch64) | 查看 `pinocchio-install/references/build_troubleshooting.md` |
| IK 求解失败 | 目标坐标超出工作空间，尝试更近的坐标 |
| 抓取序列夹不住物块 | 调整 `soarm_pick_and_place.py` 中 `PICK_AND_PLACE_SEQUENCE` 的夹爪值 (当前为 12) |
