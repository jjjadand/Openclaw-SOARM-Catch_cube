# soarm-catch-cube vs soarm-control_v2 对比

## 架构

| | soarm-catch-cube | soarm-control_v2 |
|---|---|---|
| 控制方式 | 独立 Python 脚本，直接操作机械臂 | Flask REST API 服务，通过 HTTP 接口控制 |
| 启动方式 | 直接 `python xxx.py` | 启动服务后通过 curl / HTTP 请求调用 |
| 适用场景 | 手动调试、单次任务脚本 | 被外部程序或 AI agent 调用 |

## API 接口（soarm-control_v2）

- `GET  /joints` — 读取当前关节角度和末端 XYZ
- `POST /move/joints` — 按关节角度移动，angles 数组顺序固定：shoulder_pan / shoulder_lift / elbow_flex / wrist_flex / wrist_roll / gripper
- `POST /move/xyz` — 按末端坐标移动，内部用 pinocchio IK 求解
- `GET  /healthz` — 健康检查
- `POST /disconnect` — 断开连接

## 文件结构差异

| soarm-catch-cube | soarm-control_v2 | 说明 |
|---|---|---|
| scripts/soarm_status.py | — | 状态读取改为 GET /joints |
| scripts/soarm_enable.py | — | 已移除 |
| scripts/soarm_disable.py | — | 已移除 |
| scripts/pinocchio_xyz_drive.py | scripts/move_soarm_to_xyz_pinocchio.py | IK 求解封装进 API |
| scripts/soarm_set_joints.py | scripts/control_soarm_joints.py | 关节控制 |
| scripts/soarm_get_position.py | scripts/read_soarm_joints.py | 读取关节 |
| scripts/soarm_pick_and_place.py | — | v2 没有预设抓取任务脚本 |
| references/openclaw_soarm.json | — | v2 不含标定 json |
| — | scripts/soarm_api.py | Flask API 服务主体 |
| — | scripts/start_server.sh | 启动服务脚本 |
| — | agents/openai.yaml | AI agent 接入配置 |

## 速度参数变化

- soarm-catch-cube：速度单位为 deg/s，默认 20
- soarm-control_v2：速度为 0~1 的比例参数，默认 0.2

## 备注

- soarm-control_v2 的 references 只保留了 URDF，去掉了旧工程的 openclaw_soarm.json
- v2 设计目标是作为服务常驻，适合 AI agent 通过 HTTP 调用
- 抓取任务序列（pick and place）需要在 v2 基础上另行实现
