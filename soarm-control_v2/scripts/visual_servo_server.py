"""
视觉伺服 HTTP 服务器
提供 API 接口触发抓取任务

启动服务器:
    python visual_servo_server.py

触发抓取:
    curl -X POST http://localhost:8002/pick
"""

import time
import math
import requests
import numpy as np
from flask import Flask, jsonify
from threading import Thread, Lock
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

# ── 配置 ──────────────────────────────────────────────
ROBOT_ID  = "openclaw_soarm"
SPEED_DEG = 20
HZ        = 30

IMG_W, IMG_H   = 640, 480
IMG_CX, IMG_CY = IMG_W // 2, IMG_H // 2

DETECT_URL      = "http://localhost:5000/coordinates"
DETECT_TIMEOUT  = 1.0
MAX_SERVO_STEPS = 200
LOCK_MAX_JUMP   = 150

SERVO_TARGET_X = 422
SERVO_TARGET_Y = 217

# 俯拍位置
OVERHEAD_JOINTS = {
    "shoulder_pan":   0.0,
    "shoulder_lift": -50.0,
    "elbow_flex":     20.0,
    "wrist_flex":     90.0,
    "wrist_roll":    -97.0,
}

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# 视觉伺服参数
KP_PAN    = 0.015
KP_LIFT   = 0.01
DEAD_ZONE = 4.0
MAX_STEP  = 1.0

# 复位姿态
HOME_JOINTS = {
    "shoulder_pan":    0.0,
    "shoulder_lift": -104.0,
    "elbow_flex":     95.0,
    "wrist_flex":     65.0,
    "wrist_roll":    -95.0,
}

# ── 全局状态 ──────────────────────────────────────────
app = Flask(__name__)
task_lock = Lock()
task_running = False
last_result = None

# ── 检测接口 ──────────────────────────────────────────
def get_detection(last_pos=None):
    try:
        resp = requests.get(DETECT_URL, timeout=DETECT_TIMEOUT)
        objs = resp.json().get("objects", [])
        if not objs:
            return None
        if last_pos is None:
            best = max(objs, key=lambda o: o["confidence"])
        else:
            lx, ly = last_pos
            best = min(objs, key=lambda o: (o["center"]["x"] - lx)**2 + (o["center"]["y"] - ly)**2)
            dist = math.sqrt((best["center"]["x"] - lx)**2 + (best["center"]["y"] - ly)**2)
            if dist > LOCK_MAX_JUMP:
                return None
        return best["center"]["x"], best["center"]["y"]
    except Exception:
        return None

# ── 工具函数 ──────────────────────────────────────────
def find_serial_port():
    """自动检测可用的串口设备 (ttyACM0-4)"""
    import os
    for i in range(5):
        port = f"/dev/ttyACM{i}"
        if os.path.exists(port):
            print(f"✓ 找到串口设备: {port}")
            return port
    raise RuntimeError("未找到可用的串口设备 (/dev/ttyACM0-4)")

def connect_robot():
    port = find_serial_port()
    cfg = SO101FollowerConfig(
        port=port, id=ROBOT_ID, cameras={},
        use_degrees=True, disable_torque_on_disconnect=True,
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)
    return robot

def get_joints(robot):
    obs = robot.get_observation()
    return {n: float(obs[f"{n}.pos"]) for n in JOINT_NAMES}, float(obs.get("gripper.pos", 0))

def send_joints(robot, joints, gripper):
    action = {f"{n}.pos": joints[n] for n in JOINT_NAMES}
    action["gripper.pos"] = gripper
    robot.send_action(action)

def move_to(robot, target, gripper=None, speed=SPEED_DEG):
    start, cur_gripper = get_joints(robot)
    g_target = gripper if gripper is not None else cur_gripper
    max_delta = max(abs(target[n] - start[n]) for n in JOINT_NAMES)
    duration = max(1.0, max_delta / speed)
    steps = max(1, int(duration * HZ))
    print(f"  移动中... 预计 {duration:.1f}s")
    for i in range(1, steps + 1):
        t = i / steps
        j = {n: start[n] + (target[n] - start[n]) * t for n in JOINT_NAMES}
        send_joints(robot, j, cur_gripper + (g_target - cur_gripper) * t)
        time.sleep(1.0 / HZ)
    time.sleep(0.3)

def move_gripper(robot, joints, g_start, g_target, duration=0.6):
    steps = max(1, int(duration * HZ))
    for i in range(1, steps + 1):
        t = i / steps
        send_joints(robot, joints, g_start + (g_target - g_start) * t)
        time.sleep(1.0 / HZ)
    time.sleep(0.3)

# ── 抓取任务 ──────────────────────────────────────────
def run_pick_task():
    global task_running, last_result
    
    try:
        print("=" * 55)
        print(f"视觉伺服  图像分辨率: {IMG_W}x{IMG_H}")
        print("=" * 55)

        robot = connect_robot()
        print("✓ 机械臂已连接\n")

        try:
            # 1. 移动到俯拍位置
            print("[1] 移动到俯拍位置...")
            move_to(robot, OVERHEAD_JOINTS)
            print("✓ 到达俯拍位置\n")

            # 2. 稳定1秒
            print("[2] 保持俯拍位置 1s...")
            cur_joints, gripper = get_joints(robot)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.0:
                send_joints(robot, cur_joints, gripper)
                time.sleep(1.0 / HZ)
            print("✓ 保持完成\n")

            # 3. 动态预调整 wrist_flex
            print("[3] 动态预调整 wrist_flex...")
            cur_joints, gripper = get_joints(robot)
            
            initial_det = get_detection()
            if initial_det is None:
                print("  [警告] 未检测到目标，使用默认预调整 -10°")
                wrist_adjust = -10.0
            else:
                cx, cy = initial_det
                dist_from_center = math.sqrt((cx - IMG_CX)**2 + (cy - IMG_CY)**2)
                print(f"  初始检测: ({cx}, {cy}), 距中心: {dist_from_center:.1f}px")
                
                if dist_from_center > 250:
                    wrist_adjust = -28.0
                elif dist_from_center > 200:
                    wrist_adjust = -24.0
                elif dist_from_center > 150:
                    wrist_adjust = -20.0
                elif dist_from_center > 100:
                    wrist_adjust = -15.0
                elif dist_from_center > 70:
                    wrist_adjust = -12.0
                elif dist_from_center > 40:
                    wrist_adjust = -9.0
                else:
                    wrist_adjust = -7.0
                
                print(f"  预调整角度: wrist_flex {wrist_adjust:+.1f}°")
            
            pre_joints = dict(cur_joints)
            pre_joints["wrist_flex"] += wrist_adjust
            move_to(robot, pre_joints)
            print("✓ 预调整完成\n")

            # 4. 视觉伺服
            print("[4] 开始视觉伺服...")
            print(f"    Kp_pan={KP_PAN} Kp_lift={KP_LIFT} 死区={DEAD_ZONE}px 锁定跳变={LOCK_MAX_JUMP}px\n")
            print(f"{'Step':>4}  {'cx':>5}  {'cy':>5}  {'dx':>7}  {'dy':>7}  {'pan':>8}  {'lift':>8}  {'err':>8}")
            print("-" * 65)

            cur_joints, gripper = get_joints(robot)
            step = 0
            last_pos = None

            while step < MAX_SERVO_STEPS:
                det = get_detection(last_pos)
                if det is None:
                    print("  [警告] 未检测到目标，等待...")
                    time.sleep(1.0 / HZ)
                    continue

                cx, cy = det
                if last_pos is None:
                    print(f"  锁定目标: ({cx}, {cy})")
                last_pos = (cx, cy)
                dx = cx - SERVO_TARGET_X
                dy = cy - SERVO_TARGET_Y
                err = math.sqrt(dx**2 + dy**2)
                step += 1
                print(f"{step:>4}  {cx:>5}  {cy:>5}  {dx:>7.1f}  {dy:>7.1f}  {cur_joints['shoulder_pan']:>8.3f}  {cur_joints['shoulder_lift']:>8.3f}  {err:>8.2f}")

                if err < DEAD_ZONE:
                    print(f"\n✓ 误差 {err:.2f}px < 死区 {DEAD_ZONE}px，对准目标！")
                    break

                delta_pan  = float(np.clip(dx * KP_PAN,  -MAX_STEP, MAX_STEP))
                delta_lift = float(np.clip(dy * KP_LIFT, -MAX_STEP, MAX_STEP))
                cur_joints["shoulder_pan"]  += delta_pan
                cur_joints["shoulder_lift"] += delta_lift
                send_joints(robot, cur_joints, gripper)
                time.sleep(1.0 / HZ)
            else:
                print(f"\n[警告] 达到最大迭代次数 {MAX_SERVO_STEPS}，强制继续")

            # 5. 张开夹爪
            print("\n[5] 向下抓取...")
            move_gripper(robot, cur_joints, gripper, 50, duration=0.5)
            gripper = 50

            # 6. 下压
            pick_joints = dict(cur_joints)
            pick_joints["shoulder_lift"] = 20.0
            pick_joints["elbow_flex"] = -5.0
            print("  下压 (shoulder_lift=20° / elbow_flex=-5°)")
            move_to(robot, pick_joints, gripper=gripper, speed=15)

            # 7. 合夹爪
            print("  合夹爪 -> 5")
            move_gripper(robot, pick_joints, 50, 5, duration=0.8)
            print("✓ 抓取完成")

            # 8. 抬起
            print("\n[6] 抬起...")
            lift_joints = dict(pick_joints)
            lift_joints["shoulder_lift"] -= 20.0
            lift_joints["elbow_flex"]    -= 35.0
            lift_joints["wrist_flex"]    += 45.0
            move_to(robot, lift_joints, gripper=5, speed=15)
            print("✓ 抬起完成")

            # 9. 旋转放置
            print("\n[7] 旋转 shoulder_pan -> 60°...")
            place_joints = dict(lift_joints)
            place_joints["shoulder_pan"] = 60.0
            move_to(robot, place_joints, gripper=5, speed=SPEED_DEG)
            print("✓ 旋转完成")

            # 10. 松开
            print("\n[8] 松开夹爪 -> 50...")
            move_gripper(robot, place_joints, 5, 50, duration=0.6)
            print("✓ 放下完成")

            # 11. 复位
            print("\n[9] 复位...")
            move_to(robot, HOME_JOINTS, gripper=10, speed=SPEED_DEG)
            print("✓ 复位完成")

            last_result = {"ok": True, "message": "抓取任务完成"}

        finally:
            print("\n断开连接...")
            robot.disconnect()
            print("✓ 完成")

    except Exception as e:
        print(f"\n[错误] {e}")
        last_result = {"ok": False, "error": str(e)}
    finally:
        with task_lock:
            task_running = False

# ── HTTP API ──────────────────────────────────────────
app = Flask(__name__)

@app.route('/pick', methods=['POST'])
def trigger_pick():
    global task_running
    
    with task_lock:
        if task_running:
            return jsonify({"ok": False, "message": "任务正在运行中"}), 409
        task_running = True
    
    # 在后台线程运行任务
    thread = Thread(target=run_pick_task, daemon=True)
    thread.start()
    
    return jsonify({"ok": True, "message": "抓取任务已启动"})

@app.route('/status', methods=['GET'])
def get_status():
    with task_lock:
        running = task_running
    
    return jsonify({
        "running": running,
        "last_result": last_result
    })

@app.route('/health', methods=['GET'])
def health():
    return jsonify({"ok": True, "service": "visual_servo_server"})

# ── 启动服务器 ────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 55)
    print("视觉伺服服务器启动")
    print("=" * 55)
    print("API 端点:")
    print("  POST /pick   - 触发抓取任务")
    print("  GET  /status - 查询任务状态")
    print("  GET  /health - 健康检查")
    print("\n使用示例:")
    print("  curl -X POST http://localhost:8002/pick")
    print("  curl http://localhost:8002/status")
    print("=" * 55)
    print()
    
    app.run(host='0.0.0.0', port=8002, debug=False)
