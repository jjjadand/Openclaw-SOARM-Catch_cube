#!/usr/bin/env python3
"""
视觉伺服抓取服务
POST /pick  -> 执行一次完整的视觉伺服抓取任务
GET  /status -> 查询当前任务状态
"""

import math
import os
import sys 、
import threading
import time 、
from pathlib import Path

import numpy as np
import requests
from flask import Flask, jsonify

repo_root = Path.home() / "lerobot"
if repo_root.is_dir():
    sys.path.insert(0, str(repo_root))

from lerobot.common.robots.so101_follower import SO101Follower, SO101FollowerConfig

# ── 配置 ──────────────────────────────────────────────
PORT     = os.getenv("SOARM_PORT", "/dev/ttyACM0")
ROBOT_ID = os.getenv("SOARM_ID", "openclaw_soarm")
SPEED_DEG = 20
HZ        = 30

IMG_W, IMG_H   = 640, 480
IMG_CX, IMG_CY = IMG_W // 2, IMG_H // 2

DETECT_URL      = os.getenv("DETECT_URL", "http://localhost:5000/coordinates")
DETECT_TIMEOUT  = 1.0
MAX_SERVO_STEPS = 200
LOCK_MAX_JUMP   = 150   # px，跳变超过此距离丢弃该帧

# 伺服目标点（夹爪对准目标时目标在图像中的像素坐标，实测标定值）
SERVO_TARGET_X = 316
SERVO_TARGET_Y = 209

# 俯拍位置
OVERHEAD_JOINTS = {
    "shoulder_pan":  -6.505,
    "shoulder_lift":  92.791,
    "elbow_flex":    -97.890,
    "wrist_flex":     99.912,
    "wrist_roll":      1.363,
}

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# 视觉伺服参数
KP_PAN    = 0.015
KP_LIFT   = 0.01
DEAD_ZONE = 15.0
MAX_STEP  = 1.0

# 向下抓取增量
PICK_DELTA = {
    "shoulder_lift": +14.0,
    "elbow_flex":    +28.0,
    "wrist_flex":    -39.0,
}

# 复位姿态
HOME_JOINTS = {
    "shoulder_pan":   -2.637,
    "shoulder_lift":  -4.000,
    "elbow_flex":    -13.231,
    "wrist_flex":     60.791,
    "wrist_roll":    -82.681,
}

# ── 任务状态 ──────────────────────────────────────────
task_lock   = threading.Lock()
task_status = {"running": False, "result": None}

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

# ── 机械臂工具函数 ────────────────────────────────────
def connect_robot():
    cfg = SO101FollowerConfig(
        port=PORT, id=ROBOT_ID, cameras={},
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

# ── 抓取任务主逻辑 ────────────────────────────────────
def run_pick_task():
    robot = connect_robot()
    try:
        # 1. 移动到俯拍位置
        move_to(robot, OVERHEAD_JOINTS)

        # 2. 稳定1秒
        cur_joints, gripper = get_joints(robot)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            send_joints(robot, cur_joints, gripper)
            time.sleep(1.0 / HZ)

        # 3. 预调整 wrist_flex -20°
        cur_joints, gripper = get_joints(robot)
        pre_joints = dict(cur_joints)
        pre_joints["wrist_flex"] -= 20.0
        move_to(robot, pre_joints)

        # 4. 视觉伺服
        cur_joints, gripper = get_joints(robot)
        last_pos = None
        converged = False

        for _ in range(MAX_SERVO_STEPS):
            det = get_detection(last_pos)
            if det is None:
                time.sleep(1.0 / HZ)
                continue
            cx, cy = det
            last_pos = (cx, cy)
            dx = cx - SERVO_TARGET_X
            dy = cy - SERVO_TARGET_Y
            err = math.sqrt(dx**2 + dy**2)
            if err < DEAD_ZONE:
                converged = True
                break
            delta_pan  = float(np.clip(dx * KP_PAN,  -MAX_STEP, MAX_STEP))
            delta_lift = float(np.clip(dy * KP_LIFT, -MAX_STEP, MAX_STEP))
            cur_joints["shoulder_pan"]  += delta_pan
            cur_joints["shoulder_lift"] += delta_lift
            send_joints(robot, cur_joints, gripper)
            time.sleep(1.0 / HZ)

        # 5. 张开夹爪
        move_gripper(robot, cur_joints, gripper, 50, duration=0.5)
        gripper = 50

        # 6. 下压抓取
        pick_joints = dict(cur_joints)
        for k, v in PICK_DELTA.items():
            pick_joints[k] += v
        move_to(robot, pick_joints, gripper=gripper, speed=15)

        # 7. 合夹爪到5
        move_gripper(robot, pick_joints, 50, 5, duration=0.8)

        # 8. 抬起
        lift_joints = dict(pick_joints)
        lift_joints["shoulder_lift"] -= 20.0
        lift_joints["elbow_flex"]    -= 35.0
        lift_joints["wrist_flex"]    += 45.0
        move_to(robot, lift_joints, gripper=5, speed=15)

        # 9. 旋转到放置方向
        place_joints = dict(lift_joints)
        place_joints["shoulder_pan"] = 60.0
        move_to(robot, place_joints, gripper=5, speed=SPEED_DEG)

        # 10. 松开夹爪
        move_gripper(robot, place_joints, 5, 50, duration=0.6)

        # 11. 复位
        move_to(robot, HOME_JOINTS, gripper=10, speed=SPEED_DEG)

        return {"ok": True, "converged": converged}

    except Exception as e:
        return {"ok": False, "error": str(e)}
    finally:
        robot.disconnect()

# ── Flask 服务 ────────────────────────────────────────
app = Flask(__name__)

@app.post("/pick")
def pick():
    with task_lock:
        if task_status["running"]:
            return jsonify({"ok": False, "error": "task already running"}), 409
        task_status["running"] = True
        task_status["result"] = None

    def worker():
        result = run_pick_task()
        with task_lock:
            task_status["running"] = False
            task_status["result"] = result

    threading.Thread(target=worker, daemon=True).start()
    return jsonify({"ok": True, "message": "pick task started"})

@app.get("/status")
def status():
    with task_lock:
        return jsonify({
            "running": task_status["running"],
            "result":  task_status["result"],
        })

@app.errorhandler(Exception)
def handle_error(exc):
    return jsonify({"ok": False, "error": str(exc)}), 400

if __name__ == "__main__":
    host = os.getenv("PICK_API_HOST", "127.0.0.1")
    port = int(os.getenv("PICK_API_PORT", "8001"))
    app.run(host=host, port=port, threaded=True, use_reloader=False)
