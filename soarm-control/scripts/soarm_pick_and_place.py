#!/usr/bin/env python3
"""
抓取物块并放下的连续动作序列
基于 soarm_set_joints.py 的逻辑，串联多个预设位置
"""

import argparse
import time
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

# 动作序列：每一步是 (名称, 关节角度字典, 夹爪值)
PICK_AND_PLACE_SEQUENCE = [
    ("准备位置 - 张开夹爪", {
        "shoulder_pan": -2.505, "shoulder_lift": 143.692,
        "elbow_flex": -109.055, "wrist_flex": 85.451, "wrist_roll": -1.626,
    }, 50),
    ("靠近物块 - 收紧夹爪", {
        "shoulder_pan": -2.505, "shoulder_lift": 143.692,
        "elbow_flex": -109.055, "wrist_flex": 85.451, "wrist_roll": -1.626,
    }, 12),
    ("抬起物块", {
        "shoulder_pan": 64.923, "shoulder_lift": 99.560,
        "elbow_flex": -94.110, "wrist_flex": 75.868, "wrist_roll": -2.593,
    }, 12),
    ("移动到放置位置 - 张开夹爪", {
        "shoulder_pan": 64.923, "shoulder_lift": 99.560,
        "elbow_flex": -94.110, "wrist_flex": 75.868, "wrist_roll": -2.593,
    }, 50),
    ("放下物块", {
        "shoulder_pan": 64.923, "shoulder_lift": 116.527,
        "elbow_flex": -98.330, "wrist_flex": 76.923, "wrist_roll": -81.538,
    }, 50),
    ("复位 home", {
        "shoulder_pan": 1.626, "shoulder_lift": -104.088,
        "elbow_flex": 97.495, "wrist_flex": 77.714, "wrist_roll": -95.077,
    }, 12),
]

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def move_to(robot, target_joints, gripper_target, speed, hz=30):
    """插值移动到目标位置"""
    obs = robot.get_observation()
    start_joints = {name: float(obs[f"{name}.pos"]) for name in JOINT_NAMES}
    gripper_start = float(obs.get("gripper.pos", 0))

    max_delta = max(abs(target_joints[name] - start_joints[name]) for name in JOINT_NAMES)
    duration = max(0.5, max_delta / speed)
    steps = max(1, int(duration * hz))

    for i in range(1, steps + 1):
        t = i / steps
        action = {}
        for name in JOINT_NAMES:
            action[f"{name}.pos"] = start_joints[name] + (target_joints[name] - start_joints[name]) * t
        action["gripper.pos"] = gripper_start + (gripper_target - gripper_start) * t
        robot.send_action(action)
        time.sleep(1.0 / hz)

    time.sleep(0.3)


def main():
    parser = argparse.ArgumentParser(description="抓取物块放下复位序列")
    parser.add_argument("--speed", type=float, default=20, help="移动速度 (度/秒)")
    parser.add_argument("--pause", type=float, default=0.5, help="每步之间的停顿时间 (秒)")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0")
    parser.add_argument("--robot-id", type=str, default="openclaw_soarm")
    args = parser.parse_args()

    cfg = SO101FollowerConfig(
        port=args.port, id=args.robot_id, cameras={},
        use_degrees=True, disable_torque_on_disconnect=True,
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)
    print("机械臂已连接\n")

    try:
        for i, (name, joints, gripper) in enumerate(PICK_AND_PLACE_SEQUENCE):
            print(f"[{i+1}/{len(PICK_AND_PLACE_SEQUENCE)}] {name} (夹爪: {gripper})")
            move_to(robot, joints, gripper, args.speed)
            print(f"  ✓ 完成")
            time.sleep(args.pause)

        print("\n✓ 全部动作完成")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
