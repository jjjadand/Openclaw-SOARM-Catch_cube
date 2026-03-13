#!/usr/bin/env python3

from __future__ import annotations

import atexit
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from flask import Flask, jsonify, request

repo_root = Path.home() / "lerobot"
if repo_root.is_dir():
    sys.path.insert(0, str(repo_root))

from lerobot.common.robots.so101_follower import SO101Follower, SO101FollowerConfig


SCRIPT_DIR = Path(__file__).resolve().parent
SKILL_DIR = SCRIPT_DIR.parent
URDF_PATH = SKILL_DIR / "references" / "so101_new_calib.urdf"
EE_FRAME = "gripper_frame_link"
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
JOINTS = ARM_JOINTS + ["gripper"]
DEFAULT_SPEED = 0.2
STEP_DELAY_S = 0.05


class SoArmController:
    def __init__(self, port: str, robot_id: str, skip_calibration: bool):
        self.lock = threading.Lock()
        self.model = pin.buildModelFromUrdf(str(URDF_PATH))
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(EE_FRAME)
        self.robot = SO101Follower(
            SO101FollowerConfig(
                port=port,
                id=robot_id,
                disable_torque_on_disconnect=True,
                use_degrees=True,
            )
        )
        self.robot.connect(calibrate=not skip_calibration)

    def close(self) -> None:
        if self.robot.is_connected:
            self.robot.disconnect()

    def _fk_position(self, q: np.ndarray) -> np.ndarray:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.frame_id].translation.copy()

    def _gripper_pct_to_rad(self, gripper_pct: float) -> float:
        lower = self.model.lowerPositionLimit[5]
        upper = self.model.upperPositionLimit[5]
        return lower + (gripper_pct / 100.0) * (upper - lower)

    def _observation_to_q(self, observation: dict[str, float]) -> np.ndarray:
        q = np.zeros(self.model.nq)
        for i, joint in enumerate(ARM_JOINTS):
            q[i] = np.deg2rad(observation[f"{joint}.pos"])
        q[5] = self._gripper_pct_to_rad(observation["gripper.pos"])
        return np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

    def _action_from_q(self, q: np.ndarray, gripper_pct: float) -> dict[str, float]:
        action = {f"{joint}.pos": float(np.rad2deg(q[i])) for i, joint in enumerate(ARM_JOINTS)}
        action["gripper.pos"] = float(gripper_pct)
        return action

    def _solve_ik(
        self,
        q0: np.ndarray,
        target_xyz: np.ndarray,
        max_iters: int,
        tol: float,
        step_size: float,
        damping: float,
    ) -> tuple[np.ndarray, float, int, np.ndarray]:
        q = q0.copy()
        fixed_gripper = q0[5]

        for iteration in range(max_iters):
            current_xyz = self._fk_position(q)
            error = target_xyz - current_xyz
            error_norm = float(np.linalg.norm(error))
            if error_norm < tol:
                return q, error_norm, iteration, current_xyz

            jacobian = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )[:3, :5]

            dq_arm = step_size * jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(3),
                error,
            )
            dq = np.zeros(self.model.nv)
            dq[:5] = dq_arm
            q = pin.integrate(self.model, q, dq)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
            q[5] = fixed_gripper

        final_xyz = self._fk_position(q)
        final_error = float(np.linalg.norm(target_xyz - final_xyz))
        return q, final_error, max_iters, final_xyz

    def _read_state(self) -> tuple[dict[str, float], np.ndarray]:
        observation = self.robot.get_observation()
        q = self._observation_to_q(observation)
        xyz = self._fk_position(q)
        return observation, xyz

    def _send_smooth_action(self, start: dict[str, float], target: dict[str, float], speed: float) -> tuple[dict[str, float], int]:
        if speed <= 0:
            raise ValueError("speed must be > 0.")

        arm_delta = max(abs(target[f"{joint}.pos"] - start[f"{joint}.pos"]) for joint in ARM_JOINTS)
        gripper_delta = abs(target["gripper.pos"] - start["gripper.pos"])
        arm_step_deg = 8.0 * speed
        gripper_step_pct = 10.0 * speed
        steps = max(
            1,
            int(np.ceil(arm_delta / arm_step_deg)) if arm_step_deg > 0 else 1,
            int(np.ceil(gripper_delta / gripper_step_pct)) if gripper_step_pct > 0 else 1,
        )

        sent = target
        for i in range(1, steps + 1):
            alpha = i / steps
            action = {
                f"{joint}.pos": float(start[f"{joint}.pos"] + alpha * (target[f"{joint}.pos"] - start[f"{joint}.pos"]))
                for joint in JOINTS
            }
            sent = self.robot.send_action(action)
            if i < steps:
                time.sleep(STEP_DELAY_S)
        return sent, steps

    def status(self) -> dict:
        with self.lock:
            observation, xyz = self._read_state()
            return {
                "connected": self.robot.is_connected,
                "joints": observation,
                "xyz": xyz.tolist(),
            }

    def move_joints(self, angles: list[float], sleep_s: float, speed: float) -> dict:
        if len(angles) != 6:
            raise ValueError("angles must contain 6 values.")
        if not 0.0 <= angles[5] <= 100.0:
            raise ValueError("gripper must be in [0, 100].")

        target = {f"{joint}.pos": float(value) for joint, value in zip(JOINTS, angles, strict=True)}
        with self.lock:
            before, before_xyz = self._read_state()
            sent, steps = self._send_smooth_action(before, target, speed)
            if sleep_s > 0:
                time.sleep(sleep_s)
            after, after_xyz = self._read_state()
            return {
                "before": before,
                "before_xyz": before_xyz.tolist(),
                "sent": sent,
                "speed": speed,
                "steps": steps,
                "after": after,
                "after_xyz": after_xyz.tolist(),
            }

    def move_xyz(
        self,
        x: float,
        y: float,
        z: float,
        sleep_s: float,
        speed: float,
        max_iters: int,
        tol: float,
        step_size: float,
        damping: float,
    ) -> dict:
        target_xyz = np.array([x, y, z], dtype=float)
        with self.lock:
            before, before_xyz = self._read_state()
            q0 = self._observation_to_q(before)
            gripper_pct = float(before["gripper.pos"])

            q_sol, final_error, iterations, solved_xyz = self._solve_ik(
                q0=q0,
                target_xyz=target_xyz,
                max_iters=max_iters,
                tol=tol,
                step_size=step_size,
                damping=damping,
            )
            if final_error >= tol:
                raise RuntimeError("IK did not converge within tolerance.")

            action = self._action_from_q(q_sol, gripper_pct)
            _, steps = self._send_smooth_action(before, action, speed)
            if sleep_s > 0:
                time.sleep(sleep_s)

            after, after_xyz = self._read_state()
            return {
                "target_xyz": target_xyz.tolist(),
                "before_xyz": before_xyz.tolist(),
                "solved_xyz": solved_xyz.tolist(),
                "measured_xyz": after_xyz.tolist(),
                "final_error": final_error,
                "iterations": iterations,
                "speed": speed,
                "steps": steps,
                "solved_joints_deg": {joint: float(np.rad2deg(q_sol[i])) for i, joint in enumerate(ARM_JOINTS)},
                "before_joints": before,
                "after_joints": after,
            }


def create_app() -> Flask:
    port = os.getenv("SOARM_PORT", "/dev/ttyACM0")
    robot_id = os.getenv("SOARM_ID", "openclaw_soarm")
    skip_calibration = os.getenv("SOARM_SKIP_CALIBRATION", "1") != "0"
    controller = SoArmController(port=port, robot_id=robot_id, skip_calibration=skip_calibration)
    atexit.register(controller.close)

    app = Flask(__name__)
    app.config["controller"] = controller

    @app.get("/healthz")
    def healthz():
        return jsonify({"ok": True, "connected": controller.robot.is_connected})

    @app.get("/joints")
    def joints():
        return jsonify(controller.status())

    @app.post("/move/joints")
    def move_joints():
        payload = request.get_json(force=True, silent=False) or {}
        angles = payload.get("angles")
        sleep_s = float(payload.get("sleep", 2.0))
        speed = float(payload.get("speed", DEFAULT_SPEED))
        return jsonify(controller.move_joints(angles, sleep_s, speed))

    @app.post("/move/xyz")
    def move_xyz():
        payload = request.get_json(force=True, silent=False) or {}
        return jsonify(
            controller.move_xyz(
                x=float(payload["x"]),
                y=float(payload["y"]),
                z=float(payload["z"]),
                sleep_s=float(payload.get("sleep", 2.0)),
                speed=float(payload.get("speed", DEFAULT_SPEED)),
                max_iters=int(payload.get("max_iters", 300)),
                tol=float(payload.get("tol", 1e-3)),
                step_size=float(payload.get("step_size", 0.6)),
                damping=float(payload.get("damping", 1e-4)),
            )
        )

    @app.post("/disconnect")
    def disconnect():
        controller.close()
        return jsonify({"ok": True, "connected": False})

    @app.errorhandler(Exception)
    def handle_error(exc: Exception):
        return jsonify({"ok": False, "error": str(exc)}), 400

    return app


app = create_app()


if __name__ == "__main__":
    host = os.getenv("SOARM_API_HOST", "127.0.0.1")
    port = int(os.getenv("SOARM_API_PORT", "8000"))
    app.run(host=host, port=port, threaded=True, use_reloader=False)
