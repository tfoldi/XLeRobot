"""
To Run on the host:
PYTHONPATH=src python -m lerobot.robots.xlerobot_2wheels.xlerobot_2wheels_host --robot.id=my_xlerobot_2wheels

To Run the teleop (also starts the VR HTTPS/WS server - open the printed URL in the Quest browser):
PYTHONPATH=src python examples/8_xlerobot_2wheels_teleop_vr.py
# Optional: --robot.id=my_xlerobot_2wheels_lab --robot.port1=/dev/ttyACM0 --robot.port2=/dev/ttyACM1 --ip=localhost --fps=30 --vr.scale=1.0

Each controller's *first* frame after (re)connecting sets an anchor (VR hand
position + current arm end-effector position). Every following frame maps
the hand's offset from that anchor onto the arm's end-effector, scaled by
--vr.scale - this is recomputed from the anchor each frame (not accumulated
frame-to-frame), so it can't drift if a frame is dropped. Triggers close the
grippers; each controller's own thumbstick drives the differential-drive base
(left = forward/backward + rotate) proportionally, no keyboard-key indirection.
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

from lerobot.model.SO101Robot import SO101Kinematics
from lerobot.model.teleop_arm_controller import (
    LEFT_JOINT_MAP,
    RIGHT_JOINT_MAP,
    HeadControl,
    TeleopArm,
)
from lerobot.robots.xlerobot_2wheels import (
    XLerobot2Wheels,
    XLerobot2WheelsClient,
    XLerobot2WheelsClientConfig,
    XLerobot2WheelsConfig,
)
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

# vr_monitor.py lives in XLeVR/ at the repo root, two levels up from software/examples/.
_XLEVR_DIR = Path(__file__).resolve().parents[2] / "XLeVR"
if str(_XLEVR_DIR) not in sys.path:
    sys.path.insert(0, str(_XLEVR_DIR))

BASE_MAX_LINEAR_SPEED = 0.2  # m/s at full thumbstick deflection
BASE_MAX_ANGULAR_SPEED = 60.0  # deg/s at full thumbstick deflection
BASE_DEADZONE = 0.15


def _default_port(udev_name: str, fallback: str) -> str:
    path = f"/dev/{udev_name}"
    return path if os.path.exists(path) else fallback


class AnchoredArmController:
    """Wraps a TeleopArm with anchor-based (not frame-delta) VR position mapping."""

    def __init__(self, arm: TeleopArm, pos_scale: float):
        self.arm = arm
        self.pos_scale = pos_scale
        self.anchor_vr_pos = None
        self.anchor_x = arm.current_x
        self.anchor_y = arm.current_y

    def recenter(self, vr_pos):
        self.anchor_vr_pos = np.array(vr_pos, dtype=float)
        self.anchor_x = self.arm.current_x
        self.anchor_y = self.arm.current_y

    def handle_vr_goal(self, vr_goal):
        if vr_goal is None or getattr(vr_goal, "target_position", None) is None:
            return

        vr_pos = np.asarray(vr_goal.target_position, dtype=float)
        if self.anchor_vr_pos is None:
            self.recenter(vr_pos)
            return

        offset = (vr_pos - self.anchor_vr_pos) * self.pos_scale
        # Matches the VR axis convention used elsewhere in this repo: VR z -> robot x
        # (negated), VR y -> robot y. VR x drives shoulder_pan (lateral hand sway).
        new_x = self.anchor_x - offset[2]
        new_y = self.anchor_y + offset[1]
        pitch = self.arm.pitch
        if vr_goal.wrist_flex_deg is not None:
            pitch = max(-90.0, min(90.0, vr_goal.wrist_flex_deg))
        self.arm.set_ik_target(new_x, new_y, pitch)

        self.arm.target_positions["shoulder_pan"] = max(-90.0, min(90.0, offset[0] * 400.0))
        if vr_goal.wrist_roll_deg is not None:
            self.arm.target_positions["wrist_roll"] = max(-90.0, min(90.0, vr_goal.wrist_roll_deg))
        if vr_goal.gripper_closed is not None:
            self.arm.target_positions["gripper"] = 2.0 if vr_goal.gripper_closed else 90.0


def get_vr_base_velocity(vr_goal):
    """Continuous thumbstick -> (x.vel, theta.vel), no keyboard-key indirection."""
    if vr_goal is None or not vr_goal.metadata:
        return 0.0, 0.0
    thumb = vr_goal.metadata.get("thumbstick") or {}
    tx, ty = thumb.get("x", 0.0), thumb.get("y", 0.0)
    if abs(tx) < BASE_DEADZONE:
        tx = 0.0
    if abs(ty) < BASE_DEADZONE:
        ty = 0.0
    x_vel = -ty * BASE_MAX_LINEAR_SPEED
    theta_vel = -tx * BASE_MAX_ANGULAR_SPEED
    return x_vel, theta_vel


def main():
    parser = argparse.ArgumentParser(description="XLerobot2Wheels VR Teleop")
    parser.add_argument("--robot.id", type=str, default="my_xlerobot_2wheels_lab", help="Robot config id")
    parser.add_argument("--robot.port1", type=str, default=_default_port("arm_left", "/dev/ttyACM0"))
    parser.add_argument("--robot.port2", type=str, default=_default_port("arm_right", "/dev/ttyACM1"))
    parser.add_argument("--ip", type=str, default="localhost", help="'localhost' or a ZMQ host IP")
    parser.add_argument("--fps", type=int, default=30, help="Control loop frequency")
    parser.add_argument("--vr.scale", type=float, default=1.0, help="VR hand-offset to arm-motion scale")
    args = parser.parse_args()

    fps = args.fps
    pos_scale = getattr(args, "vr.scale")

    if args.ip == "localhost":
        robot = XLerobot2Wheels(
            XLerobot2WheelsConfig(
                id=getattr(args, "robot.id"),
                port1=getattr(args, "robot.port1"),
                port2=getattr(args, "robot.port2"),
            )
        )
    else:
        robot = XLerobot2WheelsClient(
            XLerobot2WheelsClientConfig(remote_ip=args.ip, id=getattr(args, "robot.id"))
        )

    robot.connect()
    print("[MAIN] Successfully connected to robot")

    init_rerun(session_name="xlerobot_2wheels_teleop_vr")

    from vr_monitor import VRMonitor  # noqa: E402  (needs _XLEVR_DIR on sys.path first)

    vr = VRMonitor()
    if not vr.initialize():
        print("[MAIN] Failed to initialize VR monitor")
        return

    import asyncio

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(vr.https_server.start())
    loop.run_until_complete(vr.vr_server.start())
    vr.is_running = True

    obs = robot.get_observation()
    kin_left = SO101Kinematics()
    kin_right = SO101Kinematics()
    left_arm = TeleopArm(kin_left, LEFT_JOINT_MAP, obs, prefix="left")
    right_arm = TeleopArm(kin_right, RIGHT_JOINT_MAP, obs, prefix="right")
    head_control = HeadControl(obs)
    left_ctrl = AnchoredArmController(left_arm, pos_scale)
    right_ctrl = AnchoredArmController(right_arm, pos_scale)

    left_arm.move_to_zero_position(robot, steps=10)
    right_arm.move_to_zero_position(robot, steps=10)
    head_control.move_to_zero_position(robot, steps=10)

    print("[MAIN] Open the URL printed above in the Quest browser to start controlling the robot.")

    try:
        while True:
            loop.run_until_complete(asyncio.sleep(1.0 / fps))

            left_goal = vr.get_left_goal_nowait()
            right_goal = vr.get_right_goal_nowait()
            left_ctrl.handle_vr_goal(left_goal)
            right_ctrl.handle_vr_goal(right_goal)

            left_action = left_arm.p_control_action(robot)
            right_action = right_arm.p_control_action(robot)
            head_action = head_control.p_control_action(robot)

            x_vel, theta_vel = get_vr_base_velocity(left_goal)
            base_action = {"x.vel": x_vel, "theta.vel": theta_vel}

            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)

            obs = robot.get_observation()
            log_rerun_data(obs, action)
    finally:
        loop.run_until_complete(vr.stop_monitoring())
        robot.disconnect()
        print("Teleoperation ended.")


if __name__ == "__main__":
    main()
