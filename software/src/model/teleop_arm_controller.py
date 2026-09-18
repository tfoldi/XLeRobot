"""Shared arm/head teleop controllers, reused across keyboard/gamepad/VR examples.

Every per-device teleop example (keyboard, xbox, joycon, VR) needs the same two
things: an IK-driven dual-arm controller and a pan/tilt head controller. This
module exists so a new input device doesn't mean re-pasting ~150 lines of
those classes into yet another example file.
"""

LEFT_JOINT_MAP = {
    "shoulder_pan": "left_arm_shoulder_pan",
    "shoulder_lift": "left_arm_shoulder_lift",
    "elbow_flex": "left_arm_elbow_flex",
    "wrist_flex": "left_arm_wrist_flex",
    "wrist_roll": "left_arm_wrist_roll",
    "gripper": "left_arm_gripper",
}
RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}
HEAD_MOTOR_MAP = {
    "head_motor_1": "head_motor_1",
    "head_motor_2": "head_motor_2",
}

ZERO_ARM_POS = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}
ZERO_HEAD_POS = {"head_motor_1": 0.0, "head_motor_2": 0.0}

DEFAULT_START_X = 0.1629
DEFAULT_START_Y = 0.1131


class TeleopArm:
    """IK-driven single-arm controller with a hard per-call step limit.

    Every previous version of this class (keyboard/xbox/joycon/VR, each copied
    into its own example file) used a plain P-controller with kp=1, which
    computes `current + 1.0 * (target - current) == target` - i.e. an instant,
    unbounded jump to whatever the target happens to be. That's fine while a
    caller only ever nudges the target by a tiny amount per tick, but
    `move_to_zero_position()` sets the target directly to the zero pose and
    calls the same P-step once - if the arm was far from zero, it snaps there
    in one motor command. `max_step_deg` makes that structurally impossible:
    every call to `p_control_action()`, including from `move_to_zero_position`,
    is clamped to move at most that many degrees, regardless of kp or error size.
    """

    def __init__(self, kinematics, joint_map, initial_obs, prefix="left", kp=1.0, max_step_deg=3.0):
        self.kinematics = kinematics
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp
        self.max_step_deg = max_step_deg

        self.current_x = DEFAULT_START_X
        self.current_y = DEFAULT_START_Y
        self.pitch = 0.0
        self.target_positions = dict(ZERO_ARM_POS)

    def move_to_zero_position(self, robot, steps=1):
        """Ramp toward the zero pose over `steps` calls instead of a single jump.

        One call still respects max_step_deg, so even steps=1 (the default) is
        safe - use a larger `steps` if you want the whole traversal to happen
        over one connect()/reset() call rather than needing several external
        loop iterations.
        """
        print(f"[{self.prefix}] Moving to Zero Position: {ZERO_ARM_POS} ......")
        self.target_positions = dict(ZERO_ARM_POS)
        self.current_x = DEFAULT_START_X
        self.current_y = DEFAULT_START_Y
        self.pitch = 0.0
        for _ in range(max(1, steps)):
            action = self.p_control_action(robot)
            robot.send_action(action)

    def set_ik_target(self, x, y, pitch=None):
        """Set the arm's end-effector target via IK; wrist_flex stays coupled to pitch."""
        self.current_x = x
        self.current_y = y
        if pitch is not None:
            self.pitch = pitch
        try:
            joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
            self.target_positions["shoulder_lift"] = joint2
            self.target_positions["elbow_flex"] = joint3
        except Exception as e:
            print(f"[{self.prefix}] IK failed at x={x:.4f}, y={y:.4f}: {e}")
        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"] - self.target_positions["elbow_flex"] + self.pitch
        )

    def p_control_action(self, robot):
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.kp * (self.target_positions[j] - current[j])
            step = max(-self.max_step_deg, min(self.max_step_deg, error))
            action[f"{self.joint_map[j]}.pos"] = current[j] + step
        return action


class HeadControl:
    """Pan/tilt head controller with the same per-call step limit as TeleopArm."""

    def __init__(self, initial_obs, kp=1.0, max_step_deg=3.0):
        self.kp = kp
        self.max_step_deg = max_step_deg
        self.target_positions = {
            "head_motor_1": initial_obs.get("head_motor_1.pos", 0.0),
            "head_motor_2": initial_obs.get("head_motor_2.pos", 0.0),
        }

    def move_to_zero_position(self, robot, steps=1):
        self.target_positions = dict(ZERO_HEAD_POS)
        for _ in range(max(1, steps)):
            action = self.p_control_action(robot)
            robot.send_action(action)

    def p_control_action(self, robot):
        obs = robot.get_observation()
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{HEAD_MOTOR_MAP[motor]}.pos", 0.0)
            error = self.kp * (self.target_positions[motor] - current)
            step = max(-self.max_step_deg, min(self.max_step_deg, error))
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current + step
        return action
