# To Run on the host
'''
PYTHONPATH=src python -m lerobot.robots.xlerobot.xlerobot_host --robot.id=my_xlerobot
'''

# To Run the teleop:
'''
PYTHONPATH=src python examples/5_xlerobot_teleop_xbox.py
# Optional: --robot.id=my_xlerobot --robot.port1=/dev/ttyACM0 --robot.port2=/dev/ttyACM1 --fps=30 --controller.profile=generic
'''

import argparse
import time
import numpy as np
import math
import pygame

from lerobot.robots.xlerobot import XLerobotConfig, XLerobot
# from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from lerobot.model.SO101Robot import SO101Kinematics

# Keymaps (semantic action: controller mapping) - Intuitive human control
LEFT_KEYMAP = {
    # Left stick controls left arm XY (when not pressed)
    'x+': 'left_stick_up', 'x-': 'left_stick_down',
    'y+': 'left_stick_right', 'y-': 'left_stick_left',
    # Left stick pressed controls left arm shoulder_pan
    'shoulder_pan+': 'left_stick_pressed_right', 'shoulder_pan-': 'left_stick_pressed_left',
    # LB pressed controls left arm pitch and wrist_roll
    'pitch+': 'lb_up', 'pitch-': 'lb_down',
    'wrist_roll+': 'lb_right', 'wrist_roll-': 'lb_left',
    # Left trigger controls left gripper
    'gripper+': 'left_trigger',
    # Head motors
    "head_motor_1+": 'x', "head_motor_1-": 'b',
    "head_motor_2+": 'a', "head_motor_2-": 'y',
}
RIGHT_KEYMAP = {
    # Right stick controls right arm XY (when not pressed)
    'x+': 'right_stick_up', 'x-': 'right_stick_down',
    'y+': 'right_stick_right', 'y-': 'right_stick_left',
    # Right stick pressed controls right arm shoulder_pan
    'shoulder_pan+': 'right_stick_pressed_right', 'shoulder_pan-': 'right_stick_pressed_left',
    # RB pressed controls right arm pitch and wrist_roll
    'pitch+': 'rb_up', 'pitch-': 'rb_down',
    'wrist_roll+': 'rb_right', 'wrist_roll-': 'rb_left',
    # Right trigger controls right gripper
    'gripper+': 'right_trigger',
}

# Base control keymap - Only forward/backward and rotate left/right
BASE_KEYMAP = {
    'forward': 'dpad_down', 'backward': 'dpad_up',
    'rotate_left': 'dpad_left', 'rotate_right': 'dpad_right',
}

# Button/axis index layouts differ across pads even when they nominally follow the
# Xbox layout (A/B/X/Y, dual sticks, D-pad, LB/RB/LT/RT). "xbox" matches a genuine
# Xbox controller under Linux/pygame; "generic" matches common cheap USB/2.4G pads
# (e.g. DragonRise-chipset "PS2-look" controllers) - calibrate a new one by running
# calibrate_gamepad.py and comparing its output against these values.
CONTROLLER_PROFILES = {
    "xbox": {
        "a": 0, "b": 1, "x": 2, "y": 3,
        "lb": 4, "rb": 5, "back": 6,
        "left_stick_click": 9, "right_stick_click": 10,
        "left_stick_x": 0, "left_stick_y": 1,
        "right_stick_x": 3, "right_stick_y": 4,
        "left_trigger_axis": 2, "right_trigger_axis": 5,
        "trigger_threshold": 0.5,
    },
    "generic": {
        "a": 0, "b": 1, "x": 3, "y": 4,
        "lb": 6, "rb": 7, "back": 10,
        "left_stick_click": 13, "right_stick_click": 14,
        "left_stick_x": 0, "left_stick_y": 1,
        "right_stick_x": 2, "right_stick_y": 3,
        "left_trigger_axis": 4, "right_trigger_axis": 5,
        "trigger_threshold": 0.0,
    },
}

# Global reset key for all components
RESET_KEY = 'back'

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

class SimpleHeadControl:
    def __init__(self, initial_obs, kp=1):
        self.kp = kp
        self.degree_step = 1
        # Initialize head motor positions
        self.target_positions = {
            "head_motor_1": initial_obs.get("head_motor_1.pos", 0.0),
            "head_motor_2": initial_obs.get("head_motor_2.pos", 0.0),
        }
        self.zero_pos = {"head_motor_1": 0.0, "head_motor_2": 0.0}

    def move_to_zero_position(self, robot):
        self.target_positions = self.zero_pos.copy()
        action = self.p_control_action(robot)
        robot.send_action(action)

    def handle_keys(self, key_state):
        if key_state.get('head_motor_1+'):
            self.target_positions["head_motor_1"] += self.degree_step
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")
        if key_state.get('head_motor_1-'):
            self.target_positions["head_motor_1"] -= self.degree_step
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")
        if key_state.get('head_motor_2+'):
            self.target_positions["head_motor_2"] += self.degree_step
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")
        if key_state.get('head_motor_2-'):
            self.target_positions["head_motor_2"] -= self.degree_step
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")

    def p_control_action(self, robot):
        obs = robot.get_observation()
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{HEAD_MOTOR_MAP[motor]}.pos", 0.0)
            error = self.target_positions[motor] - current
            control = self.kp * error
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current + control
        return action

class SimpleTeleopArm:
    def __init__(self, kinematics, joint_map, initial_obs, prefix="left", kp=1):
        self.kinematics = kinematics
        self.joint_map = joint_map
        self.prefix = prefix  # To distinguish left and right arm
        self.kp = kp
        # Initial joint positions
        self.joint_positions = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }
        # Set initial x/y to fixed values
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        # Set the degree step and xy step
        self.degree_step = 2
        self.xy_step = 0.005
        # Set target positions to zero for P control
        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.zero_pos = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }

    def move_to_zero_position(self, robot):
        print(f"[{self.prefix}] Moving to Zero Position: {self.zero_pos} ......")
        self.target_positions = self.zero_pos.copy()  # Use copy to avoid reference issues
        
        # Reset kinematic variables to their initial state
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        
        # Don't let handle_keys recalculate wrist_flex - set it explicitly
        self.target_positions["wrist_flex"] = 0.0
        
        action = self.p_control_action(robot)
        robot.send_action(action)

    def handle_keys(self, key_state):
        # Joint increments
        if key_state.get('shoulder_pan+'):
            self.target_positions["shoulder_pan"] += self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get('shoulder_pan-'):
            self.target_positions["shoulder_pan"] -= self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get('wrist_roll+'):
            self.target_positions["wrist_roll"] += self.degree_step
            print(f"[{self.prefix}] wrist_roll: {self.target_positions['wrist_roll']}")
        if key_state.get('wrist_roll-'):
            self.target_positions["wrist_roll"] -= self.degree_step
            print(f"[{self.prefix}] wrist_roll: {self.target_positions['wrist_roll']}")
        
        # Gripper control with auto-close functionality
        if key_state.get('gripper+'):
            # Trigger pressed - open gripper (0.1)
            self.target_positions["gripper"] = 2
            print(f"[{self.prefix}] gripper: CLOSED")
        else:
            self.target_positions["gripper"] = 90
            print(f"[{self.prefix}] gripper: auto-opening to {self.target_positions['gripper']:.1f}")
        
        if key_state.get('pitch+'):
            self.pitch += self.degree_step
            print(f"[{self.prefix}] pitch: {self.pitch}")
        if key_state.get('pitch-'):
            self.pitch -= self.degree_step
            print(f"[{self.prefix}] pitch: {self.pitch}")

        # XY plane (IK)
        moved = False
        if key_state.get('x+'):
            self.current_x += self.xy_step
            moved = True
            print(f"[{self.prefix}] x+: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get('x-'):
            self.current_x -= self.xy_step
            moved = True
            print(f"[{self.prefix}] x-: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get('y+'):
            self.current_y += self.xy_step
            moved = True
            print(f"[{self.prefix}] x: {self.current_x:.4f}, y+: {self.current_y:.4f}")
        if key_state.get('y-'):
            self.current_y -= self.xy_step
            moved = True
            print(f"[{self.prefix}] x: {self.current_x:.4f}, y-: {self.current_y:.4f}")
        if moved:
            joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
            self.target_positions["shoulder_lift"] = joint2
            self.target_positions["elbow_flex"] = joint3
            print(f"[{self.prefix}] shoulder_lift: {joint2}, elbow_flex: {joint3}")

        # Wrist flex is always coupled to pitch and the other two
        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"]
            -self.target_positions["elbow_flex"]
            + self.pitch
        )
        # print(f"[{self.prefix}] wrist_flex: {self.target_positions['wrist_flex']}")

    def p_control_action(self, robot):
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action
    

# --- XBOX Controller Mapping ---
def get_xbox_key_state(joystick, keymap, profile=CONTROLLER_PROFILES["xbox"]):
    """
    Map controller state to semantic action booleans using the provided keymap
    and button/axis index profile (see CONTROLLER_PROFILES).
    """
    # Read axes, buttons, hats
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    hats = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)

    def btn(name):
        i = profile[name]
        return bool(buttons[i]) if len(buttons) > i else False

    def axis(name):
        i = profile[name]
        return axes[i] if len(axes) > i else 0.0

    # Get stick pressed states
    left_stick_pressed = btn("left_stick_click")
    right_stick_pressed = btn("right_stick_click")
    lb_pressed = btn("lb")
    rb_pressed = btn("rb")
    trigger_threshold = profile["trigger_threshold"]

    # Map controller state to semantic actions
    state = {}
    for action, control in keymap.items():
        if control == 'left_trigger':
            state[action] = axis("left_trigger_axis") > trigger_threshold
        elif control == 'right_trigger':
            state[action] = axis("right_trigger_axis") > trigger_threshold
        elif control == 'a':
            state[action] = btn("a")
        elif control == 'b':
            state[action] = btn("b")
        elif control == 'x':
            state[action] = btn("x")
        elif control == 'y':
            state[action] = btn("y")
        elif control == 'back':
            state[action] = btn("back")
        elif control == 'dpad_up':
            state[action] = hats[1] == 1
        elif control == 'dpad_down':
            state[action] = hats[1] == -1
        elif control == 'dpad_left':
            state[action] = hats[0] == -1
        elif control == 'dpad_right':
            state[action] = hats[0] == 1
        # Left stick controls (when not pressed)
        elif control == 'left_stick_up':
            state[action] = (not left_stick_pressed) and (not lb_pressed) and (axis("left_stick_y") < -0.5)
        elif control == 'left_stick_down':
            state[action] = (not left_stick_pressed) and (not lb_pressed) and (axis("left_stick_y") > 0.5)
        elif control == 'left_stick_left':
            state[action] = (not left_stick_pressed) and (not lb_pressed) and (axis("left_stick_x") < -0.5)
        elif control == 'left_stick_right':
            state[action] = (not left_stick_pressed) and (not lb_pressed) and (axis("left_stick_x") > 0.5)
        # Right stick controls (when not pressed)
        elif control == 'right_stick_up':
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (axis("right_stick_y") < -0.5)
        elif control == 'right_stick_down':
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (axis("right_stick_y") > 0.5)
        elif control == 'right_stick_left':
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (axis("right_stick_x") < -0.5)
        elif control == 'right_stick_right':
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (axis("right_stick_x") > 0.5)
        # Left stick pressed controls
        elif control == 'left_stick_pressed_right':
            state[action] = left_stick_pressed and (not lb_pressed) and (axis("left_stick_x") > 0.5)
        elif control == 'left_stick_pressed_left':
            state[action] = left_stick_pressed and (not lb_pressed) and (axis("left_stick_x") < -0.5)
        # Right stick pressed controls
        elif control == 'right_stick_pressed_right':
            state[action] = right_stick_pressed and (not rb_pressed) and (axis("right_stick_x") > 0.5)
        elif control == 'right_stick_pressed_left':
            state[action] = right_stick_pressed and (not rb_pressed) and (axis("right_stick_x") < -0.5)
        # LB pressed controls (only when stick is moved)
        elif control == 'lb_up':
            state[action] = lb_pressed and (abs(axis("left_stick_y")) > 0.5) and (axis("left_stick_y") < -0.5)
        elif control == 'lb_down':
            state[action] = lb_pressed and (abs(axis("left_stick_y")) > 0.5) and (axis("left_stick_y") > 0.5)
        elif control == 'lb_right':
            state[action] = lb_pressed and (abs(axis("left_stick_x")) > 0.5) and (axis("left_stick_x") > 0.5)
        elif control == 'lb_left':
            state[action] = lb_pressed and (abs(axis("left_stick_x")) > 0.5) and (axis("left_stick_x") < -0.5)
        # RB pressed controls (only when stick is moved)
        elif control == 'rb_up':
            state[action] = rb_pressed and (abs(axis("right_stick_y")) > 0.5) and (axis("right_stick_y") < -0.5)
        elif control == 'rb_down':
            state[action] = rb_pressed and (abs(axis("right_stick_y")) > 0.5) and (axis("right_stick_y") > 0.5)
        elif control == 'rb_right':
            state[action] = rb_pressed and (abs(axis("right_stick_x")) > 0.5) and (axis("right_stick_x") > 0.5)
        elif control == 'rb_left':
            state[action] = rb_pressed and (abs(axis("right_stick_x")) > 0.5) and (axis("right_stick_x") < -0.5)
        else:
            state[action] = False
    return state

def get_base_action(joystick, robot):
    """
    Get base action from XBOX controller input - simplified to only forward/backward and rotate.
    """
    # Read controller state
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    hats = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)
    
    # Get pressed keys for base control
    pressed_keys = set()
    
    # Map controller inputs to keyboard-like keys for base control
    if hats[1] == 1:   # D-pad up
        pressed_keys.add('k')  # Forward
    if hats[1] == -1:  # D-pad down
        pressed_keys.add('i')  # Backward
    if hats[0] == -1:  # D-pad left
        pressed_keys.add('u')  # Rotate left
    if hats[0] == 1:   # D-pad right
        pressed_keys.add('o')  # Rotate right
    
    # Convert to numpy array and get base action
    keyboard_keys = np.array(list(pressed_keys))
    base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}
    
    return base_action

def get_base_speed_control(joystick, profile=CONTROLLER_PROFILES["xbox"]):
    """
    Get base speed control from XBOX controller - LB for speed decrease, RB for speed increase.
    Returns speed multiplier (1.0, 2.0, or 3.0) and prints current speed level.
    """
    # Read controller state
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

    # Get LB and RB states
    lb_i, rb_i = profile["lb"], profile["rb"]
    lb_pressed = bool(buttons[lb_i]) if len(buttons) > lb_i else False
    rb_pressed = bool(buttons[rb_i]) if len(buttons) > rb_i else False
    
    # Get current speed level from global variable
    global current_base_speed_level
    if 'current_base_speed_level' not in globals():
        current_base_speed_level = 1  # Default speed level
    
    # Speed control logic
    if lb_pressed and not rb_pressed:
        # LB pressed alone - decrease speed
        if current_base_speed_level > 1:
            current_base_speed_level -= 1
            print(f"[BASE] Speed decreased to level {current_base_speed_level}")
    elif rb_pressed and not lb_pressed:
        # RB pressed alone - increase speed
        if current_base_speed_level < 3:
            current_base_speed_level += 1
            print(f"[BASE] Speed increased to level {current_base_speed_level}")
    
    # Map speed level to multiplier
    speed_multiplier = float(current_base_speed_level)
    
    return speed_multiplier


def main():
    parser = argparse.ArgumentParser(description="XLerobot Xbox Teleop")
    parser.add_argument("--robot.id", type=str, default="my_xlerobot", help="Robot config id")
    parser.add_argument("--robot.port1", type=str, default="/dev/ttyACM0", help="Port 1 (so101 + head camera)")
    parser.add_argument("--robot.port2", type=str, default="/dev/ttyACM1", help="Port 2 (same as lekiwi setup)")
    parser.add_argument("--fps", type=int, default=30, help="Control loop frequency")
    parser.add_argument(
        "--controller.profile", type=str, default="xbox", choices=list(CONTROLLER_PROFILES),
        help="Button/axis index layout to use - 'xbox' for a real Xbox controller, 'generic' for "
             "common cheap USB/2.4G pads (calibrate a new one with calibrate_gamepad.py)",
    )
    args = parser.parse_args()

    FPS = args.fps
    profile = CONTROLLER_PROFILES[getattr(args, "controller.profile")]
    robot_config = XLerobotConfig(
        id=getattr(args, "robot.id"),
        port1=getattr(args, "robot.port1"),
        port2=getattr(args, "robot.port2"),
    )
    robot = XLerobot(robot_config)
    try:
        robot.connect()
        print(f"[MAIN] Successfully connected to robot")
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(robot_config)
        print(robot)
        return

    init_rerun(session_name="xlerobot_teleop_xbox")

    # Init XBOX controller
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No XBOX controller detected!")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"[MAIN] Using controller: {joystick.get_name()} (profile: {getattr(args, 'controller.profile')})")

    # Init the arm and head instances
    obs = robot.get_observation()
    kin_left = SO101Kinematics()
    kin_right = SO101Kinematics()
    left_arm = SimpleTeleopArm(kin_left, LEFT_JOINT_MAP, obs, prefix="left")
    right_arm = SimpleTeleopArm(kin_right, RIGHT_JOINT_MAP, obs, prefix="right")
    head_control = SimpleHeadControl(obs)

    # Move both arms and head to zero position at start
    left_arm.move_to_zero_position(robot)
    right_arm.move_to_zero_position(robot)

    try:
        while True:
            pygame.event.pump()
            left_key_state = get_xbox_key_state(joystick, LEFT_KEYMAP, profile)
            right_key_state = get_xbox_key_state(joystick, RIGHT_KEYMAP, profile)
            
            # Check for global reset (back button)
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            back_i = profile["back"]
            global_reset = bool(buttons[back_i]) if len(buttons) > back_i else False
            
            # Handle global reset for all components
            if global_reset:
                print("[MAIN] Global reset triggered!")
                left_arm.move_to_zero_position(robot)
                right_arm.move_to_zero_position(robot)
                head_control.move_to_zero_position(robot)
                continue

            # Handle both arms separately and simultaneously
            left_arm.handle_keys(left_key_state)
            right_arm.handle_keys(right_key_state)
            head_control.handle_keys(left_key_state)  # Head controlled by left arm keymap

            left_action = left_arm.p_control_action(robot)
            right_action = right_arm.p_control_action(robot)
            head_action = head_control.p_control_action(robot)

            # Get base action and speed control from controller
            base_action = get_base_action(joystick, robot)
            speed_multiplier = get_base_speed_control(joystick, profile)
            
            # Apply speed multiplier to base actions if they exist
            if base_action:
                for key in base_action:
                    if 'vel' in key or 'velocity' in key:  # Apply to velocity commands
                        base_action[key] *= speed_multiplier

            # Merge all actions
            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)

            obs = robot.get_observation()
            log_rerun_data(obs, action)
    finally:
        robot.disconnect()
        print("Teleoperation ended.")

if __name__ == "__main__":
    main()