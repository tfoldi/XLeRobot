# Prints your gamepad's actual axis/button/hat indices - use this to build a new
# entry in CONTROLLER_PROFILES (5_xlerobot_teleop_xbox.py) for a non-Xbox controller.
import time
import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick detected by pygame. Is it plugged in?")
    raise SystemExit(1)

js = pygame.joystick.Joystick(0)
js.init()
print(f"Name: {js.get_name()}")
print(f"Axes: {js.get_numaxes()}  Buttons: {js.get_numbuttons()}  Hats: {js.get_numhats()}")
print()

STEPS = [
    "A button",
    "B button",
    "X button",
    "Y button",
    "LB / left shoulder button",
    "RB / right shoulder button",
    "LT / left trigger",
    "RT / right trigger",
    "Left stick CLICK (press straight down)",
    "Right stick CLICK (press straight down)",
    "Left stick UP",
    "Left stick DOWN",
    "Left stick LEFT",
    "Left stick RIGHT",
    "Right stick UP",
    "Right stick DOWN",
    "Right stick LEFT",
    "Right stick RIGHT",
    "D-pad UP",
    "D-pad DOWN",
    "D-pad LEFT",
    "D-pad RIGHT",
    "Back/Select button",
    "Start button",
]


def read_state():
    pygame.event.pump()
    axes = tuple(round(js.get_axis(i), 2) for i in range(js.get_numaxes()))
    buttons = tuple(js.get_button(i) for i in range(js.get_numbuttons()))
    hats = tuple(js.get_hat(i) for i in range(js.get_numhats()))
    return axes, buttons, hats


baseline = read_state()
print(f"Baseline (rest state): {baseline}\n")

for step in STEPS:
    print(f">>> Press/hold: {step}  (you have 3 seconds)")
    deadline = time.time() + 3.0
    seen = baseline
    while time.time() < deadline:
        state = read_state()
        if state != baseline:
            seen = state
        time.sleep(0.05)
    if seen != baseline:
        print(f"    CHANGED -> axes={seen[0]} buttons={seen[1]} hats={seen[2]}")
    else:
        print("    (no change detected)")
    print()

print("Calibration finished.")
