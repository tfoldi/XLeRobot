# lerobot plugin packages

These are small, independently pip-installable packages that let lerobot discover
XLeRobot's robot/model code via lerobot's own third-party plugin mechanism
(`register_third_party_plugins()` in `lerobot.utils.import_utils`), instead of
copying files into lerobot's own `src/lerobot/` tree.

Each directory here is just packaging metadata (a `pyproject.toml` + a symlink) that
points back at the real, unmoved source under `../src/`. No source files were moved
or renamed to make this work.

- `xlerobot_model/` — packages `../src/model` as the plain (non-plugin) dependency
  `xlerobot_model`, used directly by robot/teleop code and examples.
- `lerobot_robot_xlerobot_2wheels/` — packages `../src/robots/xlerobot_2wheels` as
  `lerobot_robot_xlerobot_2wheels`. lerobot auto-imports any installed distribution
  whose name starts with `lerobot_robot_`, `lerobot_teleoperator_`, etc., so once this
  is installed, `--robot.type=xlerobot_2wheels` works on any standard lerobot CLI
  (`lerobot-teleoperate`, `lerobot-record`, ...) with no other setup.

## Local install (editable, for development)

```bash
pip install -e software/plugins/xlerobot_model
pip install -e software/plugins/lerobot_robot_xlerobot_2wheels
```

Both are editable installs backed by symlinks into `../src/`, so edits to the real
source files under `software/src/` take effect immediately — no reinstall needed.

## Verifying discovery

```bash
python -c "
from lerobot.utils.import_utils import register_third_party_plugins
register_third_party_plugins()
from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config
robot = make_robot_from_config(RobotConfig.get_choice_class('xlerobot_2wheels')(id='test'))
print(robot)
"
```

## Adding more variants

The same pattern applies to `xlerobot`, `xlerobot_mecanum`, and the `xlerobot_vr`
teleoperator — add a sibling directory here with a `pyproject.toml` and a symlink
to the corresponding folder under `../src/`. Note: `xlerobot` and `xlerobot_mecanum`
currently both register `RobotConfig.register_subclass("xlerobot")` — installing
both plugins at once will silently shadow one of them, so that name collision
should be resolved in the source configs before packaging both.
