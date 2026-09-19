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
- `lerobot_robot_xlerobot_2wheels/` — packages `../src/robots/xlerobot_2wheels`
  (`--robot.type=xlerobot_2wheels`).
- `lerobot_robot_xlerobot/` — packages `../src/robots/xlerobot`, the original
  3-omniwheel base (`--robot.type=xlerobot`).
- `lerobot_robot_xlerobot_mecanum/` — packages `../src/robots/xlerobot_mecanum`
  (`--robot.type=xlerobot_mecanum`).

lerobot auto-imports any installed distribution whose name starts with
`lerobot_robot_`, `lerobot_teleoperator_`, etc., so once one of these is installed,
its `--robot.type=...` works on any standard lerobot CLI (`lerobot-teleoperate`,
`lerobot-record`, ...) with no other setup.

## Local install (editable, for development)

```bash
pip install -e software/plugins/xlerobot_model
pip install -e software/plugins/lerobot_robot_xlerobot_2wheels
pip install -e software/plugins/lerobot_robot_xlerobot
pip install -e software/plugins/lerobot_robot_xlerobot_mecanum
```

All are editable installs backed by symlinks into `../src/`, so edits to the real
source files under `software/src/` take effect immediately — no reinstall needed.

## Verifying discovery

```bash
python -c "
from lerobot.utils.import_utils import register_third_party_plugins
register_third_party_plugins()
from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config
for t in ['xlerobot', 'xlerobot_mecanum', 'xlerobot_2wheels']:
    print(t, '->', make_robot_from_config(RobotConfig.get_choice_class(t)(id='test')))
"
```

## Notes / history

`xlerobot` and `xlerobot_mecanum` both used to register
`RobotConfig.register_subclass("xlerobot")` (copy-pasted from one to the other).
That was harmless under the old copy-based install, since only one of them was ever
imported into the same process at a time, but it becomes a real, silent collision
once both are installed as plugins simultaneously — whichever gets imported last
wins the registry key. Fixed by giving `xlerobot_mecanum` its own
`"xlerobot_mecanum"` / `"xlerobot_mecanum_client"` keys; class names were left
unchanged (still `XLerobotConfig`/`XLerobot` in both packages — they don't collide,
since each lives in its own package's own namespace).

The `xlerobot_vr` teleoperator isn't converted yet — the same pattern applies
(a sibling directory here with a `pyproject.toml` and a symlink to
`../src/teleporators/xlerobot_vr`), but nothing currently deployed depends on it.
