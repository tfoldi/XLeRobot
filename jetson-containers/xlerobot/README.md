# jetson-containers package for XLeRobot

This is a [jetson-containers](https://github.com/dusty-nv/jetson-containers) package
definition for building an XLeRobot image on a Jetson (JetPack 6 / L4T r36.4+). It's kept
here instead of upstream in jetson-containers itself, since it's specific to this fork.

## Build

```bash
export JETSON_CONTAINERS_PACKAGE_DIRS=/path/to/XLeRobot/jetson-containers/*
jetson-containers build xlerobot
```

(`JETSON_CONTAINERS_PACKAGE_DIRS` requires a jetson-containers build with
[dusty-nv/jetson-containers#1745](https://github.com/dusty-nv/jetson-containers/pull/1745) -
until that merges, use `--package-dirs=/path/to/XLeRobot/jetson-containers/*` instead.)

This installs `xlerobot`, `xlerobot_2wheels`, and `xlerobot_mecanum` (and their shared
model code) as lerobot plugins - see [`software/plugins/README.md`](../software/plugins/README.md)
for how that works. Nothing gets copied into lerobot's own package tree.
