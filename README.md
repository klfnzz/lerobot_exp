# lerobot_exp

Workspace for LeRobot, Piper, and RealSense experiments.

This repository is intended to track the workspace layout and local integration
work. Most large upstream projects are kept as Git submodules so their own
histories stay intact and generated build outputs are not uploaded.

## Contents

- `lerobot`: upstream Hugging Face LeRobot checkout.
- `lerobot_piper3`: Piper integration for LeRobot. The local branch includes a
  migration from `piper_sdk` to `pyAgxArm` for the Piper motor bus.
- `piper/dual_piper`: AgileX dual Piper examples.
- `piper/piper_isaac_sim`: AgileX Piper Isaac Sim assets/examples.
- `piper/piper_ros`: AgileX Piper ROS package.
- `piper/piper_sdk`: legacy Piper SDK source.
- `piper/pyAgxArm`: newer AgileX arm SDK source.
- `realsense/librealsense`: Intel RealSense SDK source.

## Clone

```bash
git clone --recurse-submodules <repo-url>
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```
