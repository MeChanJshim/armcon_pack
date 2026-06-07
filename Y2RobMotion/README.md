# Y2RobMotion

ROS 2 single-arm motion-control package.

## Purpose

Provides the main single-arm controller node, command sender, measurement node,
launch files, and runtime configuration for ArmCon Pack.

## Main Executables

- `singleArm_motion`: main robot motion node
- `singleArm_cmd`: interactive command sender
- `robot_measure`: topic logger for current/target pose, joint, force, and MDK data

## Launch

```bash
ros2 launch Y2RobMotion ur_singleArm.launch.py
```

Isaac Sim remap:

```bash
ros2 launch Y2RobMotion ur_issacArm.launch.py
```

UR driver launch:

```bash
ros2 launch Y2RobMotion ur10e_control.launch.py
```

## Configuration

Runtime settings are stored in:

```text
config/y2_rob_motion.yaml
```

Only compile-time kinematics class selection remains in:

```text
include/Y2RobMotion/setup_parameters.hpp
```

Force control is delegated to `gforce_control`. The legacy `force_control.cpp`
flow is not used in this distribution.

## License

Proprietary. See `../LICENSE`.
