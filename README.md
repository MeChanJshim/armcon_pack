# ArmCon Pack

ArmCon Pack is a ROS 2 Humble workspace package bundle for UR/KUKA-style arm
motion, force sensing, trajectory generation, Isaac Sim bridging, and GForce
model-based 6-DOF force control.

Maintainer: Jaeyun Sim <wodbs02221@gmail.com>

License: Proprietary. See [LICENSE](LICENSE).

## Layout

```text
armcon_pack/
  gforce_control/   GForce 6-DOF force-control library and TorchScript wrapper
  Y2RobMotion/      Single-arm motion node, command node, measurement node
  Y2Matrix/         Matrix and rotation utilities
  Y2Kinematics/     Kinematics models and Jacobian utilities
  Y2Trajectory/     Trajectory profiling and blending
  Y2Filters/        Signal filtering utilities
  Y2FT_AQ/          Force/torque acquisition and processing utilities
  HTM_Jaco_Cal_Python/
                    SymPy-based HTM/Jacobian generator for C++ kinematics code
  Y2ForceCon/       Legacy force-control package retained for reference
  Y2_Joystick/      Joystick message mapping tools
  y2_isaac_bridge/  Isaac Sim bridge nodes
```

## Primary Runtime

For the integrated GForce-controlled single-arm runtime:

```bash
cd /home/jay/armcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch Y2RobMotion ur_singleArm.launch.py
```

For Isaac Sim remapping:

```bash
ros2 launch Y2RobMotion ur_issacArm.launch.py
```

For UR driver bringup:

```bash
ros2 launch Y2RobMotion ur10e_control.launch.py
```

## Configuration

Robot runtime settings:

```text
Y2RobMotion/config/y2_rob_motion.yaml
```

GForce controller settings:

```text
gforce_control/config/gforce_control.yaml
```

TorchScript checkpoint location:

```text
gforce_control/checkpoints/ts_0p008_sen_ts_0p002/gforce.pt
```

## Kinematics Code Generation

`HTM_Jaco_Cal_Python` generates symbolic homogeneous transformation matrices
and Jacobians from DH parameters. Running the scripts writes C++ assignment
blocks to `.txt` files that can be pasted into `Y2Kinematics/src`.

```bash
cd /home/jay/armcon_ws/src/armcon_pack/HTM_Jaco_Cal_Python
python3 Kinematics_UR10.py
python3 Kinematics_KUKAiiwa7.py
python3 Kinematics.py
```

Generated files:

```text
HTM_Jaco_Cal_Python/Kinematics_UR10_HTM_Jacobian_cpp.txt
HTM_Jaco_Cal_Python/Kinematics_KUKAiiwa7_HTM_Jacobian_cpp.txt
HTM_Jaco_Cal_Python/Kinematics_HTM_Jacobian_cpp.txt
```

Use the `T[...] = ...;` block in `forwardKinematics()` and the `J[...] = ...;`
block in `calculateJacobian()`.

## Requirements

See [REQUIREMENTS.md](REQUIREMENTS.md) for the full installation list.

Required build/runtime APIs include:

```text
ROS 2 Humble client libraries and message packages
LibTorch C++ API for gforce_control
yaml-cpp for runtime configuration
Universal Robots ROS 2 driver for real UR robot operation
lbr_fri_idl and lbr_fri_ros2 for KUKA/LBR builds
SymPy for HTM_Jaco_Cal_Python code generation
```

Install Python helper dependencies with:

```bash
cd /home/jay/armcon_ws/src/armcon_pack
python3 -m pip install -r requirements.txt
```

## Notes for Distribution

- `Y2ForceCon` is retained as a legacy/reference package. The current
  `Y2RobMotion` force-control integration uses `gforce_control` directly.
- `Y2RobMotion/src/force_control.cpp` is not used in this distribution.
- Package names inherited from the original Y2 packages use uppercase letters,
  so ROS 2 package-name convention warnings are expected during build.
