# Requirements

## Platform

- Ubuntu 22.04
- ROS 2 Humble
- CMake 3.8 or newer
- C++17 compiler
- `colcon`

## ROS 2 Packages

Install the normal ROS 2 build tools and message packages:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  ros-humble-rclcpp \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-srvs \
  ros-humble-yaml-cpp-vendor
```

For real UR robot operation, install and source the Universal Robots ROS 2 driver stack:

```bash
sudo apt install -y ros-humble-ur-robot-driver
```

For KUKA/LBR-related builds, provide the matching `lbr_fri_idl` and `lbr_fri_ros2` packages in the workspace or underlay.

## External Libraries

`gforce_control` requires LibTorch. Set `LIBTORCH` before building:

```bash
export LIBTORCH=/home/jay/libtorch-shared-with-deps-2.8.0+cpu
```

The path may point either to the LibTorch root or to a parent directory containing `libtorch/`.

## Model Checkpoint

Place the TorchScript GForce model here:

```text
src/armcon_pack/gforce_control/checkpoints/ts_0p008_sen_ts_0p002/gforce.pt
```

The checkpoint profile is selected in:

```text
src/armcon_pack/gforce_control/config/gforce_control.yaml
```

## Build

From the workspace root:

```bash
cd /home/jay/armcon_ws
source /opt/ros/humble/setup.bash
export LIBTORCH=/home/jay/libtorch-shared-with-deps-2.8.0+cpu
colcon build
source install/setup.bash
```
