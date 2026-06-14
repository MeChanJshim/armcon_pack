# Requirements

## Platform

- Ubuntu 22.04
- ROS 2 Humble
- CMake 3.8 or newer
- C++17 compiler
- `colcon`
- Python 3

## Required System and ROS 2 Packages

Install the normal ROS 2 build tools, message packages, and C++ libraries:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  libyaml-cpp-dev \
  ros-humble-rclcpp \
  ros-humble-ament-cmake \
  ros-humble-ament-index-cpp \
  ros-humble-ament-index-python \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-srvs \
  ros-humble-launch \
  ros-humble-launch-ros \
  ros-humble-yaml-cpp-vendor
```

## Python Packages

The core controller build is C++/ROS 2. Python is only required for the
HTM/Jacobian generator in `HTM_Jaco_Cal_Python`.

```bash
cd /home/jay/armcon_ws/src/armcon_pack
python3 -m pip install -r requirements.txt
```

This installs:

```text
sympy
```

## Robot and Simulation API Packages

For real UR robot operation, install and source the Universal Robots ROS 2
driver stack:

```bash
sudo apt install -y ros-humble-ur-robot-driver
```

For KUKA/LBR-related builds, provide the matching `lbr_fri_idl` and
`lbr_fri_ros2` packages in the workspace or in a sourced underlay. These are
declared dependencies of `Y2RobMotion`.

For joystick operation, install the ROS joystick driver if you need to publish
`sensor_msgs/msg/Joy` from a physical controller:

```bash
sudo apt install -y ros-humble-joy
```

For Isaac Sim usage, install and configure Isaac Sim separately. This
repository provides ROS 2 bridge-side nodes and launch files, but it does not
install NVIDIA Isaac Sim or its ROS bridge extension.

## External Libraries

`gforce_control` requires the LibTorch C++ API. Download the CPU or CUDA
LibTorch distribution that matches your deployment target, then set `LIBTORCH`
before building:

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
rosdep update
rosdep install --from-paths src --ignore-src -r -y
export LIBTORCH=/home/jay/libtorch-shared-with-deps-2.8.0+cpu
colcon build
source install/setup.bash
```

If the KUKA LBR packages are installed in a non-default prefix, source their
underlay or extend `CMAKE_PREFIX_PATH` before running `colcon build`.
