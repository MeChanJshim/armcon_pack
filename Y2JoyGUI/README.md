# Y2JoyGUI

Browser GUI for publishing the same operator command topics used by
`Y2_Joystick`.

## Run

```bash
colcon build --packages-select Y2JoyGUI
source install/setup.bash
ros2 launch Y2JoyGUI y2_joy_gui.launch.py
```

Open `http://127.0.0.1:8080` in a browser.

Open `http://127.0.0.1:8080/nodes.html` for the hidden node runner page.

## Published Topics

- `/ur10skku/cmdMode` (`std_msgs/msg/String`)
- `/ur10skku/record_boolean_R` (`std_msgs/msg/Bool`)
- `/ur10skku/record_boolean_L` (`std_msgs/msg/Bool`)
- `/ur10skku/joy_move` (`std_msgs/msg/Float64MultiArray`)
- `/ur10skku/cmdMode` (`std_msgs/msg/String`, PTP/TXTLoad mode changes)
- `/ur10skku/cmdMotion` (`std_msgs/msg/Float64MultiArray`, generated motion samples)

Subscribed topic: `/ur10skku/currentP` (`std_msgs/msg/Float64MultiArray`).

PTP and TXTLoad trajectory generation runs inside the Y2JoyGUI node; `singleArm_cmd` is not required. The corresponding `singleArm_motion` node must be running to provide `currentP` and consume `cmdMotion`. TXTLoad uses the `trajectory_mode` configured in `Y2RobMotion/config/y2_rob_motion.yaml`.

The Pause buttons temporarily suspend sample publishing while preserving the active trajectory and mode; the button changes to Resume to continue. The Stop buttons discard either active trajectory and publish `Idling` mode.
