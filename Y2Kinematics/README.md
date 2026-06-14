# Y2Kinematics

Robot kinematics library.

## Purpose

Provides forward kinematics, Jacobian calculations, and robot-specific
kinematics classes for UR and KUKA-style manipulators. Inverse kinematics is
solved with damped least squares (DLS).

## Main Usage

`Y2RobMotion` selects the active kinematics type through
`Y2RobMotion/include/Y2RobMotion/setup_parameters.hpp`.

## Dependencies

- `Y2Matrix`
- `rclcpp`

## License

Proprietary. See `../LICENSE`.
