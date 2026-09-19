# Y2FT_AQ

Force/torque acquisition utility package.

## Purpose

Provides ROS 2 components for force/torque sensor data acquisition, processing,
and publication.

## Dependencies

- `Y2Filters`
- `Y2Matrix`
- `rclcpp`
- `std_msgs`
- `geometry_msgs`
- `std_srvs`

## License

Proprietary. See `../LICENSE`.

## Measurement validity

`FTData::fresh` is true only when a getter consumes a new finite sample.
Callers retaining the getter API must check it before treating data as new.
Tare counts only new samples; eCAN publishes a sample after receiving both a
complete force frame and a complete moment frame, including fragmented TCP reads.

`FTGetMain` waits for a valid robot pose and completed tare before publishing.
Keep the tool stationary and unloaded during tare. It stops publishing when
sensor samples stop or the robot pose is older than 250 ms. Gravity torque uses
the configured sensor-frame CoG transformed into the same axes as gravity.

Regression commands are in `../tests/README.md`.
