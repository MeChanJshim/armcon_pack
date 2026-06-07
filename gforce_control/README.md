# gforce_control

Library-only ROS 2 package for GForce 6-DOF force control.

## Purpose

`gforce_control` wraps the TorchScript model trained from `gforce_training` and
provides a reusable C++ API for 6-DOF pose/force command generation.

## Main APIs

- `gforce_control::AdmittanceControl`
- `gforce_control::GForceControl`
- `gforce_control::SixDofForceController`

## Configuration

Edit:

```text
config/gforce_control.yaml
```

Important fields:

- `axis_mode`: `base_rotation`, `ee_rotation`, or `desired_force_vector`
- `axis_rotation`: row-major 3x3 rotation matrix. Local z, column 2, is the force axis.
- `checkpoint.profile`: checkpoint folder name
- `checkpoint.model_file_name`: usually `gforce.pt`
- `mass`, `damping`, `stiffness`: admittance parameters
- `thresholds`: force switching thresholds

## Model

Expected model path:

```text
checkpoints/ts_0p008_sen_ts_0p002/gforce.pt
```

## License

Proprietary. See `../LICENSE`.
