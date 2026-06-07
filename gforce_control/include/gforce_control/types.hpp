#pragma once

#include <array>
#include <string>

namespace gforce_control {

using Vector3 = std::array<double, 3>;
using Vector6 = std::array<double, 6>;
using Matrix3 = std::array<double, 9>;

enum class ForceAxisMode {
    BaseAxisRotation,
    EndEffectorAxisRotation,
    DesiredForceVector
};

enum class ForceControllerMode {
    ClassicAdmittance,
    GForce
};

struct AxisSelection {
    ForceAxisMode mode = ForceAxisMode::BaseAxisRotation;
    Matrix3 rotation = {1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0};
};

struct SixDofControllerConfig {
    double dt = 0.008;
    ForceControllerMode controller_mode = ForceControllerMode::GForce;
    AxisSelection axis_selection;

    Vector6 mass = {2.0, 2.0, 2.0, 0.5, 0.5, 0.5};
    Vector6 damping = {6000.0, 6000.0, 6000.0, 100.0, 100.0, 100.0};
    Vector6 stiffness = {2000.0, 2000.0, 2000.0, 100.0, 100.0, 100.0};

    double desired_force_threshold = 0.01;
    double actual_force_threshold = 1.50;
    double precontact_force_hold = 15.0;
    double return_tau_mass = 0.20;
    double return_tau_damping = 0.20;
    double return_tau_stiffness = 0.20;

    double desired_force_axis_eps = 1e-6;
    bool desired_force_is_control_frame = true;
    bool use_force_vector_magnitude_as_axis_command = true;

    std::string checkpoint_profile = "ts_0p008_sen_ts_0p002";
    std::string checkpoint_root = "checkpoints";
    std::string model_file_name = "gforce.pt";
    std::string explicit_model_path;
};

struct ControllerDebugState {
    Matrix3 control_rotation = {1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0};
    Vector3 local_desired_force = {0.0, 0.0, 0.0};
    Vector3 local_external_force = {0.0, 0.0, 0.0};
    Vector6 applied_mass = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Vector6 applied_damping = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Vector6 applied_stiffness = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

}  // namespace gforce_control
