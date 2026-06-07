#include "gforce_control/six_dof_force_controller.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace gforce_control {
namespace {

constexpr int kForceAxis = 2;

double dot(const Vector3& a, const Vector3& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm(const Vector3& v)
{
    return std::sqrt(dot(v, v));
}

Vector3 cross(const Vector3& a, const Vector3& b)
{
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

Vector3 normalizeOr(const Vector3& v, const Vector3& fallback, double eps)
{
    const double n = norm(v);
    if (n <= eps) return fallback;
    return {v[0] / n, v[1] / n, v[2] / n};
}

Vector3 column(const Matrix3& m, int c)
{
    return {m[c], m[3 + c], m[6 + c]};
}

void setColumn(Matrix3& m, int c, const Vector3& v)
{
    m[c] = v[0];
    m[3 + c] = v[1];
    m[6 + c] = v[2];
}

Vector3 matVec(const Matrix3& r, const Vector3& v)
{
    return {r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
            r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
            r[6] * v[0] + r[7] * v[1] + r[8] * v[2]};
}

Vector3 matTVec(const Matrix3& r, const Vector3& v)
{
    return {r[0] * v[0] + r[3] * v[1] + r[6] * v[2],
            r[1] * v[0] + r[4] * v[1] + r[7] * v[2],
            r[2] * v[0] + r[5] * v[1] + r[8] * v[2]};
}

Matrix3 matMul(const Matrix3& a, const Matrix3& b)
{
    Matrix3 out{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out[3 * r + c] = a[3 * r] * b[c]
                           + a[3 * r + 1] * b[3 + c]
                           + a[3 * r + 2] * b[6 + c];
        }
    }
    return out;
}

Vector3 projectOntoPlane(const Vector3& v, const Vector3& normal)
{
    const double projection = dot(v, normal);
    return {v[0] - projection * normal[0],
            v[1] - projection * normal[1],
            v[2] - projection * normal[2]};
}

Matrix3 spatialAngleToRotation(const Vector3& angle)
{
    const double theta = norm(angle);
    if (theta <= 1e-12) {
        return {1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0};
    }

    const double x = angle[0] / theta;
    const double y = angle[1] / theta;
    const double z = angle[2] / theta;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double v = 1.0 - c;

    return {c + x * x * v,     x * y * v - z * s, x * z * v + y * s,
            y * x * v + z * s, c + y * y * v,     y * z * v - x * s,
            z * x * v - y * s, z * y * v + x * s, c + z * z * v};
}

double smoothingAlpha(double dt, double tau)
{
    if (tau <= 0.0) return 1.0;
    return 1.0 - std::exp(-dt / tau);
}

double initialDampingRatio(double mass, double damping, double stiffness)
{
    const double mk = mass * stiffness;
    if (mk <= 0.0) return 0.0;
    return damping / (2.0 * std::sqrt(mk));
}

double signedForceHold(double desired_force, double hold_force)
{
    return desired_force < 0.0 ? -hold_force : hold_force;
}

template <typename T>
void loadScalarIfPresent(const YAML::Node& node, const char* key, T& value)
{
    if (node[key]) {
        value = node[key].as<T>();
    }
}

template <typename T, std::size_t N>
void loadArrayIfPresent(const YAML::Node& node,
                        const char* key,
                        std::array<T, N>& value)
{
    if (!node[key]) return;
    const YAML::Node array_node = node[key];
    if (!array_node.IsSequence() || array_node.size() != N) {
        throw std::runtime_error(std::string(key) + " must contain exactly " + std::to_string(N) + " values.");
    }
    for (std::size_t i = 0; i < N; ++i) {
        value[i] = array_node[i].as<T>();
    }
}

void returnToInitialMDK(AdmittanceControl& controller,
                        const SixDofControllerConfig& config,
                        int axis)
{
    const double mass_now = controller.monitorMDK(0);
    const double damping_now = controller.monitorMDK(1);
    const double stiffness_now = controller.monitorMDK(2);

    const double alpha_m = smoothingAlpha(config.dt, config.return_tau_mass);
    const double alpha_d = smoothingAlpha(config.dt, config.return_tau_damping);
    const double alpha_k = smoothingAlpha(config.dt, config.return_tau_stiffness);

    const double target_mass = mass_now + alpha_m * (config.mass[axis] - mass_now);
    const double target_stiffness = stiffness_now + alpha_k * (config.stiffness[axis] - stiffness_now);
    const double target_damping_raw = damping_now + alpha_d * (config.damping[axis] - damping_now);

    const double zeta = initialDampingRatio(config.mass[axis],
                                            config.damping[axis],
                                            config.stiffness[axis]);
    const double damping_floor = 2.0 * zeta * std::sqrt(std::max(0.0, target_mass * target_stiffness));
    controller.setMDK(target_mass, std::max(target_damping_raw, damping_floor), target_stiffness);
}

void returnMDToInitialWithZeroStiffness(AdmittanceControl& controller,
                                        const SixDofControllerConfig& config,
                                        int axis)
{
    const double mass_now = controller.monitorMDK(0);
    const double damping_now = controller.monitorMDK(1);

    const double alpha_m = smoothingAlpha(config.dt, config.return_tau_mass);
    const double alpha_d = smoothingAlpha(config.dt, config.return_tau_damping);

    const double target_mass = mass_now + alpha_m * (config.mass[axis] - mass_now);
    const double target_damping_raw = damping_now + alpha_d * (config.damping[axis] - damping_now);

    const double zeta = initialDampingRatio(config.mass[axis],
                                            config.damping[axis],
                                            config.stiffness[axis]);
    const double damping_floor = 2.0 * zeta * std::sqrt(std::max(0.0, target_mass * config.stiffness[axis]));
    controller.setMDK(target_mass, std::max(target_damping_raw, damping_floor), 0.0);
}

}  // namespace

SixDofForceController::SixDofForceController()
{
    configure(SixDofControllerConfig{});
}

SixDofForceController::SixDofForceController(const SixDofControllerConfig& config)
{
    configure(config);
}

void SixDofForceController::configure(const SixDofControllerConfig& config)
{
    config_ = config;
    for (int i = 0; i < 6; ++i) {
        admittance_[i] = AdmittanceControl(config_.dt);
        admittance_[i].setMDK(config_.mass[i], config_.damping[i], config_.stiffness[i]);
    }
    initialized_ = false;
}

void SixDofForceController::configureGForcePolicies(const GForcePolicyConfig& policy_config)
{
    for (int i = 0; i < 3; ++i) {
        GForcePolicyConfig axis_config = policy_config;
        axis_config.dt = static_cast<float>(config_.dt);
        policies_[i] = std::make_unique<GForceControl>(axis_config);
    }
}

void SixDofForceController::configureGForcePoliciesFromModelPath(const std::string& model_path,
                                                                 const GForcePolicyConfig& defaults)
{
    GForcePolicyConfig config = defaults;
    config.model_path = model_path;
    configureGForcePolicies(config);
}

void SixDofForceController::configureGForcePoliciesFromCheckpoint(
    const std::string& package_or_absolute_root,
    const GForcePolicyConfig& defaults)
{
    configureGForcePoliciesFromModelPath(resolveModelPath(config_, package_or_absolute_root), defaults);
}

void SixDofForceController::setAxisSelection(const AxisSelection& axis_selection)
{
    config_.axis_selection = axis_selection;
}

void SixDofForceController::reset(const Vector6& current_pose)
{
    initializeAdmittance(current_pose);
}

void SixDofForceController::initializeAdmittance(const Vector6& pose)
{
    previous_command_ = pose;
    previous_basis_ = {1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0};

    for (int i = 0; i < 3; ++i) {
        admittance_[i].setMDK(config_.mass[i], config_.damping[i], config_.stiffness[i]);
        admittance_[i].reset(pose[i] / 1000.0);
        if (policies_[i]) policies_[i]->resetState();
    }
    for (int i = 3; i < 6; ++i) {
        admittance_[i].setMDK(config_.mass[i], config_.damping[i], config_.stiffness[i]);
        admittance_[i].reset(pose[i]);
    }

    initialized_ = true;
}

Matrix3 SixDofForceController::buildDesiredForceBasis(const Vector3& desired_force)
{
    const double eps = config_.desired_force_axis_eps;
    const Vector3 previous_q3 = column(previous_basis_, 2);
    const Vector3 q3 = normalizeOr(desired_force, previous_q3, eps);

    Vector3 q1 = projectOntoPlane(column(previous_basis_, 0), q3);
    if (norm(q1) <= eps) {
        q1 = projectOntoPlane(column(previous_basis_, 1), q3);
    }
    if (norm(q1) <= eps) {
        const Vector3 base_axes[3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
        int best_axis = 0;
        double best_score = std::fabs(dot(base_axes[0], q3));
        for (int axis = 1; axis < 3; ++axis) {
            const double score = std::fabs(dot(base_axes[axis], q3));
            if (score < best_score) {
                best_score = score;
                best_axis = axis;
            }
        }
        q1 = projectOntoPlane(base_axes[best_axis], q3);
    }

    q1 = normalizeOr(q1, {1.0, 0.0, 0.0}, eps);
    Vector3 q2 = normalizeOr(cross(q3, q1), {0.0, 1.0, 0.0}, eps);
    q1 = normalizeOr(cross(q2, q3), {1.0, 0.0, 0.0}, eps);

    Matrix3 basis{};
    setColumn(basis, 0, q1);
    setColumn(basis, 1, q2);
    setColumn(basis, 2, q3);
    previous_basis_ = basis;
    return basis;
}

Matrix3 SixDofForceController::buildControlRotation(const Vector6& current_pose,
                                                    const Vector3& desired_force)
{
    if (config_.axis_selection.mode == ForceAxisMode::BaseAxisRotation) {
        return config_.axis_selection.rotation;
    }

    if (config_.axis_selection.mode == ForceAxisMode::EndEffectorAxisRotation) {
        const Vector3 spatial_angle = {current_pose[3], current_pose[4], current_pose[5]};
        return matMul(spatialAngleToRotation(spatial_angle), config_.axis_selection.rotation);
    }

    return buildDesiredForceBasis(desired_force);
}

Vector6 SixDofForceController::computeCommand(const Vector6& desired_pose,
                                              const Vector6& current_pose,
                                              const Vector3& desired_force,
                                              const Vector6& current_force)
{
    if (!initialized_) {
        initializeAdmittance(current_pose);
    }

    const Matrix3 rotation = buildControlRotation(current_pose, desired_force);
    debug_.control_rotation = rotation;

    const Vector3 desired_position_base = {
        desired_pose[0] / 1000.0,
        desired_pose[1] / 1000.0,
        desired_pose[2] / 1000.0};
    const Vector3 current_position_base = {
        current_pose[0] / 1000.0,
        current_pose[1] / 1000.0,
        current_pose[2] / 1000.0};
    const Vector3 previous_command_base = {
        previous_command_[0] / 1000.0,
        previous_command_[1] / 1000.0,
        previous_command_[2] / 1000.0};
    const Vector3 current_force_base = {current_force[0], current_force[1], current_force[2]};

    Vector3 local_desired_position = matTVec(rotation, desired_position_base);
    const Vector3 local_current_position = matTVec(rotation, current_position_base);
    const Vector3 local_previous_command = matTVec(rotation, previous_command_base);
    Vector3 local_desired_force = config_.desired_force_is_control_frame
        ? desired_force
        : matTVec(rotation, desired_force);
    const Vector3 local_external_force = matTVec(rotation, current_force_base);

    if (config_.axis_selection.mode == ForceAxisMode::DesiredForceVector) {
        if (config_.use_force_vector_magnitude_as_axis_command) {
            local_desired_force = {0.0, 0.0, norm(desired_force)};
        } else {
            local_desired_force = matTVec(rotation, desired_force);
        }
    }

    debug_.local_desired_force = local_desired_force;
    debug_.local_external_force = local_external_force;

    Vector3 local_command{};
    for (int axis = 0; axis < 3; ++axis) {
        const bool is_force_axis = axis == kForceAxis;
        double desired_force_axis = is_force_axis ? local_desired_force[axis] : 0.0;
        const double external_force_axis = local_external_force[axis];

        const bool desired_force_active =
            is_force_axis && std::fabs(desired_force_axis) > config_.desired_force_threshold;
        const bool actual_force_active =
            is_force_axis && std::fabs(external_force_axis) > config_.actual_force_threshold;
        const bool force_control_active = desired_force_active && actual_force_active;

        const double commanded_force = (desired_force_active && !actual_force_active)
            ? signedForceHold(desired_force_axis, config_.precontact_force_hold)
            : desired_force_axis;

        if (!is_force_axis) {
            returnToInitialMDK(admittance_[axis], config_, axis);
            local_command[axis] = admittance_[axis].update(
                local_desired_position[axis],
                0.0,
                external_force_axis);
        } else if (config_.controller_mode == ForceControllerMode::GForce && force_control_active) {
            if (!policies_[axis]) {
                throw std::runtime_error("GForce policy is not configured. Set explicit_model_path or checkpoint profile and call configureGForcePolicies...");
            }
            const auto out = policies_[axis]->run(
                static_cast<float>(local_previous_command[axis]),
                static_cast<float>(local_current_position[axis]),
                static_cast<float>(desired_force_axis),
                static_cast<float>(external_force_axis));
            admittance_[axis].setMDK(out[0], policies_[axis]->appliedDamping(), 0.0);
            local_command[axis] = admittance_[axis].update(
                local_desired_position[axis], commanded_force, external_force_axis);
        } else {
            if (desired_force_active) {
                returnMDToInitialWithZeroStiffness(admittance_[axis], config_, axis);
            } else {
                returnToInitialMDK(admittance_[axis], config_, axis);
            }
            local_command[axis] = admittance_[axis].update(
                local_desired_position[axis], commanded_force, external_force_axis);
        }

        debug_.applied_mass[axis] = admittance_[axis].monitorMDK(0);
        debug_.applied_damping[axis] = admittance_[axis].monitorMDK(1);
        debug_.applied_stiffness[axis] = admittance_[axis].monitorMDK(2);
    }

    const Vector3 command_base_m = matVec(rotation, local_command);

    Vector6 command = desired_pose;
    command[0] = command_base_m[0] * 1000.0;
    command[1] = command_base_m[1] * 1000.0;
    command[2] = command_base_m[2] * 1000.0;

    for (int axis = 3; axis < 6; ++axis) {
        command[axis] = admittance_[axis].update(desired_pose[axis], 0.0, 0.0);
        debug_.applied_mass[axis] = admittance_[axis].monitorMDK(0);
        debug_.applied_damping[axis] = admittance_[axis].monitorMDK(1);
        debug_.applied_stiffness[axis] = admittance_[axis].monitorMDK(2);
    }

    previous_command_ = command;
    return command;
}

const ControllerDebugState& SixDofForceController::debugState() const
{
    return debug_;
}

const SixDofControllerConfig& SixDofForceController::config() const
{
    return config_;
}

std::string resolveModelPath(const SixDofControllerConfig& config,
                             const std::string& package_or_absolute_root)
{
    if (!config.explicit_model_path.empty()) {
        return config.explicit_model_path;
    }

    std::filesystem::path root(package_or_absolute_root);
    if (root.filename() != "gforce_control") {
        root /= "gforce_control";
    }

    return (root / config.checkpoint_root / config.checkpoint_profile / config.model_file_name).string();
}

SixDofControllerConfig loadControllerConfigFromFile(const std::string& path)
{
    SixDofControllerConfig config;

    YAML::Node root = YAML::LoadFile(path);
    YAML::Node node = root["gforce_control"] ? root["gforce_control"] : root;

    loadScalarIfPresent(node, "dt", config.dt);

    if (node["controller_mode"]) {
        const std::string value = node["controller_mode"].as<std::string>();
        if (value == "classic" || value == "classic_admittance") {
            config.controller_mode = ForceControllerMode::ClassicAdmittance;
        } else if (value == "gforce") {
            config.controller_mode = ForceControllerMode::GForce;
        } else {
            throw std::runtime_error("Unknown controller_mode: " + value);
        }
    }

    if (node["axis_mode"]) {
        const std::string value = node["axis_mode"].as<std::string>();
        if (value == "base_rotation") {
            config.axis_selection.mode = ForceAxisMode::BaseAxisRotation;
        } else if (value == "ee_rotation") {
            config.axis_selection.mode = ForceAxisMode::EndEffectorAxisRotation;
        } else if (value == "desired_force_vector") {
            config.axis_selection.mode = ForceAxisMode::DesiredForceVector;
        } else {
            throw std::runtime_error("Unknown axis_mode: " + value);
        }
    }

    loadArrayIfPresent(node, "axis_rotation", config.axis_selection.rotation);
    loadArrayIfPresent(node, "mass", config.mass);
    loadArrayIfPresent(node, "damping", config.damping);
    loadArrayIfPresent(node, "stiffness", config.stiffness);

    if (node["checkpoint"]) {
        const YAML::Node checkpoint = node["checkpoint"];
        loadScalarIfPresent(checkpoint, "root", config.checkpoint_root);
        loadScalarIfPresent(checkpoint, "profile", config.checkpoint_profile);
        loadScalarIfPresent(checkpoint, "model_file_name", config.model_file_name);
        loadScalarIfPresent(checkpoint, "explicit_model_path", config.explicit_model_path);
    }

    if (node["thresholds"]) {
        const YAML::Node thresholds = node["thresholds"];
        loadScalarIfPresent(thresholds, "desired_force", config.desired_force_threshold);
        loadScalarIfPresent(thresholds, "actual_force", config.actual_force_threshold);
        loadScalarIfPresent(thresholds, "precontact_force_hold", config.precontact_force_hold);
    }

    if (node["return_tau"]) {
        const YAML::Node tau = node["return_tau"];
        loadScalarIfPresent(tau, "mass", config.return_tau_mass);
        loadScalarIfPresent(tau, "damping", config.return_tau_damping);
        loadScalarIfPresent(tau, "stiffness", config.return_tau_stiffness);
    }

    loadScalarIfPresent(node,
                        "desired_force_axis_eps",
                        config.desired_force_axis_eps);
    loadScalarIfPresent(node,
                        "desired_force_is_control_frame",
                        config.desired_force_is_control_frame);
    loadScalarIfPresent(node,
                        "use_force_vector_magnitude_as_axis_command",
                        config.use_force_vector_magnitude_as_axis_command);

    return config;
}

}  // namespace gforce_control
