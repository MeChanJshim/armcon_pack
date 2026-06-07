#pragma once

#include <array>
#include <memory>
#include <string>

#include "gforce_control/admittance_control.hpp"
#include "gforce_control/gforce_control.hpp"
#include "gforce_control/types.hpp"

namespace gforce_control {

class SixDofForceController {
public:
    SixDofForceController();
    explicit SixDofForceController(const SixDofControllerConfig& config);

    void configure(const SixDofControllerConfig& config);
    void configureGForcePolicies(const GForcePolicyConfig& policy_config);
    void configureGForcePoliciesFromModelPath(const std::string& model_path,
                                              const GForcePolicyConfig& defaults = GForcePolicyConfig{});
    void configureGForcePoliciesFromCheckpoint(const std::string& package_or_absolute_root,
                                               const GForcePolicyConfig& defaults = GForcePolicyConfig{});

    void setAxisSelection(const AxisSelection& axis_selection);
    void reset(const Vector6& current_pose);

    Vector6 computeCommand(const Vector6& desired_pose,
                           const Vector6& current_pose,
                           const Vector3& desired_force,
                           const Vector6& current_force);

    const ControllerDebugState& debugState() const;
    const SixDofControllerConfig& config() const;

private:
    SixDofControllerConfig config_;
    AdmittanceControl admittance_[6];
    std::unique_ptr<GForceControl> policies_[3];
    Vector6 previous_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Matrix3 previous_basis_ = {1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
    bool initialized_ = false;
    ControllerDebugState debug_;

    Matrix3 buildControlRotation(const Vector6& current_pose, const Vector3& desired_force);
    Matrix3 buildDesiredForceBasis(const Vector3& desired_force);
    void initializeAdmittance(const Vector6& pose);
};

SixDofControllerConfig loadControllerConfigFromFile(const std::string& path);
std::string resolveModelPath(const SixDofControllerConfig& config,
                             const std::string& package_or_absolute_root);

}  // namespace gforce_control
