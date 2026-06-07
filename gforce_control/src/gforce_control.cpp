#include "gforce_control/gforce_control.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <torch/torch.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gforce_control {

GForceControl::GForceControl(const GForcePolicyConfig& config)
    : device_(config.device),
      dt_(config.dt),
      md_ratio_(config.md_ratio),
      force_filter_cutoff_hz_(config.force_filter_cutoff_hz),
      action_low_(config.action_low),
      action_high_(config.action_high),
      mass_min_(config.mass_min),
      mass_max_(config.mass_max),
      alpha_min_(config.alpha_min),
      alpha_max_(config.alpha_max),
      alpha_rate_up_(config.alpha_rate_up),
      alpha_rate_down_(config.alpha_rate_down)
{
    if (config.model_path.empty()) {
        throw std::invalid_argument("GForcePolicyConfig::model_path is empty.");
    }

    torch::set_num_threads(std::max(1, config.threads));
    module_ = torch::jit::load(config.model_path, device_);
    module_.eval();

    updateForceFilterAlpha(dt_);
    resetState();
}

void GForceControl::setSamplingTime(float dt)
{
    dt_ = std::max(1e-6f, dt);
    updateForceFilterAlpha(dt_);
}

void GForceControl::setMDRatio(float md_ratio)
{
    md_ratio_ = safeFloat(md_ratio, md_ratio_);
    if (!std::isfinite(md_ratio_) || md_ratio_ <= 0.0f) {
        md_ratio_ = 1000.0f;
    }
}

void GForceControl::setAlphaRateLimits(float rate_up, float rate_down)
{
    alpha_rate_up_ = std::max(0.0f, safeFloat(rate_up, alpha_rate_up_));
    alpha_rate_down_ = std::max(0.0f, safeFloat(rate_down, alpha_rate_down_));
}

void GForceControl::setPhysicalBounds(float mass_min,
                                      float mass_max,
                                      float alpha_min,
                                      float alpha_max)
{
    if (mass_min > mass_max) std::swap(mass_min, mass_max);
    if (alpha_min > alpha_max) std::swap(alpha_min, alpha_max);

    mass_min_ = mass_min;
    mass_max_ = mass_max;
    alpha_min_ = alpha_min;
    alpha_max_ = alpha_max;

    mass_act_ = clamp(mass_act_, mass_min_, mass_max_);
    alpha_act_ = clamp(alpha_act_, alpha_min_, alpha_max_);
}

void GForceControl::updateForceFilterAlpha(float dt)
{
    const float dts = std::max(1e-6f, dt);
    force_filter_alpha_ = std::exp(-2.0f * static_cast<float>(M_PI) * force_filter_cutoff_hz_ * dts);
    if (!std::isfinite(force_filter_alpha_)) force_filter_alpha_ = 0.0f;
    force_filter_alpha_ = clamp(force_filter_alpha_, 0.0f, 1.0f);
}

void GForceControl::resetState()
{
    has_first_sample_ = false;
    has_second_sample_ = false;
    command_prev1_ = 0.0f;
    command_prev2_ = 0.0f;
    measured_prev1_ = 0.0f;
    measured_velocity_prev1_ = 0.0f;
    filtered_force_prev_ = 0.0f;

    mass_act_ = 0.5f * (mass_min_ + mass_max_);
    alpha_act_ = 0.5f * (alpha_min_ + alpha_max_);
    mass_act_ = clamp(mass_act_, mass_min_, mass_max_);
    alpha_act_ = clamp(alpha_act_, alpha_min_, alpha_max_);

    last_network_action_ = {0.0f, 0.0f};
    last_alpha_command_abs_ = alpha_act_;
}

float GForceControl::clamp(float value, float lower, float upper)
{
    return std::min(std::max(value, lower), upper);
}

float GForceControl::safeFloat(float value, float fallback)
{
    return std::isfinite(value) ? value : fallback;
}

std::array<float, 2> GForceControl::squashToBoundsTanh(const std::array<float, 2>& raw,
                                                       const std::array<float, 2>& low,
                                                       const std::array<float, 2>& high,
                                                       float temperature)
{
    std::array<float, 2> output;
    const float temp = std::max(1e-6f, temperature);

    for (int i = 0; i < 2; ++i) {
        const float z = std::tanh(raw[i] / temp);
        output[i] = low[i] + 0.5f * (z + 1.0f) * (high[i] - low[i]);
        output[i] = clamp(output[i], low[i], high[i]);
        if (!std::isfinite(output[i])) {
            output[i] = 0.5f * (low[i] + high[i]);
        }
    }

    return output;
}

float GForceControl::lpfUpdate(float previous_filtered, float raw) const
{
    const float y = force_filter_alpha_ * previous_filtered + (1.0f - force_filter_alpha_) * raw;
    return std::isfinite(y) ? y : raw;
}

float GForceControl::updateMassActuator(float previous_mass, float delta_mass_command) const
{
    float next = previous_mass + delta_mass_command;
    next = clamp(next, mass_min_, mass_max_);
    if (!std::isfinite(next)) {
        next = clamp(previous_mass, mass_min_, mass_max_);
    }
    return next;
}

float GForceControl::alphaCommandFromIncrement(float previous_alpha, float delta_alpha_command) const
{
    float command = previous_alpha + delta_alpha_command;
    command = clamp(command, alpha_min_, alpha_max_);
    if (!std::isfinite(command)) {
        command = clamp(previous_alpha, alpha_min_, alpha_max_);
    }
    return command;
}

float GForceControl::updateAlphaActuator(float previous_alpha, float alpha_command_abs) const
{
    const float dt = std::max(1e-8f, dt_);
    const float err = alpha_command_abs - previous_alpha;
    float delta = 0.0f;

    if (err >= 0.0f) {
        const float step_limit = std::max(0.0f, alpha_rate_up_) * dt;
        delta = std::min(err, step_limit);
    } else {
        const float step_limit = std::max(0.0f, alpha_rate_down_) * dt;
        delta = std::max(err, -step_limit);
    }

    float next = previous_alpha + delta;
    next = clamp(next, alpha_min_, alpha_max_);
    if (!std::isfinite(next)) {
        next = clamp(previous_alpha, alpha_min_, alpha_max_);
    }
    return next;
}

std::vector<double> GForceControl::run(float command_position,
                                       float measured_position,
                                       float desired_force,
                                       float external_force)
{
    torch::NoGradGuard no_grad;

    command_position = safeFloat(command_position, 0.0f);
    measured_position = safeFloat(measured_position, 0.0f);
    desired_force = safeFloat(desired_force, 0.0f);
    external_force = safeFloat(external_force, 0.0f);

    float measured_velocity = 0.0f;
    float measured_acceleration = 0.0f;
    float command_acceleration = 0.0f;

    if (!has_first_sample_) {
        command_prev1_ = command_position;
        command_prev2_ = command_position;
        measured_prev1_ = measured_position;
        measured_velocity_prev1_ = 0.0f;
        has_first_sample_ = true;
        filtered_force_prev_ = external_force;
    } else {
        measured_velocity = (measured_position - measured_prev1_) / std::max(1e-6f, dt_);

        if (!has_second_sample_) {
            measured_acceleration = 0.0f;
            command_acceleration = 0.0f;
            measured_velocity_prev1_ = measured_velocity;
            command_prev2_ = command_prev1_;
            command_prev1_ = command_position;
            measured_prev1_ = measured_position;
            has_second_sample_ = true;
        } else {
            measured_acceleration =
                (measured_velocity - measured_velocity_prev1_) / std::max(1e-6f, dt_);
            command_acceleration =
                (command_position - 2.0f * command_prev1_ + command_prev2_) /
                (std::max(1e-6f, dt_) * std::max(1e-6f, dt_));

            measured_velocity_prev1_ = measured_velocity;
            measured_prev1_ = measured_position;
            command_prev2_ = command_prev1_;
            command_prev1_ = command_position;
        }
    }

    measured_velocity = safeFloat(measured_velocity, 0.0f);
    measured_acceleration = safeFloat(measured_acceleration, 0.0f);
    command_acceleration = safeFloat(command_acceleration, 0.0f);

    const float filtered_force = safeFloat(lpfUpdate(filtered_force_prev_, external_force), external_force);

    auto state = torch::tensor(
                     {command_acceleration,
                      measured_velocity,
                      measured_acceleration,
                      desired_force - filtered_force},
                     torch::TensorOptions().dtype(torch::kFloat32))
                     .unsqueeze(0)
                     .to(device_);

    torch::IValue value = module_.get_method("act")({state});
    torch::Tensor mu;
    if (value.isTensor()) {
        mu = value.toTensor();
    } else if (value.isTuple()) {
        const auto& elements = value.toTuple()->elements();
        for (const auto& element : elements) {
            if (element.isTensor()) {
                mu = element.toTensor();
                break;
            }
        }
        if (!mu.defined()) {
            throw std::runtime_error("act() returned a tuple without a Tensor.");
        }
    } else {
        throw std::runtime_error("Unsupported return type from act().");
    }

    mu = mu.to(torch::kCPU).contiguous().reshape({-1});
    TORCH_CHECK(mu.numel() >= 2, "act() must return at least 2 scalars, got ", mu.sizes());

    const std::array<float, 2> raw_action{
        safeFloat(mu[0].item<float>(), 0.0f),
        safeFloat(mu[1].item<float>(), 0.0f),
    };

    const auto action = squashToBoundsTanh(raw_action, action_low_, action_high_, 1.0f);
    const float delta_mass_command = safeFloat(action[0], 0.0f);
    const float delta_alpha_command = safeFloat(action[1], 0.0f);

    mass_act_ = updateMassActuator(mass_act_, delta_mass_command);

    const float alpha_command_abs = alphaCommandFromIncrement(alpha_act_, delta_alpha_command);
    alpha_act_ = updateAlphaActuator(alpha_act_, alpha_command_abs);

    last_network_action_ = {delta_mass_command, delta_alpha_command};
    last_alpha_command_abs_ = alpha_command_abs;
    filtered_force_prev_ = filtered_force;

    return {static_cast<double>(mass_act_), static_cast<double>(alpha_act_)};
}

std::array<float, 2> GForceControl::appliedMassAlpha() const
{
    return {mass_act_, alpha_act_};
}

float GForceControl::appliedDamping() const
{
    const float damping = mass_act_ * alpha_act_ * md_ratio_;
    return std::isfinite(damping) ? damping : 0.0f;
}

std::array<float, 2> GForceControl::lastNetworkAction() const
{
    return last_network_action_;
}

float GForceControl::lastAlphaCommandAbs() const
{
    return last_alpha_command_abs_;
}

float GForceControl::filteredExternalForce() const
{
    return filtered_force_prev_;
}

}  // namespace gforce_control
