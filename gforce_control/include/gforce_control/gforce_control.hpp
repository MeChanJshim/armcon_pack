#pragma once

#include <array>
#include <string>
#include <vector>

#include <torch/script.h>

namespace gforce_control {

struct GForcePolicyConfig {
    std::string model_path;
    int threads = 1;
    c10::Device device = c10::Device(c10::kCPU);
    float dt = 0.008f;
    float md_ratio = 1000.0f;
    float force_filter_cutoff_hz = 50.0f;
    std::array<float, 2> action_low = {-0.25f, -0.25f};
    std::array<float, 2> action_high = {0.25f, 0.25f};
    float mass_min = 0.5f;
    float mass_max = 5.0f;
    float alpha_min = 0.5f;
    float alpha_max = 3.0f;
    float alpha_rate_up = 4.0f;
    float alpha_rate_down = 4.0f;
};

class GForceControl {
public:
    explicit GForceControl(const GForcePolicyConfig& config);

    std::vector<double> run(float command_position,
                            float measured_position,
                            float desired_force,
                            float external_force);

    void setSamplingTime(float dt);
    void resetState();
    void setMDRatio(float md_ratio);
    void setAlphaRateLimits(float rate_up, float rate_down);
    void setPhysicalBounds(float mass_min, float mass_max, float alpha_min, float alpha_max);

    std::array<float, 2> appliedMassAlpha() const;
    float appliedDamping() const;
    std::array<float, 2> lastNetworkAction() const;
    float lastAlphaCommandAbs() const;
    float filteredExternalForce() const;

private:
    c10::Device device_;
    torch::jit::script::Module module_;

    float dt_ = 0.008f;
    float md_ratio_ = 1000.0f;
    float force_filter_cutoff_hz_ = 50.0f;
    float force_filter_alpha_ = 0.0f;

    std::array<float, 2> action_low_;
    std::array<float, 2> action_high_;

    float mass_min_ = 0.5f;
    float mass_max_ = 5.0f;
    float alpha_min_ = 0.5f;
    float alpha_max_ = 3.0f;

    float alpha_rate_up_ = 4.0f;
    float alpha_rate_down_ = 4.0f;

    bool has_first_sample_ = false;
    bool has_second_sample_ = false;
    float command_prev1_ = 0.0f;
    float command_prev2_ = 0.0f;
    float measured_prev1_ = 0.0f;
    float measured_velocity_prev1_ = 0.0f;
    float filtered_force_prev_ = 0.0f;

    float mass_act_ = 0.0f;
    float alpha_act_ = 0.0f;

    std::array<float, 2> last_network_action_{0.0f, 0.0f};
    float last_alpha_command_abs_ = 0.0f;

    void updateForceFilterAlpha(float dt);
    float lpfUpdate(float previous_filtered, float raw) const;
    float updateMassActuator(float previous_mass, float delta_mass_command) const;
    float alphaCommandFromIncrement(float previous_alpha, float delta_alpha_command) const;
    float updateAlphaActuator(float previous_alpha, float alpha_command_abs) const;

    static float clamp(float value, float lower, float upper);
    static float safeFloat(float value, float fallback = 0.0f);
    static std::array<float, 2> squashToBoundsTanh(const std::array<float, 2>& raw,
                                                   const std::array<float, 2>& low,
                                                   const std::array<float, 2>& high,
                                                   float temperature = 1.0f);
};

}  // namespace gforce_control
