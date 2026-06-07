#pragma once

#include <torch/script.h>
#include <array>
#include <string>
#include <vector>

class RL_Naf_mdGradi {
public:
    RL_Naf_mdGradi(const std::string& model_path,
                   int threads,
                   c10::Device device,
                   float dt = 0.01f,
                   float md_ratio = 1000.0f,
                   float fc_fext = 50.0f,
                   std::array<float, 2> action_low = {-0.25f, -0.25f},
                   std::array<float, 2> action_high = {+0.25f, +0.25f},
                   float mass_min = 0.5f,
                   float mass_max = 5.0f,
                   float alpha_min = 0.5f,
                   float alpha_max = 3.0f,
                   float alpha_rate_up = 4.0f,
                   float alpha_rate_down = 4.0f);

    // raw inputs
    // xc : command position (reference position)
    // x  : measured position
    // Fd : desired force
    // Env_Fext : measured external force (raw)
    //
    // returns: [mass_act, alpha_act]
    std::vector<double> run(float xc, float x, float Fd, float Env_Fext);

    void set_sampling_time(float dt);
    void reset_state();

    void set_md_ratio(float md_ratio);
    void set_alpha_rate_limits(float rate_up, float rate_down);
    void set_physical_bounds(float mass_min, float mass_max, float alpha_min, float alpha_max);

    std::array<float, 2> get_applied_mass_alpha() const;
    float get_applied_damping() const;
    std::array<float, 2> get_last_network_action() const;
    float get_last_alpha_cmd_abs() const;
    float get_filtered_fext() const;

private:
    c10::Device device_;
    torch::jit::script::Module module_;

    float dt_ = 0.01f;
    float md_ratio_ = 1000.0f;
    float fc_fext_ = 50.0f;
    float alpha_lp_ = 0.0f;

    std::array<float, 2> action_low_;
    std::array<float, 2> action_high_;

    float mass_min_ = 0.5f;
    float mass_max_ = 5.0f;
    float alpha_min_ = 0.5f;
    float alpha_max_ = 3.0f;

    float alpha_rate_up_ = 4.0f;
    float alpha_rate_down_ = 4.0f;

    bool has1_ = false;
    bool has2_ = false;
    float xc_prev1_ = 0.0f;
    float xc_prev2_ = 0.0f;
    float x_prev1_ = 0.0f;
    float x_dot_prev1_ = 0.0f;

    float fext_filt_prev_ = 0.0f;

    float mass_act_ = 0.0f;
    float alpha_act_ = 0.0f;

    std::array<float, 2> last_a_net_{0.0f, 0.0f};
    float last_alpha_cmd_abs_ = 0.0f;

private:
    void update_alpha_lp(float dt);

    static float clampf(float v, float lo, float hi);
    static float safe_float(float v, float fallback = 0.0f);

    static std::array<float, 2> squash_to_bounds_tanh(const std::array<float, 2>& raw,
                                                      const std::array<float, 2>& low,
                                                      const std::array<float, 2>& high,
                                                      float temp = 1.0f);

    float lpf_update(float prev_filt, float raw) const;
    float update_mass_actuator_incremental(float mass_prev, float delta_mass_cmd) const;
    float alpha_cmd_from_increment(float alpha_prev, float delta_alpha_cmd) const;
    float update_alpha_actuator(float alpha_prev, float alpha_cmd_abs) const;
};
