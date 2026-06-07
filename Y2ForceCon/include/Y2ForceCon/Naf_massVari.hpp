#pragma once

#include <torch/script.h>
#include <array>
#include <string>
#include <vector>

class RL_Naf_massVari {
public:
    RL_Naf_massVari(const std::string& model_path,
                    int threads,
                    c10::Device device,
                    float dt = 0.01f,
                    float md_ratio = 1000.0f,
                    float alpha_fixed = 1.0f,
                    float fc_fext = 50.0f,
                    std::array<float, 1> action_low = {0.5f},
                    std::array<float, 1> action_high = {5.0f},
                    float mass_min = 0.5f,
                    float mass_max = 5.0f);

    // raw inputs
    // xc : command position (reference position)
    // x  : measured position
    // Fd : desired force
    // Env_Fext : measured external force (raw)
    //
    // returns: [mass, alpha_fixed]
    std::vector<double> run(float xc, float x, float Fd, float Env_Fext);

    void set_sampling_time(float dt);
    void reset_state();

    void set_md_ratio(float md_ratio);
    void set_alpha_fixed(float alpha_fixed);
    void set_mass_bounds(float mass_min, float mass_max);

    float get_applied_mass() const;
    float get_applied_alpha() const;
    float get_applied_damping() const;
    float get_last_network_action() const;
    float get_filtered_fext() const;

private:
    c10::Device device_;
    torch::jit::script::Module module_;

    float dt_ = 0.01f;
    float md_ratio_ = 1000.0f;
    float alpha_fixed_ = 1.0f;
    float fc_fext_ = 50.0f;
    float alpha_lp_ = 0.0f;

    std::array<float, 1> action_low_;
    std::array<float, 1> action_high_;

    float mass_min_ = 0.5f;
    float mass_max_ = 5.0f;

    bool has1_ = false;
    bool has2_ = false;
    float xc_prev1_ = 0.0f;
    float xc_prev2_ = 0.0f;
    float x_prev1_ = 0.0f;
    float x_dot_prev1_ = 0.0f;

    float fext_filt_prev_ = 0.0f;

    float mass_act_ = 0.0f;
    float last_a_net_ = 0.0f;

private:
    void update_alpha_lp(float dt);

    static float clampf(float v, float lo, float hi);
    static float safe_float(float v, float fallback = 0.0f);
    static float squash_to_bounds_tanh(float raw, float low, float high, float temp = 1.0f);

    float lpf_update(float prev_filt, float raw) const;
};
