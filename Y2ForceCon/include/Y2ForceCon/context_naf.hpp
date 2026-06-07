#pragma once

#include <torch/script.h>
#include <vector>
#include <string>
#include <array>

class RL_ContextNAF {
public:
    RL_ContextNAF(const std::string& model_path,
                  int threads,
                  c10::Device device,
                  float dt = 0.01f,
                  float md_ratio = 1000.0f,
                  float fc_fext = 50.0f,
                  std::array<float,2> action_low = {0.5f, 0.5f},
                  std::array<float,2> action_high = {3.0f, 5.0f});

    // raw inputs
    // xc : command position (or reference position)
    // x  : measured position
    // Fd : desired force
    // Env_Fext : measured external force (raw)
    //
    // returns: [mass, alpha] (already tanh-squashed + bounded)
    std::vector<double> run(float xc, float x, float Fd, float Env_Fext);

    void set_sampling_time(float dt);
    void reset_state();

private:
    // ---- Torch ----
    c10::Device device_;
    torch::jit::script::Module module_;

    // ---- time / params ----
    float dt_ = 0.01f;
    float md_ratio_ = 1000.0f;
    float fc_fext_ = 50.0f;
    float alpha_lp_ = 0.0f;

    std::array<float,2> action_low_;
    std::array<float,2> action_high_;
    std::array<float,2> action_mid_;

    // ---- numerical derivative state (for s = [xc_ddot, x_dot, x_ddot, (Fd - Fext_filt)] ) ----
    bool  has1_ = false, has2_ = false;
    float xc_prev1_ = 0.0f, xc_prev2_ = 0.0f; // for xc_ddot (2nd difference)
    float x_prev1_  = 0.0f;                   // for x_dot
    float x_dot_prev1_ = 0.0f;                // for x_ddot

    // ---- residual bookkeeping (for q_t) ----
    float prev_er_ = 0.0f;
    std::array<float,2> prev_a_;      // a_{t-1} in (mass, alpha) after squashing
    float md_prev_ = 0.0f;            // applied mass at t-1
    float dd_prev_ = 0.0f;            // applied damping at t-1 (D)
    float fext_filt_prev_ = 0.0f;     // previous filtered Fext

    // ---- GRU hidden state ----
    bool has_hidden_ = false;
    torch::Tensor h_;                 // [1,1,h_dim] on device_

private:
    void update_alpha_lp(float dt);
    torch::Tensor ensure_hidden();

    static float clampf(float v, float lo, float hi);
    static std::array<float,2> squash_to_bounds_tanh(const std::array<float,2>& raw,
                                                     const std::array<float,2>& low,
                                                     const std::array<float,2>& high);
};
