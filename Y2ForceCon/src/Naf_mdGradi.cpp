#include "Y2ForceCon/Naf_mdGradi.hpp"

#include <torch/torch.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

RL_Naf_mdGradi::RL_Naf_mdGradi(const std::string& model_path,
                               int threads,
                               c10::Device device,
                               float dt,
                               float md_ratio,
                               float fc_fext,
                               std::array<float, 2> action_low,
                               std::array<float, 2> action_high,
                               float mass_min,
                               float mass_max,
                               float alpha_min,
                               float alpha_max,
                               float alpha_rate_up,
                               float alpha_rate_down)
    : device_(device),
      dt_(dt),
      md_ratio_(md_ratio),
      fc_fext_(fc_fext),
      action_low_(action_low),
      action_high_(action_high),
      mass_min_(mass_min),
      mass_max_(mass_max),
      alpha_min_(alpha_min),
      alpha_max_(alpha_max),
      alpha_rate_up_(alpha_rate_up),
      alpha_rate_down_(alpha_rate_down) {
    torch::set_num_threads(std::max(1, threads));
    module_ = torch::jit::load(model_path, device_);
    module_.eval();

    update_alpha_lp(dt_);
    reset_state();
}

void RL_Naf_mdGradi::set_sampling_time(float dt) {
    dt_ = std::max(1e-6f, dt);
    update_alpha_lp(dt_);
}

void RL_Naf_mdGradi::set_md_ratio(float md_ratio) {
    md_ratio_ = safe_float(md_ratio, md_ratio_);
    if (!std::isfinite(md_ratio_) || md_ratio_ <= 0.0f) {
        md_ratio_ = 1000.0f;
    }
}

void RL_Naf_mdGradi::set_alpha_rate_limits(float rate_up, float rate_down) {
    alpha_rate_up_ = std::max(0.0f, safe_float(rate_up, alpha_rate_up_));
    alpha_rate_down_ = std::max(0.0f, safe_float(rate_down, alpha_rate_down_));
}

void RL_Naf_mdGradi::set_physical_bounds(float mass_min, float mass_max, float alpha_min, float alpha_max) {
    if (mass_min > mass_max) std::swap(mass_min, mass_max);
    if (alpha_min > alpha_max) std::swap(alpha_min, alpha_max);

    mass_min_ = mass_min;
    mass_max_ = mass_max;
    alpha_min_ = alpha_min;
    alpha_max_ = alpha_max;

    mass_act_ = clampf(mass_act_, mass_min_, mass_max_);
    alpha_act_ = clampf(alpha_act_, alpha_min_, alpha_max_);
}

void RL_Naf_mdGradi::update_alpha_lp(float dt) {
    const float dts = std::max(1e-6f, dt);
    alpha_lp_ = std::exp(-2.0f * static_cast<float>(M_PI) * fc_fext_ * dts);
    if (!std::isfinite(alpha_lp_)) alpha_lp_ = 0.0f;
    alpha_lp_ = clampf(alpha_lp_, 0.0f, 1.0f);
}

void RL_Naf_mdGradi::reset_state() {
    has1_ = false;
    has2_ = false;
    xc_prev1_ = 0.0f;
    xc_prev2_ = 0.0f;
    x_prev1_ = 0.0f;
    x_dot_prev1_ = 0.0f;

    fext_filt_prev_ = 0.0f;

    mass_act_ = 0.5f * (mass_min_ + mass_max_);
    alpha_act_ = 0.5f * (alpha_min_ + alpha_max_);
    mass_act_ = clampf(mass_act_, mass_min_, mass_max_);
    alpha_act_ = clampf(alpha_act_, alpha_min_, alpha_max_);

    last_a_net_ = {0.0f, 0.0f};
    last_alpha_cmd_abs_ = alpha_act_;
}

float RL_Naf_mdGradi::clampf(float v, float lo, float hi) {
    return std::min(std::max(v, lo), hi);
}

float RL_Naf_mdGradi::safe_float(float v, float fallback) {
    return std::isfinite(v) ? v : fallback;
}

std::array<float, 2> RL_Naf_mdGradi::squash_to_bounds_tanh(const std::array<float, 2>& raw,
                                                           const std::array<float, 2>& low,
                                                           const std::array<float, 2>& high,
                                                           float temp) {
    std::array<float, 2> out;
    const float T = std::max(1e-6f, temp);

    for (int i = 0; i < 2; ++i) {
        const float z = std::tanh(raw[i] / T);
        out[i] = low[i] + 0.5f * (z + 1.0f) * (high[i] - low[i]);
        out[i] = clampf(out[i], low[i], high[i]);
        if (!std::isfinite(out[i])) {
            out[i] = 0.5f * (low[i] + high[i]);
        }
    }
    return out;
}

float RL_Naf_mdGradi::lpf_update(float prev_filt, float raw) const {
    const float y = alpha_lp_ * prev_filt + (1.0f - alpha_lp_) * raw;
    return std::isfinite(y) ? y : raw;
}

float RL_Naf_mdGradi::update_mass_actuator_incremental(float mass_prev, float delta_mass_cmd) const {
    float m_new = mass_prev + delta_mass_cmd;
    m_new = clampf(m_new, mass_min_, mass_max_);
    if (!std::isfinite(m_new)) {
        m_new = clampf(mass_prev, mass_min_, mass_max_);
    }
    return m_new;
}

float RL_Naf_mdGradi::alpha_cmd_from_increment(float alpha_prev, float delta_alpha_cmd) const {
    float a_cmd = alpha_prev + delta_alpha_cmd;
    a_cmd = clampf(a_cmd, alpha_min_, alpha_max_);
    if (!std::isfinite(a_cmd)) {
        a_cmd = clampf(alpha_prev, alpha_min_, alpha_max_);
    }
    return a_cmd;
}

float RL_Naf_mdGradi::update_alpha_actuator(float alpha_prev, float alpha_cmd_abs) const {
    const float dt = std::max(1e-8f, dt_);
    const float err = alpha_cmd_abs - alpha_prev;
    float da = 0.0f;

    if (err >= 0.0f) {
        const float step_lim = std::max(0.0f, alpha_rate_up_) * dt;
        da = std::min(err, step_lim);
    } else {
        const float step_lim = std::max(0.0f, alpha_rate_down_) * dt;
        da = std::max(err, -step_lim);
    }

    float a_new = alpha_prev + da;
    a_new = clampf(a_new, alpha_min_, alpha_max_);
    if (!std::isfinite(a_new)) {
        a_new = clampf(alpha_prev, alpha_min_, alpha_max_);
    }
    return a_new;
}

std::vector<double> RL_Naf_mdGradi::run(float xc, float x, float Fd, float Env_Fext) {
    torch::NoGradGuard _ng;

    xc = safe_float(xc, 0.0f);
    x = safe_float(x, 0.0f);
    Fd = safe_float(Fd, 0.0f);
    Env_Fext = safe_float(Env_Fext, 0.0f);

    float x_dot = 0.0f;
    float x_ddot = 0.0f;
    float xc_ddot = 0.0f;

    if (!has1_) {
        xc_prev1_ = xc;
        xc_prev2_ = xc;
        x_prev1_ = x;
        x_dot_prev1_ = 0.0f;
        has1_ = true;

        fext_filt_prev_ = Env_Fext;
    } else {
        x_dot = (x - x_prev1_) / std::max(1e-6f, dt_);

        if (!has2_) {
            x_ddot = 0.0f;
            xc_ddot = 0.0f;

            x_dot_prev1_ = x_dot;
            xc_prev2_ = xc_prev1_;
            xc_prev1_ = xc;
            x_prev1_ = x;
            has2_ = true;
        } else {
            x_ddot = (x_dot - x_dot_prev1_) / std::max(1e-6f, dt_);
            xc_ddot = (xc - 2.0f * xc_prev1_ + xc_prev2_) / (std::max(1e-6f, dt_) * std::max(1e-6f, dt_));

            x_dot_prev1_ = x_dot;
            x_prev1_ = x;
            xc_prev2_ = xc_prev1_;
            xc_prev1_ = xc;
        }
    }

    x_dot = safe_float(x_dot, 0.0f);
    x_ddot = safe_float(x_ddot, 0.0f);
    xc_ddot = safe_float(xc_ddot, 0.0f);

    const float fext_filt = safe_float(lpf_update(fext_filt_prev_, Env_Fext), Env_Fext);

    auto s = torch::tensor(
                 {xc_ddot, x_dot, x_ddot, (Fd - fext_filt)},
                 torch::TensorOptions().dtype(torch::kFloat32))
                 .unsqueeze(0)
                 .to(device_);

    torch::IValue iv = module_.get_method("act")({s});
    torch::Tensor mu;
    if (iv.isTensor()) {
        mu = iv.toTensor();
    } else if (iv.isTuple()) {
        const auto& elems = iv.toTuple()->elements();
        for (const auto& e : elems) {
            if (e.isTensor()) {
                mu = e.toTensor();
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

    const std::array<float, 2> mu_raw{
        safe_float(mu[0].item<float>(), 0.0f),
        safe_float(mu[1].item<float>(), 0.0f),
    };

    const auto a_net = squash_to_bounds_tanh(mu_raw, action_low_, action_high_, 1.0f);
    const float delta_mass_cmd = safe_float(a_net[0], 0.0f);
    const float delta_alpha_cmd = safe_float(a_net[1], 0.0f);

    mass_act_ = update_mass_actuator_incremental(mass_act_, delta_mass_cmd);

    const float alpha_cmd_abs = alpha_cmd_from_increment(alpha_act_, delta_alpha_cmd);
    alpha_act_ = update_alpha_actuator(alpha_act_, alpha_cmd_abs);

    last_a_net_ = {delta_mass_cmd, delta_alpha_cmd};
    last_alpha_cmd_abs_ = alpha_cmd_abs;
    fext_filt_prev_ = fext_filt;

    std::vector<double> out(2);
    out[0] = static_cast<double>(mass_act_);
    out[1] = static_cast<double>(alpha_act_);
    return out;
}

std::array<float, 2> RL_Naf_mdGradi::get_applied_mass_alpha() const {
    return {mass_act_, alpha_act_};
}

float RL_Naf_mdGradi::get_applied_damping() const {
    const float D = mass_act_ * alpha_act_ * md_ratio_;
    return std::isfinite(D) ? D : 0.0f;
}

std::array<float, 2> RL_Naf_mdGradi::get_last_network_action() const {
    return last_a_net_;
}

float RL_Naf_mdGradi::get_last_alpha_cmd_abs() const {
    return last_alpha_cmd_abs_;
}

float RL_Naf_mdGradi::get_filtered_fext() const {
    return fext_filt_prev_;
}
