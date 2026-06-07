#include "Y2ForceCon/Naf_massVari.hpp"

#include <torch/torch.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

RL_Naf_massVari::RL_Naf_massVari(const std::string& model_path,
                                 int threads,
                                 c10::Device device,
                                 float dt,
                                 float md_ratio,
                                 float alpha_fixed,
                                 float fc_fext,
                                 std::array<float, 1> action_low,
                                 std::array<float, 1> action_high,
                                 float mass_min,
                                 float mass_max)
    : device_(device),
      dt_(dt),
      md_ratio_(md_ratio),
      alpha_fixed_(alpha_fixed),
      fc_fext_(fc_fext),
      action_low_(action_low),
      action_high_(action_high),
      mass_min_(mass_min),
      mass_max_(mass_max) {
    torch::set_num_threads(std::max(1, threads));
    module_ = torch::jit::load(model_path, device_);
    module_.eval();

    update_alpha_lp(dt_);
    reset_state();
}

void RL_Naf_massVari::set_sampling_time(float dt) {
    dt_ = std::max(1e-6f, dt);
    update_alpha_lp(dt_);
}

void RL_Naf_massVari::set_md_ratio(float md_ratio) {
    md_ratio_ = safe_float(md_ratio, md_ratio_);
    if (!std::isfinite(md_ratio_) || md_ratio_ <= 0.0f) {
        md_ratio_ = 1000.0f;
    }
}

void RL_Naf_massVari::set_alpha_fixed(float alpha_fixed) {
    alpha_fixed_ = safe_float(alpha_fixed, alpha_fixed_);
    if (!std::isfinite(alpha_fixed_) || alpha_fixed_ <= 0.0f) {
        alpha_fixed_ = 1.0f;
    }
}

void RL_Naf_massVari::set_mass_bounds(float mass_min, float mass_max) {
    if (mass_min > mass_max) std::swap(mass_min, mass_max);
    mass_min_ = mass_min;
    mass_max_ = mass_max;
    mass_act_ = clampf(mass_act_, mass_min_, mass_max_);
}

void RL_Naf_massVari::update_alpha_lp(float dt) {
    const float dts = std::max(1e-6f, dt);
    alpha_lp_ = std::exp(-2.0f * static_cast<float>(M_PI) * fc_fext_ * dts);
    if (!std::isfinite(alpha_lp_)) alpha_lp_ = 0.0f;
    alpha_lp_ = clampf(alpha_lp_, 0.0f, 1.0f);
}

void RL_Naf_massVari::reset_state() {
    has1_ = false;
    has2_ = false;
    xc_prev1_ = 0.0f;
    xc_prev2_ = 0.0f;
    x_prev1_ = 0.0f;
    x_dot_prev1_ = 0.0f;

    fext_filt_prev_ = 0.0f;

    mass_act_ = 0.5f * (mass_min_ + mass_max_);
    mass_act_ = clampf(mass_act_, mass_min_, mass_max_);
    last_a_net_ = mass_act_;
}

float RL_Naf_massVari::clampf(float v, float lo, float hi) {
    return std::min(std::max(v, lo), hi);
}

float RL_Naf_massVari::safe_float(float v, float fallback) {
    return std::isfinite(v) ? v : fallback;
}

float RL_Naf_massVari::squash_to_bounds_tanh(float raw, float low, float high, float temp) {
    const float T = std::max(1e-6f, temp);
    const float z = std::tanh(raw / T);
    float out = low + 0.5f * (z + 1.0f) * (high - low);
    out = clampf(out, low, high);
    if (!std::isfinite(out)) {
        out = 0.5f * (low + high);
    }
    return out;
}

float RL_Naf_massVari::lpf_update(float prev_filt, float raw) const {
    const float y = alpha_lp_ * prev_filt + (1.0f - alpha_lp_) * raw;
    return std::isfinite(y) ? y : raw;
}

std::vector<double> RL_Naf_massVari::run(float xc, float x, float Fd, float Env_Fext) {
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
    TORCH_CHECK(mu.numel() >= 1, "act() must return at least 1 scalar, got ", mu.sizes());

    const float mu_raw = safe_float(mu[0].item<float>(), 0.0f);
    const float mass = squash_to_bounds_tanh(mu_raw, action_low_[0], action_high_[0], 1.0f);

    mass_act_ = clampf(mass, mass_min_, mass_max_);
    last_a_net_ = mass_act_;
    fext_filt_prev_ = fext_filt;

    std::vector<double> out(2);
    out[0] = static_cast<double>(mass_act_);
    out[1] = static_cast<double>(alpha_fixed_);
    return out;
}

float RL_Naf_massVari::get_applied_mass() const {
    return mass_act_;
}

float RL_Naf_massVari::get_applied_alpha() const {
    return alpha_fixed_;
}

float RL_Naf_massVari::get_applied_damping() const {
    const float D = mass_act_ * alpha_fixed_ * md_ratio_;
    return std::isfinite(D) ? D : 0.0f;
}

float RL_Naf_massVari::get_last_network_action() const {
    return last_a_net_;
}

float RL_Naf_massVari::get_filtered_fext() const {
    return fext_filt_prev_;
}
