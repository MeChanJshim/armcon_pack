#include "Y2ForceCon/context_naf.hpp"

#include <torch/torch.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>

RL_ContextNAF::RL_ContextNAF(const std::string& model_path,
                             int threads,
                             c10::Device device,
                             float dt,
                             float md_ratio,
                             float fc_fext,
                             std::array<float,2> action_low,
                             std::array<float,2> action_high)
: device_(device),
  dt_(dt),
  md_ratio_(md_ratio),
  fc_fext_(fc_fext),
  action_low_(action_low),
  action_high_(action_high)
{
    torch::set_num_threads(std::max(1, threads));
    module_ = torch::jit::load(model_path, device_);
    module_.eval();

    action_mid_[0] = 0.5f * (action_low_[0] + action_high_[0]);
    action_mid_[1] = 0.5f * (action_low_[1] + action_high_[1]);

    update_alpha_lp(dt_);
    reset_state();
}

void RL_ContextNAF::set_sampling_time(float dt) {
    dt_ = dt;
    update_alpha_lp(dt_);
}

void RL_ContextNAF::update_alpha_lp(float dt) {
    // alpha_lp = exp(-2*pi*fc*dt)
    alpha_lp_ = std::exp(-2.0f * static_cast<float>(M_PI) * fc_fext_ * dt);
}

void RL_ContextNAF::reset_state() {
    // derivative state
    has1_ = has2_ = false;
    xc_prev1_ = xc_prev2_ = 0.0f;
    x_prev1_ = 0.0f;
    x_dot_prev1_ = 0.0f;

    // residual bookkeeping
    prev_er_ = 0.0f;
    prev_a_ = action_mid_;     // start with midpoint action
    md_prev_ = prev_a_[0];
    dd_prev_ = md_prev_ * prev_a_[1] * md_ratio_;

    // IMPORTANT: this will be initialized on first run() call
    fext_filt_prev_ = 0.0f;

    // hidden
    has_hidden_ = false;
    h_ = torch::Tensor();
}

torch::Tensor RL_ContextNAF::ensure_hidden() {
    if (has_hidden_ && h_.defined()) return h_;

    // call scripted method: init_hidden(batch_size)
    // Python wrapper: init_hidden(int) -> Tensor [1,B,h_dim]
    torch::IValue iv = module_.get_method("init_hidden")({1});
    if (!iv.isTensor()) {
        throw std::runtime_error("init_hidden() did not return a Tensor.");
    }
    h_ = iv.toTensor().to(device_);
    has_hidden_ = true;
    return h_;
}

float RL_ContextNAF::clampf(float v, float lo, float hi) {
    return std::min(std::max(v, lo), hi);
}

std::array<float,2> RL_ContextNAF::squash_to_bounds_tanh(const std::array<float,2>& raw,
                                                         const std::array<float,2>& low,
                                                         const std::array<float,2>& high)
{
    // python: z = tanh(x); a = low + 0.5*(z+1)*(high-low)
    std::array<float,2> out;
    for (int i=0;i<2;i++){
        float z = std::tanh(raw[i]); // [-1,1]
        out[i] = low[i] + 0.5f * (z + 1.0f) * (high[i] - low[i]);
        out[i] = clampf(out[i], low[i], high[i]);
    }
    return out;
}

static inline float safe_float(float v, float fallback = 0.0f) {
    return std::isfinite(v) ? v : fallback;
}

std::vector<double> RL_ContextNAF::run(float xc, float x, float Fd, float Env_Fext) {
    torch::NoGradGuard _ng;

    // finite guards on inputs
    xc       = safe_float(xc, 0.0f);
    x        = safe_float(x,  0.0f);
    Fd       = safe_float(Fd, 0.0f);
    Env_Fext = safe_float(Env_Fext, 0.0f);

    // ---------------------------------------------------------
    // 1) numerical derivatives for RL state s
    //    s = [xc_ddot, x_dot, x_ddot, (Fd - Fext_filt)]
    //
    // NOTE:
    // - We compute xc_dot numerically (since run() does not receive env-provided xc_dot).
    // - To reduce "initial 2-step dead zone", we compute xc_dot as soon as we have xc_prev1_.
    // ---------------------------------------------------------
    float x_dot = 0.0f, x_ddot = 0.0f;
    float xc_dot = 0.0f, xc_ddot = 0.0f;

    if (!has1_) {
        // first sample
        xc_prev1_ = xc;
        xc_prev2_ = xc;   // so that 2nd difference starts from 0 naturally
        x_prev1_  = x;
        x_dot_prev1_ = 0.0f;
        has1_ = true;

        // on first step, filtered force baseline
        fext_filt_prev_ = Env_Fext;

        // derivatives remain 0 on very first sample
        x_dot = 0.0f; x_ddot = 0.0f;
        xc_dot = 0.0f; xc_ddot = 0.0f;
    }
    else {
        // we can compute first derivatives from here
        x_dot  = (x  - x_prev1_)  / dt_;
        xc_dot = (xc - xc_prev1_) / dt_;

        if (!has2_) {
            // second sample: we have velocity but not reliable acceleration yet
            x_ddot  = 0.0f;
            xc_ddot = 0.0f;

            // shift history
            x_dot_prev1_ = x_dot;

            xc_prev2_ = xc_prev1_;
            xc_prev1_ = xc;
            x_prev1_  = x;
            has2_ = true;
        }
        else {
            // third+ sample: full accel available
            x_ddot  = (x_dot  - x_dot_prev1_) / dt_;
            xc_ddot = (xc - 2.0f*xc_prev1_ + xc_prev2_) / (dt_ * dt_);

            // update derivative state
            x_dot_prev1_ = x_dot;
            x_prev1_  = x;
            xc_prev2_ = xc_prev1_;
            xc_prev1_ = xc;
        }
    }

    // ---------------------------------------------------------
    // 2) Low-pass filter Fext to match python compute_residual_q
    //    Fext_filt = alpha_lp * prev + (1-alpha_lp)*raw
    // ---------------------------------------------------------
    float fext_filt = alpha_lp_ * safe_float(fext_filt_prev_, Env_Fext)
                    + (1.0f - alpha_lp_) * Env_Fext;
    fext_filt = safe_float(fext_filt, Env_Fext);

    // ---------------------------------------------------------
    // 3) Residual and q_t  (MATCH TRAINING)
    //    e_r  = Fext_filt - Fd - md_prev*xc_ddot - dd_prev*xc_dot
    //    de_r = e_r - prev_er
    //    q    = [e_r, de_r, a_{t-1}(mass,alpha)]
    // ---------------------------------------------------------
    float er  = fext_filt - Fd - md_prev_ * xc_ddot - dd_prev_ * xc_dot;
    er = safe_float(er, 0.0f);
    float der = er - prev_er_;
    der = safe_float(der, 0.0f);

    // q_dim = 2 + action_dim = 4
    std::array<float,4> q_arr { er, der, prev_a_[0], prev_a_[1] };

    // ---------------------------------------------------------
    // 4) Build tensors (B=1)
    // ---------------------------------------------------------
    // s: [1,4]
    // NOTE: s[3] uses (Fd - Fext_filt) consistent with training
    auto s = torch::tensor(
        { xc_ddot, x_dot, x_ddot, (Fd - fext_filt) },
        torch::TensorOptions().dtype(torch::kFloat32)
    ).unsqueeze(0).to(device_);

    auto q = torch::tensor(
        { q_arr[0], q_arr[1], q_arr[2], q_arr[3] },
        torch::TensorOptions().dtype(torch::kFloat32)
    ).unsqueeze(0).to(device_);

    // h: [1,1,h_dim]
    auto h = ensure_hidden();

    // ---------------------------------------------------------
    // 5) Forward: act(s,q,h) -> (mu_raw, h_next)
    // ---------------------------------------------------------
    torch::IValue iv = module_.get_method("act")({s, q, h});
    if (!iv.isTuple()) {
        throw std::runtime_error("act() must return a tuple (mu, h_next).");
    }
    const auto& elems = iv.toTuple()->elements();
    if (elems.size() < 2 || !elems[0].isTensor() || !elems[1].isTensor()) {
        throw std::runtime_error("act() returned invalid tuple. Expected (Tensor mu, Tensor h_next).");
    }

    torch::Tensor mu = elems[0].toTensor();     // [1,2] (raw)
    torch::Tensor h_next = elems[1].toTensor(); // [1,1,h_dim]

    // update hidden
    h_ = h_next.to(device_);
    has_hidden_ = true;

    // flatten mu -> [2] on CPU
    mu = mu.to(torch::kCPU).contiguous().reshape({-1});
    TORCH_CHECK(mu.numel() >= 2, "act() must return at least 2 scalars, got ", mu.sizes());

    float mu0 = safe_float(mu[0].item<float>(), 0.0f);
    float mu1 = safe_float(mu[1].item<float>(), 0.0f);
    std::array<float,2> mu_raw { mu0, mu1 };

    // ---------------------------------------------------------
    // 6) Squash + bound (same as python workers)
    // ---------------------------------------------------------
    auto a_squashed = squash_to_bounds_tanh(mu_raw, action_low_, action_high_);

    float mass  = safe_float(a_squashed[0], action_mid_[0]);
    float alpha = safe_float(a_squashed[1], action_mid_[1]);

    // ---------------------------------------------------------
    // 7) Update bookkeeping for next step
    // ---------------------------------------------------------
    prev_er_ = er;
    prev_a_  = { mass, alpha };
    md_prev_ = mass;
    dd_prev_ = mass * alpha * md_ratio_;
    fext_filt_prev_ = fext_filt;

    // return [mass, alpha]
    std::vector<double> out(2);
    out[0] = static_cast<double>(mass);
    out[1] = static_cast<double>(alpha);
    return out;
}
