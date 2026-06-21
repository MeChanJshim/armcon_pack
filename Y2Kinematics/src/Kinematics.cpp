#include "Y2Kinematics/Kinematics.hpp"

#include <algorithm>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace {

std::vector<double> clampToBounds(const std::vector<double>& values,
                                  const std::vector<double>& lb,
                                  const std::vector<double>& ub) {
    std::vector<double> clamped = values;
    for (size_t i = 0; i < clamped.size(); ++i) {
        clamped[i] = std::max(lb[i], std::min(ub[i], clamped[i]));
    }
    return clamped;
}

YMatrix vectorToColumnMatrix(const std::vector<double>& values) {
    YMatrix column(values.size(), 1);
    for (size_t i = 0; i < values.size(); ++i) {
        column[i][0] = values[i];
    }
    return column;
}

}

// Constructor
Kinematics::Kinematics(double SamplingTime_, size_t numOfAxis_, const YMatrix& EE2TCP_)
: dt(SamplingTime_), numOfAxis(numOfAxis_), EE2TCP(EE2TCP_) {

    // Joint limits default
    q_min.assign(numOfAxis, -2.0 * M_PI);
    q_max.assign(numOfAxis,  2.0 * M_PI);

    qd_min.assign(numOfAxis, (q_min[0] / 2.0) / dt);
    qd_max.assign(numOfAxis, (q_max[0] / 2.0) / dt);

    // Accel limits default: unbounded
    a_min.assign(numOfAxis, -std::numeric_limits<double>::infinity());
    a_max.assign(numOfAxis,  std::numeric_limits<double>::infinity());

    q_prev.resize(numOfAxis, 0.0);
    has_prev = false;
}

// Bounded-iteration Damped Least Squares IK solver
std::vector<double> Kinematics::solve_IK(const std::vector<double>& q_current,
                                         const YMatrix& target_HTM_) {

    if (q_current.size() != numOfAxis) {
        throw std::invalid_argument("solve_IK: q_current size mismatch");
    }

    constexpr int max_iterations = 5;
    constexpr double position_tolerance_mm = 0.05;
    constexpr double rotation_tolerance_rad = 1.0e-4;
    constexpr double max_delta_q_per_iteration = 0.03;

    // 0) Align target to the TCP frame used by the Jacobian.
    YMatrix target_HTM = target_HTM_ * EE2TCP.inverse();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            target_HTM[i][j] = roundToNthDecimal(target_HTM[i][j], ik_precision);
        }
    }

    // 1) Bounds on q_next (angle/vel/acc intersection for this control tick).
    std::vector<double> lb(numOfAxis), ub(numOfAxis);
    for (size_t i = 0; i < numOfAxis; ++i) {
        const double lb_angle = q_min[i];
        const double ub_angle = q_max[i];

        const double lb_vel = q_current[i] + dt * qd_min[i];
        const double ub_vel = q_current[i] + dt * qd_max[i];

        double lb_acc = -std::numeric_limits<double>::infinity();
        double ub_acc =  std::numeric_limits<double>::infinity();
        if (has_prev) {
            lb_acc = dt*dt * a_min[i] + 2.0 * q_current[i] - q_prev[i];
            ub_acc = dt*dt * a_max[i] + 2.0 * q_current[i] - q_prev[i];
        }

        lb[i] = std::max({lb_angle, lb_vel, lb_acc});
        ub[i] = std::min({ub_angle, ub_vel, ub_acc});

        if (lb[i] > ub[i]) {
            // infeasible: collapse to midpoint (minimal disturbance)
            const double mid = 0.5 * (lb[i] + ub[i]);
            lb[i] = mid - 1e-9;
            ub[i] = mid + 1e-9;
        }
    }

    std::vector<double> q_next = clampToBounds(q_current, lb, ub);
    const double damping = std::max(std::abs(dls_damping), 1e-9);

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        YMatrix current_HTM = forwardKinematics(q_next) * EE2TCP.inverse();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                current_HTM[i][j] = roundToNthDecimal(current_HTM[i][j], ik_precision);
            }
        }

        std::vector<double> e_pos(3);
        double pos_norm_sq = 0.0;
        for (int i = 0; i < 3; ++i) {
            e_pos[i] = target_HTM[i][3] - current_HTM[i][3];
            pos_norm_sq += e_pos[i] * e_pos[i];
        }

        YMatrix R_target  = target_HTM.extract(0, 0, 3, 3);
        YMatrix R_current = current_HTM.extract(0, 0, 3, 3);
        YMatrix R_err = R_target * R_current.transpose();

        std::vector<double> e_rot(3);
        e_rot[0] = 0.5 * (R_err[2][1] - R_err[1][2]);
        e_rot[1] = 0.5 * (R_err[0][2] - R_err[2][0]);
        e_rot[2] = 0.5 * (R_err[1][0] - R_err[0][1]);
        const double rot_norm_sq =
            e_rot[0] * e_rot[0] + e_rot[1] * e_rot[1] + e_rot[2] * e_rot[2];

        if (std::sqrt(pos_norm_sq) < position_tolerance_mm &&
            std::sqrt(rot_norm_sq) < rotation_tolerance_rad) {
            break;
        }

        std::vector<double> Delta_x_des(6);
        for (int i = 0; i < 3; ++i) {
            Delta_x_des[i]   = Kp_pos * e_pos[i];
            Delta_x_des[i+3] = Kp_rot * e_rot[i];
        }

        YMatrix J = calculateJacobian(q_next);
        if (J.rows() != 6 || J.cols() != numOfAxis) {
            std::cerr << "[Kinematics] Jacobian size error: got "
                      << J.rows() << "x" << J.cols()
                      << ", expected 6x" << numOfAxis << "\n";
            return q_current;
        }

        YMatrix task_metric = (J * J.transpose()) + (YMatrix::identity(6) * (damping * damping));
        YMatrix delta_q_mat = J.transpose() * task_metric.inverse() * vectorToColumnMatrix(Delta_x_des);

        double max_abs_delta = 0.0;
        for (size_t i = 0; i < numOfAxis; ++i) {
            const double delta_q = std::clamp(
                delta_q_mat[i][0],
                -max_delta_q_per_iteration,
                max_delta_q_per_iteration);
            q_next[i] += delta_q;
            max_abs_delta = std::max(max_abs_delta, std::fabs(delta_q));
        }

        q_next = clampToBounds(q_next, lb, ub);

        if (max_abs_delta < 1.0e-7) {
            break;
        }
    }

    // 2) Update history.
    q_prev = q_current;
    has_prev = true;

    return q_next;
}

// Utility
double Kinematics::roundToNthDecimal(double value, int n) {
    const double m = std::pow(10.0, static_cast<double>(n));
    return std::round(value * m) / m;
}

void Kinematics::setControlGains(double kp_pos, double kp_rot) {
    Kp_pos = kp_pos;
    Kp_rot = kp_rot;
}

void Kinematics::setDLSDamping(double damping) {
    if (damping <= 0.0) {
        throw std::invalid_argument("DLS damping must be positive");
    }
    dls_damping = damping;
}

void Kinematics::setJointLimits(const std::vector<double>& q_min_in,
                                const std::vector<double>& q_max_in,
                                const std::vector<double>& qd_min_in,
                                const std::vector<double>& qd_max_in) {
    if (q_min_in.size() != numOfAxis || q_max_in.size() != numOfAxis ||
        qd_min_in.size() != numOfAxis || qd_max_in.size() != numOfAxis) {
        throw std::invalid_argument("Joint limits must have " + std::to_string(numOfAxis) + " elements each");
    }
    q_min  = q_min_in;
    q_max  = q_max_in;
    qd_min = qd_min_in;
    qd_max = qd_max_in;
}

void Kinematics::setAccelLimits(const std::vector<double>& a_min_in,
                                const std::vector<double>& a_max_in) {
    if (a_min_in.size() != numOfAxis || a_max_in.size() != numOfAxis) {
        throw std::invalid_argument("Acceleration limits must have " + std::to_string(numOfAxis) + " elements each");
    }
    a_min = a_min_in;
    a_max = a_max_in;
}

void Kinematics::setPrevQ(const std::vector<double>& q_prev_in) {
    if (q_prev_in.size() != numOfAxis) {
        throw std::invalid_argument("q_prev must have " + std::to_string(numOfAxis) + " elements");
    }
    q_prev = q_prev_in;
    has_prev = true;
}

void Kinematics::printPose(const YMatrix& pose, const std::string& label) {
    std::cout << label << " Pose:\n";
    for (const auto& row : pose) {
        for (const auto& val : row) std::cout << val << " ";
        std::cout << "\n";
    }
}
