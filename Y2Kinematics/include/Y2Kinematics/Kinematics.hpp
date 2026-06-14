#pragma once

#include "Y2Matrix/YMatrix.hpp"

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <string>

class Kinematics {
public:
    Kinematics(double SamplingTime_ = 0.01,
               size_t numOfAxis_ = 7,
               const YMatrix& EE2TCP_ = YMatrix::identity(4));

    // Forward Kinematics (Based on End-Effector)
    virtual YMatrix forwardKinematics(const std::vector<double>& q) = 0;

    // Jacobian Calculation (Based on End-Effector)
    virtual YMatrix calculateJacobian(const std::vector<double>& q) = 0;

    // IK solver (Based on End-Effector)
    std::vector<double> solve_IK(const std::vector<double>& q_current,
                                 const YMatrix& target_HTM);

    // Utility
    static double roundToNthDecimal(double value, int n);

    void setControlGains(double kp_pos, double kp_rot);
    void setDLSDamping(double damping);

    void setJointLimits(const std::vector<double>& q_min,
                        const std::vector<double>& q_max,
                        const std::vector<double>& qd_min,
                        const std::vector<double>& qd_max);

    void setAccelLimits(const std::vector<double>& a_min,
                        const std::vector<double>& a_max);

    void setPrevQ(const std::vector<double>& q_prev_in);

    void printPose(const YMatrix& pose, const std::string& label);

protected:
    double dt = 0.01;
    size_t numOfAxis = 7;
    YMatrix EE2TCP = YMatrix::identity(4);

    int ik_precision = 4;

    // task gains
    double Kp_pos = 100.0;
    double Kp_rot = 10.0;

    double dls_damping = 0.001;

    // limits
    std::vector<double> q_min, q_max;
    std::vector<double> qd_min, qd_max;
    std::vector<double> a_min, a_max;

    // history
    std::vector<double> q_prev;
    bool has_prev = false;
};
