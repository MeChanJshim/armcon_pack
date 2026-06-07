#include "gforce_control/admittance_control.hpp"

#include <cstdio>

namespace gforce_control {

AdmittanceControl::AdmittanceControl(double sampling_time)
{
    dt_ = sampling_time;
    hardReset();
}

bool AdmittanceControl::setMDK(double mass, double damping, double stiffness)
{
    mass_ = mass;
    damping_ = damping;
    stiffness_ = stiffness;
    return true;
}

double AdmittanceControl::monitorMDK(int select) const
{
    if (select == 0) return mass_;
    if (select == 1) return damping_;
    if (select == 2) return stiffness_;

    std::printf("[Admittance] Wrong MDK monitor selection\n");
    return 0.0;
}

double AdmittanceControl::update(double desired_position,
                                 double desired_force,
                                 double external_force)
{
    force_error_[0] = desired_force - external_force;

    a_ = 4.0 * mass_ - 2.0 * dt_ * damping_ + dt_ * dt_ * stiffness_;
    b_ = 2.0 * dt_ * dt_ * stiffness_ - 8.0 * mass_;
    c_ = 4.0 * mass_ + 2.0 * dt_ * damping_ + dt_ * dt_ * stiffness_;

    command_ = desired_position
             + (1.0 / c_) *
               (b_ * position_error_[1]
              + a_ * position_error_[2]
              - dt_ * dt_ *
                (force_error_[0] + 2.0 * force_error_[1] + force_error_[2]));

    position_error_[0] = desired_position - command_;
    position_error_[2] = position_error_[1];
    position_error_[1] = position_error_[0];

    force_error_[2] = force_error_[1];
    force_error_[1] = force_error_[0];

    return command_;
}

void AdmittanceControl::reset(double desired_position)
{
    command_ = desired_position;

    for (int i = 0; i < 3; ++i) {
        position_error_[i] = 0.0;
        force_error_[i] = 0.0;
    }
}

void AdmittanceControl::hardReset()
{
    command_ = 0.0;

    for (int i = 0; i < 3; ++i) {
        position_error_[i] = 0.0;
        force_error_[i] = 0.0;
    }

    a_ = 0.0;
    b_ = 0.0;
    c_ = 1.0;
}

}  // namespace gforce_control
