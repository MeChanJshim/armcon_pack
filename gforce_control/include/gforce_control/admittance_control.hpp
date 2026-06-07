#pragma once

namespace gforce_control {

class AdmittanceControl {
public:
    AdmittanceControl() = default;
    explicit AdmittanceControl(double sampling_time);

    bool setMDK(double mass, double damping, double stiffness);
    double monitorMDK(int select) const;
    double update(double desired_position, double desired_force, double external_force);
    void reset(double desired_position);
    void hardReset();

    bool adm_1D_MDK(double mass, double damping, double stiffness) { return setMDK(mass, damping, stiffness); }
    double adm_MDK_monitor(int select) const { return monitorMDK(select); }
    double adm_1D_control(double desired_position, double desired_force, double external_force)
    {
        return update(desired_position, desired_force, external_force);
    }

private:
    double position_error_[3] = {0.0, 0.0, 0.0};
    double force_error_[3] = {0.0, 0.0, 0.0};

    double mass_ = 1.0;
    double damping_ = 1.0;
    double stiffness_ = 0.0;

    double a_ = 0.0;
    double b_ = 0.0;
    double c_ = 1.0;

    double dt_ = 0.001;
    double command_ = 0.0;
};

}  // namespace gforce_control
