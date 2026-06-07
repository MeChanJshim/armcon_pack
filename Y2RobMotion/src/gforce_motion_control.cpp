#include "Y2RobMotion/robot_motion.hpp"

#include <array>

namespace {

gforce_control::Vector6 toVector6(const std::vector<double>& values)
{
    gforce_control::Vector6 out = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < out.size() && i < values.size(); ++i) {
        out[i] = values[i];
    }
    return out;
}

}  // namespace

void robot_motion::initialize_force_control_state()
{
    target_pose = current_pose;
    target_angles = current_angles;
    joy_target_pose = current_pose;

    FC_AC_desX = std::vector<double>(9, 0.0);
    FC_AC_desX[0] = current_pose[0] / 1000.0;
    FC_AC_desX[1] = current_pose[1] / 1000.0;
    FC_AC_desX[2] = current_pose[2] / 1000.0;
    FC_AC_desX[3] = current_pose[3];
    FC_AC_desX[4] = current_pose[4];
    FC_AC_desX[5] = current_pose[5];

    gforce_controller_.reset(toVector6(current_pose));

    AC_pose = current_pose;
    AC_pose[0] /= 1000.0;
    AC_pose[1] /= 1000.0;
    AC_pose[2] /= 1000.0;

    RCLCPP_INFO(node_->get_logger(), "GForce control was initialized");
}

void robot_motion::update_force_target_from_joystick()
{
    for(int i = 0; i < 6; ++i)
    {
        joy_target_pose[i] += joy_velocity_command_[i] * Control_period_;
    }

    FC_AC_desX[0] = joy_target_pose[0] / 1000.0;
    FC_AC_desX[1] = joy_target_pose[1] / 1000.0;
    FC_AC_desX[2] = joy_target_pose[2] / 1000.0;
    FC_AC_desX[3] = joy_target_pose[3];
    FC_AC_desX[4] = joy_target_pose[4];
    FC_AC_desX[5] = joy_target_pose[5];

    for(int i = 0; i < 3; ++i)
    {
        FC_AC_desX[i + 6] = joy_force_command_[i];
    }
}

void robot_motion::execute_force_control()
{
    force_con_mode = "GForce";

    gforce_control::Vector6 desired_pose = {
        FC_AC_desX[0] * 1000.0,
        FC_AC_desX[1] * 1000.0,
        FC_AC_desX[2] * 1000.0,
        FC_AC_desX[3],
        FC_AC_desX[4],
        FC_AC_desX[5]};

    const gforce_control::Vector6 measured_pose = toVector6(current_pose);
    const gforce_control::Vector3 desired_force = {
        FC_AC_desX[6],
        FC_AC_desX[7],
        FC_AC_desX[8]};
    const gforce_control::Vector6 measured_force = toVector6(ft1data);

    const gforce_control::Vector6 command =
        gforce_controller_.computeCommand(desired_pose, measured_pose, desired_force, measured_force);

    target_pose.assign(command.begin(), command.end());

    AC_pose = target_pose;
    AC_pose[0] /= 1000.0;
    AC_pose[1] /= 1000.0;
    AC_pose[2] /= 1000.0;

    const auto& debug = gforce_controller_.debugState();
    gforce_last_mass_ = debug.applied_mass;
    gforce_last_damping_ = debug.applied_damping;
    gforce_last_stiffness_ = debug.applied_stiffness;
    for(int i = 0; i < 6; ++i) {
        AControl[i].adm_1D_MDK(
            gforce_last_mass_[i],
            gforce_last_damping_[i],
            gforce_last_stiffness_[i]);
    }

    std::vector<double> target_ori = {target_pose[3], target_pose[4], target_pose[5]};
    auto target_rot = YMatrix::fromSpatialAngle(target_ori);
    target_HTM = YMatrix::identity(4);
    target_HTM.insert(0, 0, target_rot);
    target_HTM[0][3] = target_pose[0];
    target_HTM[1][3] = target_pose[1];
    target_HTM[2][3] = target_pose[2];

    target_angles = solve_IK(target_angles, target_HTM);
    pre_control_mode = control_mode;
}

void robot_motion::control_force()
{
    control_mode = "Force";

    if(pre_control_mode != control_mode)
    {
        initialize_force_control_state();
    }

    execute_force_control();
}
