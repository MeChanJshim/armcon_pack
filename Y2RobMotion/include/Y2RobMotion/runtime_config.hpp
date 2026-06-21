#pragma once

#include <array>
#include <string>
#include <vector>

#include "Y2Matrix/YMatrix.hpp"

struct RobotRuntimeConfig {
    int number_of_joints = 6;
    std::string package_bundle_dir = "/home/jay/armcon_ws/src";
    int trajectory_mode = 1;
    double control_period = 0.001;
    std::string robot_name = "ur10skku";

    bool test_mode = false;
    bool remapping_enabled = true;
    std::string remap_state_topic = "/joint_states";
    std::string remap_command_topic = "/forward_position_controller/commands";
    std::string joy_move_topic_suffix = "/joy_move";
    std::string camera_teaching_command_topic = "/y2_gesteaching/teaching_command";

    std::array<int, 6> joystick_axis_mapping = {4, 3, 7, 1, 0, 6};
    std::array<double, 6> joystick_axis_scales = {20.0, 20.0, 20.0, 0.2, 0.2, 0.2};

    std::array<double, 6> camera_pose_gains = {1.0, 1.0, 1.0, 0.6, 0.6, 0.6};
    std::array<double, 6> camera_max_deltas = {800.0, 800.0, 800.0, 1.35, 1.35, 1.35};
    std::array<double, 6> camera_deadbands = {5.0, 5.0, 5.0, 0.03, 0.03, 0.03};
    std::array<double, 6> camera_rate_limits = {1.5, 1.5, 1.5, 0.008, 0.008, 0.008};
    double camera_command_timeout = 0.3;

    int joystick_force_input_axis = 2;
    double joystick_force_input_scale = 5.0;
    int joystick_force_target_axis = 2;
    double joystick_force_input_neutral = 1.0;
    double joystick_force_deadband = 0.03;

    YMatrix ee_to_tcp = {
        {-1.0,  0.0,  0.0,  0.0 },
        { 0.0,  1.0,  0.0,  0.0 },
        { 0.0,  0.0, -1.0,  153.0 },
        { 0.0,  0.0,  0.0,  1.0 }
    };

    std::vector<std::string> joint_names = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        ""
    };
};

RobotRuntimeConfig loadRobotRuntimeConfig(const std::string& path);
RobotRuntimeConfig loadInstalledRobotRuntimeConfig();
