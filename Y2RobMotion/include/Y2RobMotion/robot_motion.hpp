#pragma once

#include <array>
#include <vector>
#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include "Y2Matrix/YMatrix.hpp"
#include "Y2RobMotion/setup_parameters.hpp"
#include "Y2RobMotion/runtime_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <cstdio>
#include "gforce_control/admittance_control.hpp"
#include "gforce_control/six_dof_force_controller.hpp"

class robot_motion: public BaseKinematics
{
public:
    robot_motion(rclcpp::Node::SharedPtr node, const std::string& RB_name,
                 double Control_period, int numOfJoint, const YMatrix& HTMEE2TCP);

    bool jointsReceived() const {
        return current_angles_received;
    }
    void start(bool monitoring_flag_ = true){
        start_flag = true;
        monitoring_flag = monitoring_flag_;
    }

    /* Robot states */
    std::vector<double> current_angles, pre_current_angles, current_angvel;
    std::vector<double> target_angles, pre_target_angles, target_angvel;
    std::vector<double> current_pose, pre_current_pose; // x,y,z,wx,wy,wz
    std::vector<double> current_carvel;
    std::vector<double> target_pose, pre_target_pose; // x,y,z,wx,wy,wz
    std::vector<double> target_carvel;
    std::vector<double> joy_target_pose; // x,y,z,wx,wy,wz
    std::vector<double> AC_pose; // x,y,z,wx,wy,wz
    YMatrix target_HTM;

    /* FT Sensor States */
    std::vector<double> ft1data;

    /* Control mode */
    std::string control_mode; // Idling, Position, Custom
    std::string pre_control_mode; // Previous control mode for comparison

    /* Force control mode */
    std::string force_con_mode;

    /* Robot name */
    std::string robot_name;

    /* Number of joints */
    unsigned int numOfJoints;

private:
    /* Node pointer instance */
    rclcpp::Node::SharedPtr node_;

    /* Basic parameters */
    double Control_period_ = 0.008;

    /* Monitoring flag */
    bool monitoring_flag = false;

    /* Start flag */
    bool start_flag = false;

    /* Init flags */
    bool current_angles_received = false;

    /* Mimic mode */
    std::string Mimic_mode; // Master, Slave, None

    /* Admittance control object */
    gforce_control::AdmittanceControl AControl[6];
    std::vector<double> HG_AC_desX; // Hand-guiding Desired Pose
    std::vector<double> FC_AC_desX; // Force-control Desired Pose
    std::vector<double> FC_MASS, FC_DAMPER, FC_STIFFNESS;
    std::vector<double> joy_move_axes_;
    std::vector<double> joy_velocity_command_;
    std::vector<double> joy_force_command_;
    std::vector<double> camera_teaching_command_;
    std::vector<double> camera_joystick_reference_pose_;
    std::vector<double> robot_joystick_reference_pose_;
    bool camera_teaching_command_received_ = false;
    bool camera_joystick_reference_valid_ = false;
    rclcpp::Time camera_teaching_command_stamp_;
    RobotRuntimeConfig runtime_config_;
    std::array<double, 6> camera_pose_gains_ = {};
    std::array<double, 6> camera_max_deltas_ = {};
    std::array<double, 6> camera_deadbands_ = {};
    std::array<double, 6> camera_rate_limits_ = {};
    std::array<int, 6> joy_axis_mapping_ = {};
    std::array<double, 6> joy_axis_scales_ = {};
    int joy_force_input_axis_ = 2;
    double joy_force_input_scale_ = 5.0;
    int joy_force_target_axis_ = 2;
    double joy_force_input_neutral_ = 1.0;
    double joy_force_deadband_ = 0.03;

    /* GForce 6-DOF force controller */
    gforce_control::SixDofForceController gforce_controller_;
    gforce_control::SixDofControllerConfig gforce_config_;
    gforce_control::Vector6 gforce_last_mass_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    gforce_control::Vector6 gforce_last_damping_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    gforce_control::Vector6 gforce_last_stiffness_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    /* Timer callback */
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr minitoring_timer_;

    /* ROS message parts */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        currentJ_pub, currentP_pub, currentF_pub, targetJ_pub, targetP_pub, targetF_pub,
        remapedCmd_pub, currentMDK_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ctlMode_pub;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmdMotion_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joyMove_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cameraTeaching_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmdMode_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr JointState_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr RemapedState_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ftsensor_sub;

    std_msgs::msg::Float64MultiArray currentJ_msg;
    std_msgs::msg::Float64MultiArray currentp_msg;
    std_msgs::msg::Float64MultiArray currentF_msg;
    std_msgs::msg::Float64MultiArray currentMDK_msg;

    std_msgs::msg::Float64MultiArray targetJ_msg;
    std_msgs::msg::Float64MultiArray targetP_msg;
    std_msgs::msg::Float64MultiArray targetF_msg;
    std_msgs::msg::Float64MultiArray remapedCmd_msg;

    std_msgs::msg::String ctlMode_msg;

    /* Callback function */
    void cmdMotionCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void joyMoveCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void cameraTeachingCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void cmdModeCB(const std_msgs::msg::String::SharedPtr msg);
    void JointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg);
    void RemapedStateCB(const sensor_msgs::msg::JointState::SharedPtr msg);
    void ftsensorCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

    /* Joint_state: mapping generation */
    std::vector<std::string> joint_names;
    std::vector<int> joint_mapping_;
    bool mapping_initialized_ = false;
    void initializeJointMapping(const sensor_msgs::msg::JointState::SharedPtr msg);

    /* Control functions */
    void control_idling();
    void control_joystick();
    void control_joystick_force();
    void control_position();
    void control_guiding();
    void control_force();
    void initialize_force_control_state();
    void update_force_target_from_joystick();
    void execute_force_control();

    /* Main control loop */
    void main_control();

    void state_update();
    void state_monitoring();
    void state_publisher();

};
