#include "Y2RobMotion/robot_motion.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>

/* Constructor */
robot_motion::robot_motion(rclcpp::Node::SharedPtr node, const std::string& RB_name,
                           double Control_period, int numOfJoint, const YMatrix& HTMEE2TCP)
: node_(node)
, robot_name(RB_name)
, BaseKinematics(Control_period, static_cast<size_t>(numOfJoint), HTMEE2TCP)
, numOfJoints(numOfJoint)
, current_angles(numOfJoint,0.0)
, target_angles(numOfJoint,0.0)
, current_angvel(numOfJoint,0.0)
, target_angvel(numOfJoint,0.0)
, Control_period_(Control_period)
, current_pose(6,0.0)
, target_pose(6,0.0)
, joy_target_pose(6,0.0)
, AC_pose(6,0.0)
, HG_AC_desX(6,0.0)
, target_HTM(4,4)
, ft1data(6,0.0)
, current_carvel(6,0.0)
, target_carvel(6,0.0)
, pre_current_angles(numOfJoint,0.0)
, pre_target_angles(numOfJoint,0.0)
, pre_current_pose(6,0.0)
, pre_target_pose(6,0.0)
, FC_AC_desX(9,0.0)
, FC_MASS(6,0.0)
, FC_DAMPER(6,0.0)
, FC_STIFFNESS(6,0.0)
, joy_move_axes_(8,0.0)
, joy_velocity_command_(6,0.0)
, joy_force_command_(3,0.0)
, camera_teaching_command_(7,0.0)
, camera_joystick_reference_pose_(6,0.0)
, robot_joystick_reference_pose_(6,0.0)
{
    runtime_config_ = loadInstalledRobotRuntimeConfig();
    camera_pose_gains_ = runtime_config_.camera_pose_gains;
    camera_max_deltas_ = runtime_config_.camera_max_deltas;
    camera_deadbands_ = runtime_config_.camera_deadbands;
    camera_rate_limits_ = runtime_config_.camera_rate_limits;
    joy_axis_mapping_ = runtime_config_.joystick_axis_mapping;
    joy_axis_scales_ = runtime_config_.joystick_axis_scales;
    joy_force_input_axis_ = runtime_config_.joystick_force_input_axis;
    joy_force_input_scale_ = runtime_config_.joystick_force_input_scale;
    joy_force_target_axis_ = runtime_config_.joystick_force_target_axis;
    joy_force_input_neutral_ = runtime_config_.joystick_force_input_neutral;
    joint_names = runtime_config_.joint_names;

    /* Target HTM  init */
    target_HTM = YMatrix::identity(4);

    /* Control mode init */
    control_mode = "Idling";
    pre_control_mode = "none";

    /* Force control mode init */
    force_con_mode = "Defualt";

    /* ROS Publisher init */
    std::string currentJ_TP = robot_name + "/currentJ";
    std::string currentP_TP = robot_name + "/currentP";
    std::string currentF_TP = robot_name + "/currentF";
    std::string currentMDK_TP = robot_name + "/currentMDK";
    std::string targetJ_TP = robot_name + "/targetJ";
    std::string targetP_TP = robot_name + "/targetP";
    std::string targetF_TP = robot_name + "/targetF";
    std::string ctlMode_TP = robot_name + "/ctlMode";
    std::string remapedCmd_TP = runtime_config_.remap_command_topic;

    currentJ_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(currentJ_TP,1);
    currentP_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(currentP_TP,1);
    currentF_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(currentF_TP,1);
    currentMDK_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(currentMDK_TP,1);
    targetJ_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(targetJ_TP,1);
    targetP_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(targetP_TP,1);
    targetF_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(targetF_TP,1);
    remapedCmd_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(remapedCmd_TP,1);
    ctlMode_pub = node_->create_publisher<std_msgs::msg::String>(ctlMode_TP,10);
    RCLCPP_INFO(node_->get_logger(),"Publisher was generated");

    /* ROS Subscriber init */
    std::string cmdMotion_TP = robot_name + "/cmdMotion";
    std::string cmdMode_TP = robot_name + "/cmdMode";
    std::string joyMove_TP = robot_name + runtime_config_.joy_move_topic_suffix;
    std::string cameraTeaching_TP = runtime_config_.camera_teaching_command_topic;
    std::string jointState_TP = robot_name + "/joint_states";
    std::string ftdata_TP = robot_name + "/ftdata";
    std::string remapedState_TP = runtime_config_.remap_state_topic;

    cmdMotion_sub = node_->create_subscription<std_msgs::msg::Float64MultiArray>(cmdMotion_TP, 1,
        std::bind(&robot_motion::cmdMotionCB, this, std::placeholders::_1));
    joyMove_sub = node_->create_subscription<std_msgs::msg::Float64MultiArray>(joyMove_TP, 1,
        std::bind(&robot_motion::joyMoveCB, this, std::placeholders::_1));
    cameraTeaching_sub = node_->create_subscription<std_msgs::msg::Float64MultiArray>(cameraTeaching_TP, 1,
        std::bind(&robot_motion::cameraTeachingCB, this, std::placeholders::_1));
    cmdMode_sub = node_->create_subscription<std_msgs::msg::String>(cmdMode_TP, 10,
        std::bind(&robot_motion::cmdModeCB, this, std::placeholders::_1));

    JointState_sub = node_->create_subscription<sensor_msgs::msg::JointState>(jointState_TP, 1,
        std::bind(&robot_motion::JointStateCB, this, std::placeholders::_1));
    RemapedState_sub = node_->create_subscription<sensor_msgs::msg::JointState>(remapedState_TP, 1,
        std::bind(&robot_motion::RemapedStateCB, this, std::placeholders::_1));
    ftsensor_sub = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(ftdata_TP, 1,
        std::bind(&robot_motion::ftsensorCB, this, std::placeholders::_1));

    /* ROS timer callback */
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(Control_period*1000)),
        std::bind(&robot_motion::main_control,this));

    minitoring_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(100)),
        std::bind(&robot_motion::state_monitoring,this));

    RCLCPP_INFO(node_->get_logger(),"Subscription was generated");
    RCLCPP_INFO(node_->get_logger(),"Camera teaching command topic: %s", cameraTeaching_TP.c_str());

    /* Admittance controller initialization */
    for(int i =0;i<6;i++){ AControl[i] = gforce_control::AdmittanceControl(Control_period); }

    const std::string gforce_share_dir =
        ament_index_cpp::get_package_share_directory("gforce_control");
    const std::string gforce_config_path = gforce_share_dir + "/config/gforce_control.yaml";
    gforce_config_ = gforce_control::loadControllerConfigFromFile(gforce_config_path);
    gforce_config_.dt = Control_period;
    gforce_controller_.configure(gforce_config_);

    const std::string gforce_model_path =
        gforce_control::resolveModelPath(gforce_config_, gforce_share_dir);
    if(std::filesystem::exists(gforce_model_path)) {
        gforce_controller_.configureGForcePoliciesFromModelPath(gforce_model_path);
        RCLCPP_INFO(node_->get_logger(), "GForce model loaded: %s", gforce_model_path.c_str());
    } else {
        RCLCPP_WARN(
            node_->get_logger(),
            "GForce model not found yet: %s. Put gforce.pt there before running active GForce force control.",
            gforce_model_path.c_str());
    }
}

/* ROS CMD Motion Callback */
void robot_motion::cmdMotionCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if(!strcmp(control_mode.c_str(), "Position"))
    {
        for (size_t i = 0; i < msg->data.size() && i < 6; ++i) {
            target_pose[i] = msg->data[i];
        }
    }
    else if(!strcmp(control_mode.c_str(), "Force"))
    {
        for (size_t i = 0; i < msg->data.size() && i < 9; ++i) {
            /* mm->m, AC input: m, rad */
            if(i<3) {FC_AC_desX[i] = (msg->data[i])/(1000.0);}
            else{FC_AC_desX[i] = msg->data[i];}
        }
    }
    else {printf("\033[33mWrong data was received \033[0m\n");}
}

void robot_motion::joyMoveCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std::fill(joy_move_axes_.begin(), joy_move_axes_.end(), 0.0);
    for (size_t i = 0; i < msg->data.size() && i < joy_move_axes_.size(); ++i) {
        joy_move_axes_[i] = msg->data[i];
    }

    for (size_t i = 0; i < joy_velocity_command_.size(); ++i) {
        const int axis_index = joy_axis_mapping_[i];
        if (axis_index < 0 || axis_index >= static_cast<int>(joy_move_axes_.size())) {
            joy_velocity_command_[i] = 0.0;
            continue;
        }

        joy_velocity_command_[i] = joy_move_axes_[axis_index] * joy_axis_scales_[i];
    }

    std::fill(joy_force_command_.begin(), joy_force_command_.end(), 0.0);
    if (joy_force_target_axis_ >= 0 &&
        joy_force_target_axis_ < static_cast<int>(joy_force_command_.size()) &&
        joy_force_input_axis_ >= 0 &&
        joy_force_input_axis_ < static_cast<int>(joy_move_axes_.size())) {
        const double axis_delta =
            joy_force_input_neutral_ - joy_move_axes_[joy_force_input_axis_];
        joy_force_command_[joy_force_target_axis_] =
            axis_delta * joy_force_input_scale_;
    }
}

void robot_motion::cameraTeachingCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 6) {
        RCLCPP_WARN(node_->get_logger(), "Camera teaching command requires at least 6 values");
        return;
    }

    for (size_t i = 0; i < camera_teaching_command_.size(); ++i) {
        if (i < msg->data.size() && std::isfinite(msg->data[i])) {
            camera_teaching_command_[i] = msg->data[i];
        }
    }
    camera_teaching_command_received_ = true;
    camera_teaching_command_stamp_ = node_->now();
}

/* ROS CMD Mode Callback */
void robot_motion::cmdModeCB(const std_msgs::msg::String::SharedPtr msg)
{
    printf("Control mode callback \n");
    control_mode = msg->data;
}

/* ROS Joint State Callback */
void robot_motion::JointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if(!current_angles_received) {
        current_angles_received = true;
    }

    if (msg->position.size() < numOfJoints || msg->name.size() != msg->position.size()) {
        RCLCPP_WARN(node_->get_logger(), "Invalid joint state message");
        return;
    }

    // 첫 번째 메시지에서 매핑 테이블 생성
    if (!mapping_initialized_) {
        initializeJointMapping(msg);
    }

    // 매핑 테이블을 사용해서 빠르게 복사
    for (size_t i = 0; i < numOfJoints; ++i) {
        if (joint_mapping_[i] >= 0 && joint_mapping_[i] < static_cast<int>(msg->position.size())) {
            current_angles[i] = msg->position[joint_mapping_[i]];
        } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "Invalid mapping for joint %zu", i);
        }
    }
}

void robot_motion::RemapedStateCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if(!runtime_config_.remapping_enabled) {
        return;
    }

    if(!current_angles_received) {
        current_angles_received = true;
    }

    if (msg->position.size() < numOfJoints || msg->name.size() != msg->position.size()) {
        RCLCPP_WARN(node_->get_logger(), "Invalid joint state message");
        return;
    }

    // 첫 번째 메시지에서 매핑 테이블 생성
    if (!mapping_initialized_) {
        initializeJointMapping(msg);
    }

    // 매핑 테이블을 사용해서 빠르게 복사
    for (size_t i = 0; i < numOfJoints; ++i) {
        if (joint_mapping_[i] >= 0 && joint_mapping_[i] < static_cast<int>(msg->position.size())) {
            current_angles[i] = msg->position[joint_mapping_[i]];
        } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "Invalid mapping for joint %zu", i);
        }
    }
}

/* ROS FT Sensor Callback */
void robot_motion::ftsensorCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    ft1data[0] = msg->wrench.force.x;
    ft1data[1] = msg->wrench.force.y;
    ft1data[2] = msg->wrench.force.z;

    ft1data[3] = msg->wrench.torque.x;
    ft1data[4] = msg->wrench.torque.y;
    ft1data[5] = msg->wrench.torque.z;
}

/* joint state mapping function */
void robot_motion::initializeJointMapping(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_mapping_.resize(numOfJoints, -1);

    for (size_t i = 0; i < numOfJoints; ++i) {
        for (size_t j = 0; j < msg->name.size(); ++j) {
            if (joint_names[i] == msg->name[j]) {
                joint_mapping_[i] = j;
                RCLCPP_INFO(node_->get_logger(),
                            "Joint %s mapped: expected_index=%zu -> msg_index=%zu",
                            joint_names[i].c_str(), i, j);
                break;
            }
        }

        if (joint_mapping_[i] == -1) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Joint %s not found in joint state message!",
                         joint_names[i].c_str());
        }
    }
    mapping_initialized_ = true;
}

/* State Update */
void robot_motion::state_update()
{
    for(int i=0;i<numOfJoints;i++)
    {
        current_angvel[i] = (current_angles[i]-pre_current_angles[i])/Control_period_;
        target_angvel[i]  = (target_angles[i]-pre_target_angles[i])/Control_period_;
    }

    YMatrix current_HTM = forwardKinematics(current_angles);
    YMatrix current_orientation = current_HTM.extract(0,0,3,3);
    SpatialAngle current_SPangle = current_orientation.toSpatialAngle();

    current_pose[0] = current_HTM[0][3];
    current_pose[1] = current_HTM[1][3];
    current_pose[2] = current_HTM[2][3];
    current_pose[3] = current_SPangle.x;
    current_pose[4] = current_SPangle.y;
    current_pose[5] = current_SPangle.z;

    for(int i=0;i<6;i++)
    {
        current_carvel[i] = (current_pose[i]-pre_current_pose[i])/Control_period_;
        target_carvel[i]  = (target_pose[i]-pre_target_pose[i])/Control_period_;
    }

    pre_current_angles = current_angles;
    pre_target_angles  = target_angles;
    pre_current_pose   = current_pose;
    pre_target_pose    = target_pose;
}

/* ROS State Publisher */
void robot_motion::state_publisher()
{
    ctlMode_msg.data.clear();
    currentJ_msg.data.clear();
    currentp_msg.data.clear();
    currentF_msg.data.clear();
    currentMDK_msg.data.clear();

    targetJ_msg.data.clear();
    targetP_msg.data.clear();
    targetF_msg.data.clear();
    remapedCmd_msg.data.clear();

    for (int i=0;i<3;i++)
    {
        double norminal_MDK = sqrt(
            pow(AControl[0].adm_MDK_monitor(i),2)
            + pow(AControl[1].adm_MDK_monitor(i),2)
            + pow(AControl[2].adm_MDK_monitor(i),2)
        );
        currentMDK_msg.data.push_back(norminal_MDK);
    }

    for(int i =0; i<6; i++)
    {
        currentp_msg.data.push_back(current_pose[i]);
        currentF_msg.data.push_back(ft1data[i]);

        targetP_msg.data.push_back(target_pose[i]);
        if(i<3){ targetF_msg.data.push_back(FC_AC_desX[i+6]); }
        else {  targetF_msg.data.push_back(0.0); }
    }

    for(int i =0; i<numOfJoints; i++)
    {
        currentJ_msg.data.push_back(current_angles[i]);
        targetJ_msg.data.push_back(target_angles[i]);
        remapedCmd_msg.data.push_back(target_angles[i]);
    }

    ctlMode_msg.data = control_mode;

    ctlMode_pub->publish(ctlMode_msg);
    currentJ_pub->publish(currentJ_msg);
    currentP_pub->publish(currentp_msg);
    currentF_pub->publish(currentF_msg);
    currentMDK_pub->publish(currentMDK_msg);

    if(!runtime_config_.test_mode) {
        targetJ_pub->publish(targetJ_msg);
        targetP_pub->publish(targetP_msg);
        targetF_pub->publish(targetF_msg);
        if(runtime_config_.remapping_enabled){ remapedCmd_pub->publish(remapedCmd_msg); }
    }
}

/* Control Mode : Idling */
void robot_motion::control_idling()
{
    control_mode = "Idling";

    if(pre_control_mode != control_mode)
    {
        target_pose = current_pose;
        target_angles = current_angles;
    }

    if(control_mode == "Idling"){
        std::vector<double> target_ori = {target_pose[3], target_pose[4], target_pose[5]};
        auto target_rot = YMatrix::fromSpatialAngle(target_ori);

        target_HTM = YMatrix::identity(4);
        target_HTM.insert(0, 0, target_rot);
        target_HTM[0][3] = target_pose[0];
        target_HTM[1][3] = target_pose[1];
        target_HTM[2][3] = target_pose[2];

        auto current_HTM = forwardKinematics(target_angles);

        target_angles = solve_IK(target_angles, target_HTM);
        
    }

    pre_control_mode = control_mode;
}

/* Control Mode : Position */
void robot_motion::control_position()
{
    control_mode = "Position";

    if(pre_control_mode != control_mode)
    {
        target_pose = current_pose;
        target_angles = current_angles;
    }

    if(control_mode == "Position"){
        std::vector<double> target_ori = {target_pose[3], target_pose[4], target_pose[5]};
        auto target_rot = YMatrix::fromSpatialAngle(target_ori);

        target_HTM = YMatrix::identity(4);
        target_HTM.insert(0, 0, target_rot);
        target_HTM[0][3] = target_pose[0];
        target_HTM[1][3] = target_pose[1];
        target_HTM[2][3] = target_pose[2];

        target_angles = solve_IK(target_angles, target_HTM);
    }

    pre_control_mode = control_mode;
}

void robot_motion::control_joystick()
{
    control_mode = "Joystick";

    if(pre_control_mode != control_mode)
    {
        camera_joystick_reference_valid_ = false;
        initialize_force_control_state();
    }

    if(!camera_teaching_command_received_)
    {
        FC_AC_desX[0] = current_pose[0] / 1000.0;
        FC_AC_desX[1] = current_pose[1] / 1000.0;
        FC_AC_desX[2] = current_pose[2] / 1000.0;
        FC_AC_desX[3] = current_pose[3];
        FC_AC_desX[4] = current_pose[4];
        FC_AC_desX[5] = current_pose[5];
        for(int i = 0; i < 3; ++i) FC_AC_desX[i + 6] = 0.0;
        execute_force_control();
        return;
    }

    const double command_age = (node_->now() - camera_teaching_command_stamp_).seconds();
    if(command_age > runtime_config_.camera_command_timeout)
    {
        camera_joystick_reference_valid_ = false;
        FC_AC_desX[0] = current_pose[0] / 1000.0;
        FC_AC_desX[1] = current_pose[1] / 1000.0;
        FC_AC_desX[2] = current_pose[2] / 1000.0;
        FC_AC_desX[3] = current_pose[3];
        FC_AC_desX[4] = current_pose[4];
        FC_AC_desX[5] = current_pose[5];
        for(int i = 0; i < 3; ++i) FC_AC_desX[i + 6] = 0.0;
        execute_force_control();
        return;
    }

    if(!camera_joystick_reference_valid_)
    {
        robot_joystick_reference_pose_ = current_pose;
        joy_target_pose = current_pose;
        for(int i = 0; i < 6; ++i)
        {
            camera_joystick_reference_pose_[i] = camera_teaching_command_[i];
        }
        camera_joystick_reference_valid_ = true;
    }

    for(int i = 0; i < 6; ++i)
    {
        double delta = camera_teaching_command_[i] - camera_joystick_reference_pose_[i];
        if(i < 3) delta *= 1000.0; // camera metric command: m -> robot pose: mm

        if(std::fabs(delta) < camera_deadbands_[i])
        {
            delta = 0.0;
        }
        else
        {
            delta = std::copysign(std::fabs(delta) - camera_deadbands_[i], delta);
        }

        delta = std::clamp(delta, -camera_max_deltas_[i], camera_max_deltas_[i]);
        const double desired_pose = robot_joystick_reference_pose_[i] + camera_pose_gains_[i] * delta;
        const double limited_step = std::clamp(
            desired_pose - joy_target_pose[i],
            -camera_rate_limits_[i],
            camera_rate_limits_[i]);
        joy_target_pose[i] += limited_step;
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

    execute_force_control();
}

void robot_motion::control_joystick_force()
{
    control_mode = "Joystick_force";

    if(pre_control_mode != control_mode)
    {
        joy_target_pose = current_pose;
        initialize_force_control_state();
    }

    update_force_target_from_joystick();
    execute_force_control();
}

void robot_motion::main_control()
{
    if(start_flag){
        state_update();

        if(control_mode == "Idling") control_idling();
        else if (control_mode == "Position") control_position();
        else if (control_mode == "Joystick") control_joystick();
        else if (control_mode == "Joystick_force") control_joystick_force();
        else if (control_mode == "Guiding") control_guiding();
        else if (control_mode == "Force") control_force();
        else control_idling();

        state_publisher();
    }
}
