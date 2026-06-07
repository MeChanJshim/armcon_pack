// ============================================================
// robot_command.cpp  (FULL FILE)
// - Added PTP command topic: <robot_name>/ptp_cmd  (Float64MultiArray)
//   data = [x_mm,y_mm,z_mm,wx_deg,wy_deg,wz_deg,vel_deg_s]
// - BLOCK new ptp_cmd while trajectory is running (ignored)
// - Publish state: <robot_name>/cmdState ("trajectory transfer done" at end)
// - Existing cmd_continue9D logic preserved
// ============================================================

#include "Y2RobMotion/robot_command.hpp"

#include <fstream>
#include <sstream>
#include <filesystem>
#include <thread>
#include <cmath>
#include <iostream>
#include <stdexcept>

// ------------------------------------------------------------
// robot_command::LoadTxtToYMatrix (private static)
// ------------------------------------------------------------
YMatrix robot_command::LoadTxtToYMatrix(const std::string& path, int expected_cols)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        throw std::runtime_error("Failed to open file: " + path);
    }

    std::vector<std::vector<double>> rows;
    std::string line;

    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        if (!line.empty() && line[0] == '#') continue;

        // Support comma-separated too
        for (auto& ch : line) {
            if (ch == ',') ch = ' ';
        }

        std::istringstream iss(line);
        std::vector<double> vals;
        double v;
        while (iss >> v) vals.push_back(v);

        if (vals.empty()) continue;

        if ((int)vals.size() != expected_cols) {
            std::ostringstream oss;
            oss << "Invalid column count in " << path
                << " (got " << vals.size()
                << ", expected " << expected_cols << ")";
            throw std::runtime_error(oss.str());
        }

        rows.push_back(std::move(vals));
    }

    if (rows.empty()) {
        throw std::runtime_error("No valid data rows in file: " + path);
    }

    YMatrix mat((int)rows.size(), expected_cols);
    for (int r = 0; r < (int)rows.size(); ++r) {
        for (int c = 0; c < expected_cols; ++c) {
            mat[r][c] = rows[r][c];
        }
    }
    return mat;
}

// ------------------------------------------------------------

robot_command::robot_command(rclcpp::Node* node,
                             const std::string& robot_name,
                             double control_period,
                             const PathGenParam& pg_param_)
: node_(node)
, control_period_(control_period)
, pg_param(pg_param_)          // copy
, current_position(6, 0.0)
, robot_name_(robot_name)
{
    // Topic names
    const std::string current_p_sub_topic  = robot_name_ + "/currentP";
    const std::string cmd_motion_pub_topic = robot_name_ + "/cmdMotion";
    const std::string cmd_pub_topic        = robot_name_ + "/cmdMode";

    // --- Added topics ---
    const std::string ptp_cmd_topic        = robot_name_ + "/ptp_cmd";
    const std::string state_pub_topic      = robot_name_ + "/cmdState";

    // Subscriber
    current_p_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        current_p_sub_topic, 1,
        std::bind(&robot_command::currentPCallback, this, std::placeholders::_1));

    // Publishers
    cmd_motion_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(cmd_motion_pub_topic, 1);
    cmd_pub_        = node_->create_publisher<std_msgs::msg::String>(cmd_pub_topic, 10);

    // --- Added: state publisher ---
    state_pub_      = node_->create_publisher<std_msgs::msg::String>(state_pub_topic, 10);

    // --- Added: ptp command subscriber (KeepLast(1) prevents queue buildup) ---
    auto qos_ptp = rclcpp::QoS(rclcpp::KeepLast(1));
    ptp_cmd_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        ptp_cmd_topic, qos_ptp,
        std::bind(&robot_command::ptpCmdCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "UrCmd node initialized for robot: %s", robot_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "PTP cmd topic: %s  (Float64MultiArray size=7)", ptp_cmd_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "State topic  : %s  (String)", state_pub_topic.c_str());
}

void robot_command::publish_state(const std::string& s)
{
    if (!state_pub_) return;
    std_msgs::msg::String m;
    m.data = s;
    state_pub_->publish(m);
}

void robot_command::currentPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!robot_init) {
        robot_init = true;
        RCLCPP_INFO(node_->get_logger(), "CurrentP received! Robot initialized.");
    }

    for (size_t i = 0; i < 6 && i < msg->data.size(); ++i) {
        current_position[i] = msg->data[i];
    }
}

// ------------------------------------------------------------
// PTP command by TOPIC
//   msg->data = [x_mm y_mm z_mm wx_deg wy_deg wz_deg vel_deg_s]  (size == 7)
// - If trajectory running, ignore new msg (blocked)
// ------------------------------------------------------------
void robot_command::ptpCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!robot_init) {
        publish_state("ptp rejected: robot not initialized");
        return;
    }

    if (msg->data.size() != 7) {
        publish_state("ptp rejected: invalid cmd size (need 7: x y z wx wy wz vel)");
        return;
    }

    // Block new commands while running
    if (traj_busy_.exchange(true)) {
        publish_state("ptp busy (ignored)");
        return;
    }

    const double x_mm      = msg->data[0];
    const double y_mm      = msg->data[1];
    const double z_mm      = msg->data[2];
    const double wx_deg    = msg->data[3];
    const double wy_deg    = msg->data[4];
    const double wz_deg    = msg->data[5];
    const double vel_deg_s = msg->data[6];

    // Reuse existing PTP generator contract (degrees for wx/wy/wz, and velocity in deg/s)
    pg_param.ptp_target_velocity = vel_deg_s;
    YMatrix ptp_loaded_motion = {{x_mm, y_mm, z_mm, wx_deg, wy_deg, wz_deg}};

    publish_state("ptp started");

    // Run PTP in a detached thread so we don't block ROS callbacks
    std::thread([this, ptp_loaded_motion]() mutable {
        try {
            this->PTP_command_gen(ptp_loaded_motion);
            this->publish_state("trajectory transfer done");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->node_->get_logger(), "PTP exception: %s", e.what());
            this->publish_state(std::string("ptp failed: ") + e.what());
        }
        traj_busy_ = false;
    }).detach();
}

void robot_command::sendCommand(const std::string& command, const YMatrix& loaded_motion)
{
    if (command == "PTP") {
        PTP_command_gen(loaded_motion);
    }
    else if (command == "TxtLoad") {
        TxtLoad_command_gen(loaded_motion);
    }
    else if (command == "Idling") {
        RCLCPP_INFO(node_->get_logger(), "Idling mode activated");
        cmd_msg_.data = command;
        cmd_pub_->publish(cmd_msg_);
    }
    else if (command == "Guiding") {
        RCLCPP_INFO(node_->get_logger(), "Guiding mode activated");
        cmd_msg_.data = command;
        cmd_pub_->publish(cmd_msg_);
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Unknown command: %s", command.c_str());
    }
}

YMatrix robot_command::PTP_command_input()
{
    std::vector<double> ptp_target_pose(6, 0.0);
    YMatrix ptp_loaded_motion(1, 6);

    printf("\033[33mCurrent position(mm,degree) x:%.2f, y:%.2f, z:%.2f, wx:%.2f, wy:%.2f, wz:%.2f\033[0m\n",
           current_position[0], current_position[1], current_position[2],
           RadianToDegree(current_position[3]),
           RadianToDegree(current_position[4]),
           RadianToDegree(current_position[5]));

    std::cout << "Enter target pose(8888:Same with current, 9999:Quit):\n";

    // X
    std::cout << "X(mm): ";
    std::cin >> ptp_target_pose[0];
    if (ptp_target_pose[0] == 9999) { rclcpp::shutdown(); std::exit(1); }
    else if (ptp_target_pose[0] == 8888) { ptp_target_pose[0] = current_position[0]; }

    // Y
    std::cout << "Y(mm): ";
    std::cin >> ptp_target_pose[1];
    if (ptp_target_pose[1] == 9999) { rclcpp::shutdown(); std::exit(1); }
    else if (ptp_target_pose[1] == 8888) { ptp_target_pose[1] = current_position[1]; }

    // Z
    std::cout << "Z(mm): ";
    std::cin >> ptp_target_pose[2];
    if (ptp_target_pose[2] == 9999) { rclcpp::shutdown(); std::exit(1); }
    else if (ptp_target_pose[2] == 8888) { ptp_target_pose[2] = current_position[2]; }

    // Wx
    std::cout << "Wx(degree): ";
    std::cin >> ptp_target_pose[3];
    if (ptp_target_pose[3] == 9999) { rclcpp::shutdown(); std::exit(1); }
    else if (ptp_target_pose[3] == 8888) { ptp_target_pose[3] = RadianToDegree(current_position[3]); }

    // Wy
    std::cout << "Wy(degree): ";
    std::cin >> ptp_target_pose[4];
    if (ptp_target_pose[4] == 9999) { rclcpp::shutdown(); std::exit(1); }
    else if (ptp_target_pose[4] == 8888) { ptp_target_pose[4] = RadianToDegree(current_position[4]); }

    // Wz
    std::cout << "Wz(degree): ";
    std::cin >> ptp_target_pose[5];
    if (ptp_target_pose[5] == 9999) { rclcpp::shutdown(); std::exit(1); }
    else if (ptp_target_pose[5] == 8888) { ptp_target_pose[5] = RadianToDegree(current_position[5]); }

    // Target velocity
    std::cout << "Target_vel(degree/s, 9999:Quit): ";
    std::cin >> pg_param.ptp_target_velocity;
    if (pg_param.ptp_target_velocity == 9999) { rclcpp::shutdown(); std::exit(1); }

    std::cout << "Target pose: [" << ptp_target_pose[0] << ", " << ptp_target_pose[1]
              << ", " << ptp_target_pose[2] << ", " << ptp_target_pose[3]
              << ", " << ptp_target_pose[4] << ", " << ptp_target_pose[5] << "]\n";

    ptp_loaded_motion = {{ptp_target_pose[0], ptp_target_pose[1], ptp_target_pose[2],
                          ptp_target_pose[3], ptp_target_pose[4], ptp_target_pose[5]}};

    return ptp_loaded_motion;
}

void robot_command::PTP_command_gen(const YMatrix& loaded_motion)
{
    cmd_msg_.data = "Position";
    cmd_pub_->publish(cmd_msg_);

    YMatrix position = {
        {current_position[0], current_position[1], current_position[2],
         current_position[3], current_position[4], current_position[5]},
        {loaded_motion[0][0], loaded_motion[0][1], loaded_motion[0][2],
         DegreeToRadian(loaded_motion[0][3]),
         DegreeToRadian(loaded_motion[0][4]),
         DegreeToRadian(loaded_motion[0][5])}
    };

    std::vector<double> velocity      = {0.0, pg_param.ptp_target_velocity};
    std::vector<double> ang_velocity  = {0.0, 0.0};
    std::vector<double> holding_time  = {0.0, 0.0};

    MotionBlender6D blender(position, velocity, ang_velocity, holding_time,
                            DegreeToRadian(pg_param.angularVelocityLimit),
                            pg_param.startingTime, pg_param.lastRestingTime,
                            pg_param.accelerationTime, control_period_);

    YMatrix blended_motion = blender.blendMotion(pg_param.defualt_travelTime);

    printf("Path Transferring...\n");

    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(control_period_)
    );
    auto next_time = std::chrono::steady_clock::now();

    for (const auto& pos : blended_motion) {
        if (!rclcpp::ok()) {
            RCLCPP_WARN(node_->get_logger(), "ROS is not ok, stopping motion transfer.");
            return;
        }

        cmd_motion_msg_.data.clear();
        for (const auto& v : pos) cmd_motion_msg_.data.push_back(v);
        cmd_motion_pub_->publish(cmd_motion_msg_);

        next_time += period_ns;
        std::this_thread::sleep_until(next_time);
    }

    printf("Path Transferred.\n");
}

void robot_command::TxtLoad_command_gen(const YMatrix& loaded_motion)
{
    // ============================================================
    // cmd_6D
    // ============================================================
    if (pg_param.loadFileType == "cmd_6D") {

        cmd_msg_.data = "Position";
        cmd_pub_->publish(cmd_msg_);

        printf("\033[32mLoading 6D motion profile from %s...\033[0m\n", pg_param.loadFileType.c_str());
        loaded_motion.print();

        if (loaded_motion.cols() != 9) {
            printf("\033[33mthere is not proper setup inside of cmd_6D !! \033[0m\n");
            return;
        }

        YMatrix loaded_position(loaded_motion.rows() + 2, 6);
        std::vector<double> loaded_velocity(loaded_motion.rows() + 2, 0.0);
        std::vector<double> loaded_ang_velocity(loaded_motion.rows() + 2, 0.0);
        std::vector<double> loaded_holding_time(loaded_motion.rows() + 2, 0.0);

        loaded_position.insert(1, 0, loaded_motion.extract(0, 0, loaded_motion.rows(), 6));
        auto loaded_velocity_matrix     = loaded_motion.extract(0, 6, loaded_motion.rows(), 1);
        auto loaded_ang_velocity_matrix = loaded_motion.extract(0, 7, loaded_motion.rows(), 1);
        auto loaded_holding_time_matrix = loaded_motion.extract(0, 8, loaded_motion.rows(), 1);

        for (int i = 0; i < (int)loaded_velocity_matrix.rows(); i++) {
            loaded_velocity[i + 1]      = loaded_velocity_matrix[i][0];
            loaded_ang_velocity[i + 1]  = loaded_ang_velocity_matrix[i][0];
            loaded_holding_time[i + 1]  = loaded_holding_time_matrix[i][0];
        }

        // Convert degrees to radians (wx,wy,wz)
        for (int i = 0; i < (int)loaded_position.rows(); i++) {
            for (int j = 0; j < (int)loaded_position.cols(); j++) {
                if (j >= 3) loaded_position[i][j] = DegreeToRadian(loaded_position[i][j]);
            }
        }

        // Insert start & end pose
        loaded_position.insert(0, 0, {{current_position[0], current_position[1], current_position[2],
                                       current_position[3], current_position[4], current_position[5]}});
        loaded_position.insert(loaded_position.rows() - 1, 0, {{current_position[0], current_position[1], current_position[2],
                                                                current_position[3], current_position[4], current_position[5]}});

        printf("Loaded Position:\n");
        loaded_position.print();

        // Insert start & end (velocity & angular_velocity & holding_time)
        loaded_velocity[1] = pg_param.initialTransferSpeed;
        loaded_velocity.back() = pg_param.initialTransferSpeed;

        loaded_ang_velocity[1] = 0.0;
        loaded_ang_velocity.back() = 0.0;

        loaded_holding_time[1] = 0.0;
        loaded_holding_time.back() = 0.0;

        MotionBlender6D blender(loaded_position, loaded_velocity, loaded_ang_velocity, loaded_holding_time,
                                DegreeToRadian(pg_param.angularVelocityLimit),
                                pg_param.startingTime, pg_param.lastRestingTime,
                                pg_param.accelerationTime, control_period_);

        YMatrix blended_motion = blender.blendMotion(pg_param.defualt_travelTime);
        printf("Blended motion generated\n");

        printf("Path Transferring...\n");

        auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(control_period_)
        );
        auto next_time = std::chrono::steady_clock::now();

        for (const auto& pos : blended_motion) {
            if (!rclcpp::ok()) {
                RCLCPP_WARN(node_->get_logger(), "ROS is not ok, stopping motion transfer.");
                return;
            }

            cmd_motion_msg_.data.clear();
            for (const auto& v : pos) cmd_motion_msg_.data.push_back(v);
            cmd_motion_pub_->publish(cmd_motion_msg_);

            next_time += period_ns;
            std::this_thread::sleep_until(next_time);
        }

        printf("Path Transferred.\n");
    }

    // ============================================================
    // cmd_9D  (generate + save cmd_continue9D)
    // ============================================================
    else if (pg_param.loadFileType == "cmd_9D") {

        cmd_msg_.data = "Force";
        cmd_pub_->publish(cmd_msg_);

        printf("\033[32mLoading 9D motion profile from %s...\033[0m\n", pg_param.loadFileType.c_str());
        loaded_motion.print();

        if (loaded_motion.cols() != 12) {
            printf("\033[33mthere is not proper setup inside of cmd_9D !! \033[0m\n");
            return;
        }

        YMatrix loaded_position(loaded_motion.rows() + 2, 9);
        std::vector<double> loaded_velocity(loaded_motion.rows() + 2, 0.0);
        std::vector<double> loaded_ang_velocity(loaded_motion.rows() + 2, 0.0);
        std::vector<double> loaded_holding_time(loaded_motion.rows() + 2, 0.0);

        loaded_position.insert(1, 0, loaded_motion.extract(0, 0, loaded_motion.rows(), 9));
        auto loaded_velocity_matrix     = loaded_motion.extract(0, 9,  loaded_motion.rows(), 1);
        auto loaded_ang_velocity_matrix = loaded_motion.extract(0, 10, loaded_motion.rows(), 1);
        auto loaded_holding_time_matrix = loaded_motion.extract(0, 11, loaded_motion.rows(), 1);

        for (int i = 0; i < (int)loaded_velocity_matrix.rows(); i++) {
            loaded_velocity[i + 1]      = loaded_velocity_matrix[i][0];
            loaded_ang_velocity[i + 1]  = loaded_ang_velocity_matrix[i][0];
            loaded_holding_time[i + 1]  = loaded_holding_time_matrix[i][0];
        }

        // Convert degrees to radians only for wx,wy,wz (3..5)
        for (int i = 0; i < (int)loaded_position.rows(); i++) {
            for (int j = 0; j < (int)loaded_position.cols(); j++) {
                if (j >= 3 && j < 6) loaded_position[i][j] = DegreeToRadian(loaded_position[i][j]);
            }
        }

        // Insert start & end pose (pose part; force part is already included in 9 cols)
        loaded_position.insert(0, 0, {{current_position[0], current_position[1], current_position[2],
                                       current_position[3], current_position[4], current_position[5]}});
        loaded_position.insert(loaded_position.rows() - 1, 0, {{current_position[0], current_position[1], current_position[2],
                                                                current_position[3], current_position[4], current_position[5]}});

        printf("Loaded Position:\n");
        loaded_position.print();

        loaded_velocity[1] = pg_param.initialTransferSpeed;
        loaded_velocity.back() = pg_param.initialTransferSpeed;

        loaded_ang_velocity[1] = 0.0;
        loaded_ang_velocity.back() = 0.0;

        loaded_holding_time[1] = 0.0;
        loaded_holding_time.back() = 0.0;

        MotionBlender9D blender(loaded_position, loaded_velocity, loaded_ang_velocity, loaded_holding_time,
                                DegreeToRadian(pg_param.angularVelocityLimit),
                                pg_param.startingTime, pg_param.lastRestingTime,
                                pg_param.accelerationTime, control_period_);

        YMatrix blended_motion = blender.blendMotion(pg_param.defualt_travelTime);
        printf("Blended motion generated\n");

        // Save cmd_continue9D.txt
        std::filesystem::path source_path(__FILE__);
        std::string save_path = source_path.parent_path().string() + "/../txtcmd/cmd_continue9D.txt";
        blended_motion.saveToFile(save_path);
        RCLCPP_INFO(node_->get_logger(), "9D data was saved to cmd_continue9D: %s", save_path.c_str());

        // Transfer to robot
        printf("Path Transferring...\n");

        auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(control_period_)
        );
        auto next_time = std::chrono::steady_clock::now();

        for (const auto& row : blended_motion) {
            if (!rclcpp::ok()) {
                RCLCPP_WARN(node_->get_logger(), "ROS is not ok, stopping motion transfer.");
                return;
            }

            cmd_motion_msg_.data.clear();
            for (const auto& v : row) cmd_motion_msg_.data.push_back(v);
            cmd_motion_pub_->publish(cmd_motion_msg_);

            next_time += period_ns;
            std::this_thread::sleep_until(next_time);
        }

        printf("Path Transferred.\n");
    }

    // ============================================================
    // cmd_continue6D (not implemented)
    // ============================================================
    else if (pg_param.loadFileType == "cmd_continue6D") {
        RCLCPP_WARN(node_->get_logger(), "cmd_continue6D motion profile not implemented yet");
    }

    // ============================================================
    // cmd_continue9D (LOAD + CONNECT + REPLAY)
    // ============================================================
    else if (pg_param.loadFileType == "cmd_continue9D") {

        // Load stored 9D: x y z wx wy wz fx fy fz
        std::filesystem::path source_path(__FILE__);
        const std::string cont_path = source_path.parent_path().string() + "/../txtcmd/cmd_continue9D.txt";

        YMatrix cont_motion = robot_command::LoadTxtToYMatrix(cont_path, 9);

        // first pose (rad)
        std::vector<double> first_pose(6, 0.0);
        for (int i = 0; i < 6; ++i) first_pose[i] = cont_motion[0][i];

        // Safety thresholds
        const double MAX_TRANS_MM = 200.0;
        const double MAX_ROT_DEG  = 30.0;

        const double dx = first_pose[0] - current_position[0];
        const double dy = first_pose[1] - current_position[1];
        const double dz = first_pose[2] - current_position[2];
        const double trans_norm = std::sqrt(dx*dx + dy*dy + dz*dz);

        const double dwx_deg = RadianToDegree(first_pose[3] - current_position[3]);
        const double dwy_deg = RadianToDegree(first_pose[4] - current_position[4]);
        const double dwz_deg = RadianToDegree(first_pose[5] - current_position[5]);
        const double rot_norm_deg = std::sqrt(dwx_deg*dwx_deg + dwy_deg*dwy_deg + dwz_deg*dwz_deg);

        if (trans_norm > MAX_TRANS_MM || rot_norm_deg > MAX_ROT_DEG) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Safety abort: current->first pose gap too large. trans=%.2fmm, rot=%.2fdeg",
                         trans_norm, rot_norm_deg);
            RCLCPP_ERROR(node_->get_logger(), "Move robot closer to the start pose, then retry.");
            return;
        }

        // 1) Connect path (Position mode)
        cmd_msg_.data = "Position";
        cmd_pub_->publish(cmd_msg_);

        YMatrix connect_position = {
            {current_position[0], current_position[1], current_position[2],
             current_position[3], current_position[4], current_position[5]},
            {first_pose[0], first_pose[1], first_pose[2],
             first_pose[3], first_pose[4], first_pose[5]}
        };

        // Mirror your "Insert start & end pose" concept: 2-point blend
        std::vector<double> connect_velocity     = {0.0, pg_param.initialTransferSpeed};
        std::vector<double> connect_ang_velocity = {0.0, 0.0};
        std::vector<double> connect_holding_time = {0.0, 0.0};

        MotionBlender6D connect_blender(connect_position,
                                       connect_velocity,
                                       connect_ang_velocity,
                                       connect_holding_time,
                                       DegreeToRadian(pg_param.angularVelocityLimit),
                                       pg_param.startingTime,
                                       pg_param.lastRestingTime,
                                       pg_param.accelerationTime,
                                       control_period_);

        YMatrix connect_motion = connect_blender.blendMotion(pg_param.defualt_travelTime);

        auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(control_period_)
        );
        auto next_time = std::chrono::steady_clock::now();

        RCLCPP_INFO(node_->get_logger(), "Connecting current pose -> first point (Position mode)...");

        for (const auto& pos6 : connect_motion) {
            if (!rclcpp::ok()) {
                RCLCPP_WARN(node_->get_logger(), "ROS not ok, stop connecting path.");
                return;
            }

            cmd_motion_msg_.data.clear();
            for (const auto& v : pos6) cmd_motion_msg_.data.push_back(v);
            cmd_motion_pub_->publish(cmd_motion_msg_);

            next_time += period_ns;
            std::this_thread::sleep_until(next_time);
        }

        // 2) Replay 9D in Force mode
        cmd_msg_.data = "Force";
        cmd_pub_->publish(cmd_msg_);

        RCLCPP_INFO(node_->get_logger(), "Replaying cmd_continue9D (Force mode)...");

        next_time = std::chrono::steady_clock::now();
        for (const auto& row9 : cont_motion) {
            if (!rclcpp::ok()) {
                RCLCPP_WARN(node_->get_logger(), "ROS not ok, stop continue9D replay.");
                return;
            }

            cmd_motion_msg_.data.clear();
            for (const auto& v : row9) cmd_motion_msg_.data.push_back(v);
            cmd_motion_pub_->publish(cmd_motion_msg_);

            next_time += period_ns;
            std::this_thread::sleep_until(next_time);
        }

        RCLCPP_INFO(node_->get_logger(), "cmd_continue9D finished.");
    }

    else {
        RCLCPP_ERROR(node_->get_logger(), "Unknown file type: %s", pg_param.loadFileType.c_str());
    }
}
