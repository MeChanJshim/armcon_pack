// ============================================================
// robot_command.hpp  (FULL FILE)
// - Added PTP command by topic:  <robot_name>/ptp_cmd  (Float64MultiArray)
//     data = [x_mm, y_mm, z_mm, wx_deg, wy_deg, wz_deg, vel_deg_s]  (size==7)
// - While executing a trajectory, new ptp_cmd messages are BLOCKED (ignored)
// - Publish state: <robot_name>/cmdState  (String)
//     "ptp rejected: ...", "ptp started", "ptp busy (ignored)", "trajectory transfer done", "ptp failed: ..."
// ============================================================

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <atomic>
#include <thread>

#include "Y2Matrix/YMatrix.hpp"
#include "Y2Trajectory/MotionBlender6D.hpp"
#include "Y2Trajectory/MotionBlender9D.hpp"

// ---- Safer math helpers (no macros) ----
constexpr double kPi = 3.14159265358979323846;
inline double DegreeToRadian(double degree) { return degree * kPi / 180.0; }
inline double RadianToDegree(double rad)    { return rad * 180.0 / kPi; }

struct PathGenParam {
    double defualt_travelTime = 5.0;        // seconds (keep name)
    double initialTransferSpeed = 5.0;      // mm/s
    double angularVelocityLimit = 5.0;      // degrees/s
    double accelerationTime = 1.0;          // seconds
    double startingTime = 2.0;              // seconds
    double lastRestingTime = 2.0;           // seconds
    double ptp_target_velocity = 0.5;       // degrees/s
    std::string loadFileType = "cmd_9D";    // cmd_6D, cmd_9D, cmd_continue6D, cmd_continue9D
};

class robot_command
{
public:
    robot_command(rclcpp::Node* node,
                  const std::string& robot_name,
                  double control_period,
                  const PathGenParam& pg_param_);

    void sendCommand(const std::string& command, const YMatrix& loaded_motion);
    YMatrix PTP_command_input();
    void PTP_command_gen(const YMatrix& loaded_motion);
    void TxtLoad_command_gen(const YMatrix& loaded_motion);

    // Public members
    std::vector<double> current_position;   // [x y z wx wy wz] (mm, rad)
    bool robot_init = false;
    PathGenParam pg_param;                  // stored copy
    std::string robot_name_;

private:
    rclcpp::Node* node_;

    // ⚠️ 선언 순서가 "초기화 순서"가 됨 (Wreorder 방지 위해 아래 순서 유지)
    std::vector<double> dummy_; // (사용 안하면 제거 가능; 여기선 아무것도 안함)

    double control_period_;

    // Publishers / Subscribers (existing)
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_motion_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_p_sub_;

    // --- Added: PTP command by topic + state publisher ---
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ptp_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    std::atomic<bool> traj_busy_{false};

    // Messages
    std_msgs::msg::String cmd_msg_;
    std_msgs::msg::Float64MultiArray cmd_motion_msg_;

    void currentPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // --- Added callbacks/helpers ---
    void ptpCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void publish_state(const std::string& s);

    // Helper inside class
    static YMatrix LoadTxtToYMatrix(const std::string& path, int expected_cols);
};
