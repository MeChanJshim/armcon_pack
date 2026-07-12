#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{
std::string timestampString()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&time, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

template<typename T>
std::string joinVector(const std::vector<T> & values)
{
  std::ostringstream oss;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << ';';
    }
    oss << values[i];
  }
  return oss.str();
}

std::string quoted(const std::string & value)
{
  return "\"" + value + "\"";
}
}  // namespace

class ResponseRecorderNode : public rclcpp::Node
{
public:
  ResponseRecorderNode()
  : Node("response_recorder")
  {
    output_dir_ = declare_parameter<std::string>(
      "output_dir", "/home/jay/armcon_ws/src/armcon_pack/Y2SYS_ID/results");
    experiment_name_ = declare_parameter<std::string>("experiment_name", "cartesian_id");
    record_rate_hz_ = declare_parameter<double>("record_rate_hz", 1000.0);

    command_pose_topic_ = declare_parameter<std::string>("command_pose_topic", "/ur10skku/cmdMotion");
    current_pose_topic_ = declare_parameter<std::string>("current_pose_topic", "/ur10skku/currentP");
    target_pose_topic_ = declare_parameter<std::string>("target_pose_topic", "/ur10skku/targetP");
    current_joint_topic_ = declare_parameter<std::string>("current_joint_topic", "/ur10skku/currentJ");
    target_joint_topic_ = declare_parameter<std::string>("target_joint_topic", "/ur10skku/targetJ");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/armsimul/joint_states");
    wrench_topic_ = declare_parameter<std::string>("wrench_topic", "/armsimul/ee_wrench");

    openOutputFile();
    createSubscriptions();

    const auto period = std::chrono::duration<double>(1.0 / record_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ResponseRecorderNode::writeRow, this));
  }

private:
  void openOutputFile()
  {
    std::filesystem::create_directories(output_dir_);
    output_path_ = std::filesystem::path(output_dir_) /
      (experiment_name_ + "_" + timestampString() + ".csv");
    file_.open(output_path_);
    if (!file_.is_open()) {
      throw std::runtime_error("Failed to open output CSV: " + output_path_.string());
    }

    file_ <<
      "time_sec,"
      "command_pose,"
      "current_pose,"
      "target_pose,"
      "current_joint,"
      "target_joint,"
      "joint_state_names,"
      "joint_state_position,"
      "joint_state_velocity,"
      "joint_state_effort,"
      "wrench_force,"
      "wrench_torque\n";

    RCLCPP_INFO(get_logger(), "Recording Y2SYS_ID data to %s", output_path_.c_str());
  }

  void createSubscriptions()
  {
    command_pose_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      command_pose_topic_, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {command_pose_ = msg->data;});
    current_pose_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      current_pose_topic_, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {current_pose_ = msg->data;});
    target_pose_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      target_pose_topic_, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {target_pose_ = msg->data;});
    current_joint_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      current_joint_topic_, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {current_joint_ = msg->data;});
    target_joint_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      target_joint_topic_, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {target_joint_ = msg->data;});
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_state_names_ = msg->name;
        joint_state_position_ = msg->position;
        joint_state_velocity_ = msg->velocity;
        joint_state_effort_ = msg->effort;
      });
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_topic_, 10,
      [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        wrench_force_ = {
          msg->wrench.force.x,
          msg->wrench.force.y,
          msg->wrench.force.z};
        wrench_torque_ = {
          msg->wrench.torque.x,
          msg->wrench.torque.y,
          msg->wrench.torque.z};
      });
  }

  void writeRow()
  {
    if (!file_.is_open()) {
      return;
    }
    const double t = now().seconds();
    file_ << std::fixed << std::setprecision(9) << t << ','
          << quoted(joinVector(command_pose_)) << ','
          << quoted(joinVector(current_pose_)) << ','
          << quoted(joinVector(target_pose_)) << ','
          << quoted(joinVector(current_joint_)) << ','
          << quoted(joinVector(target_joint_)) << ','
          << quoted(joinVector(joint_state_names_)) << ','
          << quoted(joinVector(joint_state_position_)) << ','
          << quoted(joinVector(joint_state_velocity_)) << ','
          << quoted(joinVector(joint_state_effort_)) << ','
          << quoted(joinVector(wrench_force_)) << ','
          << quoted(joinVector(wrench_torque_)) << '\n';
  }

  std::string output_dir_;
  std::string experiment_name_;
  double record_rate_hz_ = 1000.0;
  std::string command_pose_topic_;
  std::string current_pose_topic_;
  std::string target_pose_topic_;
  std::string current_joint_topic_;
  std::string target_joint_topic_;
  std::string joint_state_topic_;
  std::string wrench_topic_;
  std::filesystem::path output_path_;
  std::ofstream file_;

  std::vector<double> command_pose_;
  std::vector<double> current_pose_;
  std::vector<double> target_pose_;
  std::vector<double> current_joint_;
  std::vector<double> target_joint_;
  std::vector<std::string> joint_state_names_;
  std::vector<double> joint_state_position_;
  std::vector<double> joint_state_velocity_;
  std::vector<double> joint_state_effort_;
  std::vector<double> wrench_force_;
  std::vector<double> wrench_torque_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ResponseRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
