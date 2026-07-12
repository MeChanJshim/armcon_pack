#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{
int axisIndex(const std::string & axis)
{
  if (axis == "x") {return 0;}
  if (axis == "y") {return 1;}
  if (axis == "z") {return 2;}
  if (axis == "wx") {return 3;}
  if (axis == "wy") {return 4;}
  if (axis == "wz") {return 5;}
  return -1;
}
}  // namespace

class OnlineMetricsNode : public rclcpp::Node
{
public:
  OnlineMetricsNode()
  : Node("online_metrics")
  {
    command_pose_topic_ = declare_parameter<std::string>("command_pose_topic", "/ur10skku/cmdMotion");
    current_pose_topic_ = declare_parameter<std::string>("current_pose_topic", "/ur10skku/currentP");
    axis_ = declare_parameter<std::string>("axis", "z");
    report_period_sec_ = declare_parameter<double>("report_period_sec", 1.0);

    axis_index_ = axisIndex(axis_);
    if (axis_index_ < 0) {
      throw std::runtime_error("axis must be one of x, y, z, wx, wy, wz");
    }

    command_pose_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      command_pose_topic_, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        command_pose_ = msg->data;
        updateError();
      });
    current_pose_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      current_pose_topic_, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        current_pose_ = msg->data;
        updateError();
      });

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(report_period_sec_)),
      std::bind(&OnlineMetricsNode::report, this));
  }

private:
  void updateError()
  {
    if (command_pose_.size() <= static_cast<size_t>(axis_index_) ||
      current_pose_.size() <= static_cast<size_t>(axis_index_))
    {
      return;
    }
    const double error = command_pose_[axis_index_] - current_pose_[axis_index_];
    sum_sq_error_ += error * error;
    peak_abs_error_ = std::max(peak_abs_error_, std::abs(error));
    ++sample_count_;
  }

  void report()
  {
    if (sample_count_ == 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for command/current pose samples.");
      return;
    }
    const double rms_error = std::sqrt(sum_sq_error_ / static_cast<double>(sample_count_));
    RCLCPP_INFO(
      get_logger(),
      "axis=%s samples=%zu rms_error=%.6f peak_abs_error=%.6f",
      axis_.c_str(), sample_count_, rms_error, peak_abs_error_);
  }

  std::string command_pose_topic_;
  std::string current_pose_topic_;
  std::string axis_;
  double report_period_sec_ = 1.0;
  int axis_index_ = 2;

  std::vector<double> command_pose_;
  std::vector<double> current_pose_;
  double sum_sq_error_ = 0.0;
  double peak_abs_error_ = 0.0;
  size_t sample_count_ = 0;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OnlineMetricsNode>());
  rclcpp::shutdown();
  return 0;
}
