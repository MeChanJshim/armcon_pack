#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

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

class CartesianExcitationNode : public rclcpp::Node
{
public:
  CartesianExcitationNode()
  : Node("cartesian_excitation")
  {
    mode_topic_ = declare_parameter<std::string>("mode_topic", "/ur10skku/cmdMode");
    command_topic_ = declare_parameter<std::string>("command_topic", "/ur10skku/cmdMotion");
    current_pose_topic_ = declare_parameter<std::string>("current_pose_topic", "/ur10skku/currentP");
    excitation_type_ = declare_parameter<std::string>("excitation_type", "sine");
    axis_ = declare_parameter<std::string>("axis", "z");
    amplitude_ = declare_parameter<double>("amplitude", 5.0);
    frequency_hz_ = declare_parameter<double>("frequency_hz", 0.5);
    frequencies_hz_ = declare_parameter<std::vector<double>>(
      "frequencies_hz", std::vector<double>{0.1, 0.2, 0.5, 1.0, 2.0});
    cycles_per_frequency_ = declare_parameter<double>("cycles_per_frequency", 8.0);
    chirp_start_hz_ = declare_parameter<double>("chirp_start_hz", 0.1);
    chirp_end_hz_ = declare_parameter<double>("chirp_end_hz", 5.0);
    pre_hold_sec_ = declare_parameter<double>("pre_hold_sec", 2.0);
    duration_sec_ = declare_parameter<double>("duration_sec", 20.0);
    post_hold_sec_ = declare_parameter<double>("post_hold_sec", 2.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1000.0);
    mode_publish_count_ = declare_parameter<int>("mode_publish_count", 20);
    shutdown_on_finish_ = declare_parameter<bool>("shutdown_on_finish", false);

    axis_index_ = axisIndex(axis_);
    if (axis_index_ < 0) {
      throw std::runtime_error("axis must be one of x, y, z, wx, wy, wz");
    }
    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive");
    }
    if (excitation_type_ == "sine_sweep") {
      duration_sec_ = sweepDuration();
      RCLCPP_INFO(get_logger(), "sine_sweep duration set to %.3f sec", duration_sec_);
    }

    mode_pub_ = create_publisher<std_msgs::msg::String>(mode_topic_, 10);
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    current_pose_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      current_pose_topic_, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) {
          return;
        }
        if (!baseline_received_) {
          for (size_t i = 0; i < baseline_pose_.size(); ++i) {
            baseline_pose_[i] = msg->data[i];
          }
          baseline_received_ = true;
          start_time_ = now();
          RCLCPP_INFO(
            get_logger(),
            "Baseline pose received. Starting %s excitation on %s axis.",
            excitation_type_.c_str(), axis_.c_str());
        }
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CartesianExcitationNode::onTimer, this));
  }

private:
  void onTimer()
  {
    publishPositionMode();

    if (!baseline_received_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for current pose on %s", current_pose_topic_.c_str());
      return;
    }

    const double elapsed = (now() - start_time_).seconds();
    const double total_time = pre_hold_sec_ + duration_sec_ + post_hold_sec_;
    if (elapsed > total_time) {
      publishCommand(0.0);
      RCLCPP_INFO_ONCE(get_logger(), "Cartesian excitation finished.");
      timer_->cancel();
      if (shutdown_on_finish_) {
        rclcpp::shutdown();
      }
      return;
    }

    double offset = 0.0;
    const double active_time = elapsed - pre_hold_sec_;
    if (active_time >= 0.0 && active_time <= duration_sec_) {
      offset = excitationValue(active_time);
    }
    publishCommand(offset);
  }

  void publishPositionMode()
  {
    if (mode_publish_count_ <= 0 || mode_publish_sent_ >= mode_publish_count_) {
      return;
    }
    std_msgs::msg::String mode_msg;
    mode_msg.data = "Position";
    mode_pub_->publish(mode_msg);
    ++mode_publish_sent_;
  }

  double excitationValue(double t) const
  {
    if (excitation_type_ == "step") {
      return amplitude_;
    }
    if (excitation_type_ == "chirp") {
      const double ratio = duration_sec_ > 0.0 ? t / duration_sec_ : 0.0;
      const double freq = chirp_start_hz_ + (chirp_end_hz_ - chirp_start_hz_) * ratio;
      return amplitude_ * std::sin(2.0 * kPi * freq * t);
    }
    if (excitation_type_ == "sine_sweep") {
      double segment_start = 0.0;
      for (const double freq : frequencies_hz_) {
        if (freq <= 0.0) {
          continue;
        }
        const double segment_duration = cycles_per_frequency_ / freq;
        if (t <= segment_start + segment_duration) {
          return amplitude_ * std::sin(2.0 * kPi * freq * (t - segment_start));
        }
        segment_start += segment_duration;
      }
      return 0.0;
    }
    return amplitude_ * std::sin(2.0 * kPi * frequency_hz_ * t);
  }

  double sweepDuration() const
  {
    double duration = 0.0;
    for (const double freq : frequencies_hz_) {
      if (freq > 0.0) {
        duration += cycles_per_frequency_ / freq;
      }
    }
    return duration;
  }

  void publishCommand(double axis_offset)
  {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(baseline_pose_.begin(), baseline_pose_.end());
    cmd.data[axis_index_] = baseline_pose_[axis_index_] + axis_offset;
    command_pub_->publish(cmd);
  }

  std::string mode_topic_;
  std::string command_topic_;
  std::string current_pose_topic_;
  std::string excitation_type_;
  std::string axis_;
  double amplitude_ = 0.0;
  double frequency_hz_ = 0.0;
  std::vector<double> frequencies_hz_;
  double cycles_per_frequency_ = 8.0;
  double chirp_start_hz_ = 0.0;
  double chirp_end_hz_ = 0.0;
  double pre_hold_sec_ = 0.0;
  double duration_sec_ = 0.0;
  double post_hold_sec_ = 0.0;
  double publish_rate_hz_ = 1000.0;
  int mode_publish_count_ = 20;
  int mode_publish_sent_ = 0;
  bool shutdown_on_finish_ = false;
  int axis_index_ = 2;
  bool baseline_received_ = false;
  std::array<double, 6> baseline_pose_{};
  rclcpp::Time start_time_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianExcitationNode>());
  rclcpp::shutdown();
  return 0;
}
