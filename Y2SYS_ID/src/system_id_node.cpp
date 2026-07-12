#include <algorithm>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

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

double mean(const std::vector<double> & values)
{
  if (values.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const double value : values) {
    sum += value;
  }
  return sum / static_cast<double>(values.size());
}

double rms(const std::vector<double> & values)
{
  if (values.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const double value : values) {
    sum += value * value;
  }
  return std::sqrt(sum / static_cast<double>(values.size()));
}

double wrapPhase(double phase)
{
  while (phase > kPi) {phase -= 2.0 * kPi;}
  while (phase < -kPi) {phase += 2.0 * kPi;}
  return phase;
}

struct FrequencyResult
{
  double frequency_hz = 0.0;
  double command_amplitude = 0.0;
  double response_amplitude = 0.0;
  double gain = 0.0;
  double phase_lag_rad = 0.0;
};
}  // namespace

class SystemIdNode : public rclcpp::Node
{
public:
  SystemIdNode()
  : Node("system_id")
  {
    // This node is an analyzer, not a controller. It never publishes commands.
    // It samples cmdMotion/currentP, estimates ID metrics, writes a YAML result,
    // and exits after the configured experiment time so launch can shut down.
    command_pose_topic_ = declare_parameter<std::string>("command_pose_topic", "/ur10skku/cmdMotion");
    current_pose_topic_ = declare_parameter<std::string>("current_pose_topic", "/ur10skku/currentP");
    output_dir_ = declare_parameter<std::string>(
      "output_dir", "/home/jay/armcon_ws/src/armcon_pack/Y2SYS_ID/results");
    experiment_name_ = declare_parameter<std::string>("experiment_name", "cartesian_id");
    excitation_type_ = declare_parameter<std::string>("excitation_type", "sine");
    axis_ = declare_parameter<std::string>("axis", "z");
    frequency_hz_ = declare_parameter<double>("frequency_hz", 0.5);
    frequencies_hz_ = declare_parameter<std::vector<double>>(
      "frequencies_hz", std::vector<double>{0.1, 0.2, 0.5, 1.0, 2.0});
    cycles_per_frequency_ = declare_parameter<double>("cycles_per_frequency", 8.0);
    settling_cycles_ = declare_parameter<double>("settling_cycles", 2.0);
    pre_hold_sec_ = declare_parameter<double>("pre_hold_sec", 2.0);
    duration_sec_ = declare_parameter<double>("duration_sec", 20.0);
    post_hold_sec_ = declare_parameter<double>("post_hold_sec", 2.0);
    finish_margin_sec_ = declare_parameter<double>("finish_margin_sec", 0.5);
    auto_shutdown_ = declare_parameter<bool>("auto_shutdown", true);

    axis_index_ = axisIndex(axis_);
    if (axis_index_ < 0) {
      throw std::runtime_error("axis must be one of x, y, z, wx, wy, wz");
    }
    if (excitation_type_ == "sine_sweep") {
      duration_sec_ = sweepDuration();
      RCLCPP_INFO(get_logger(), "sine_sweep ID duration set to %.3f sec", duration_sec_);
    }

    command_pose_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      command_pose_topic_, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() > static_cast<size_t>(axis_index_)) {
          latest_command_ = msg->data[axis_index_];
          command_received_ = true;
          sampleIfReady();
        }
      });
    current_pose_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      current_pose_topic_, 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() > static_cast<size_t>(axis_index_)) {
          latest_response_ = msg->data[axis_index_];
          response_received_ = true;
          sampleIfReady();
        }
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SystemIdNode::checkFinish, this));
  }

private:
  void sampleIfReady()
  {
    if (!command_received_ || !response_received_) {
      return;
    }
    if (!started_) {
      started_ = true;
      start_time_ = now();
      RCLCPP_INFO(get_logger(), "System ID sampling started.");
    }
    const double t = (now() - start_time_).seconds();
    times_.push_back(t);
    commands_.push_back(latest_command_);
    responses_.push_back(latest_response_);
  }

  void checkFinish()
  {
    if (!started_ || finished_) {
      return;
    }
    const double total_time = pre_hold_sec_ + duration_sec_ + post_hold_sec_ + finish_margin_sec_;
    const double elapsed = (now() - start_time_).seconds();
    if (elapsed < total_time) {
      return;
    }
    finished_ = true;
    writeResult();
    if (auto_shutdown_) {
      rclcpp::shutdown();
    }
  }

  std::vector<size_t> activeIndices() const
  {
    std::vector<size_t> indices;
    for (size_t i = 0; i < times_.size(); ++i) {
      if (times_[i] >= pre_hold_sec_ && times_[i] <= pre_hold_sec_ + duration_sec_) {
        indices.push_back(i);
      }
    }
    return indices;
  }

  void writeResult()
  {
    std::filesystem::create_directories(output_dir_);
    const auto output_path = std::filesystem::path(output_dir_) /
      (experiment_name_ + "_id_" + timestampString() + ".yaml");
    std::ofstream file(output_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to write ID result: %s", output_path.c_str());
      return;
    }

    const auto indices = activeIndices();
    std::vector<double> active_command;
    std::vector<double> active_response;
    std::vector<double> active_error;
    active_command.reserve(indices.size());
    active_response.reserve(indices.size());
    active_error.reserve(indices.size());

    for (const size_t index : indices) {
      active_command.push_back(commands_[index]);
      active_response.push_back(responses_[index]);
      active_error.push_back(commands_[index] - responses_[index]);
    }

    const auto minmax_command = std::minmax_element(active_command.begin(), active_command.end());
    const auto minmax_response = std::minmax_element(active_response.begin(), active_response.end());
    const double command_span = active_command.empty() ? 0.0 : *minmax_command.second - *minmax_command.first;
    const double response_span = active_response.empty() ? 0.0 : *minmax_response.second - *minmax_response.first;
    const double span_gain = std::abs(command_span) > 1e-12 ? response_span / command_span : 0.0;
    const double mean_command = mean(active_command);
    const double mean_response = mean(active_response);
    const double mean_error_bias = mean(active_error);
    std::vector<double> active_error_bias_corrected;
    active_error_bias_corrected.reserve(active_error.size());
    for (const double error : active_error) {
      active_error_bias_corrected.push_back(error - mean_error_bias);
    }
    const double rms_error_raw = rms(active_error);
    const double peak_abs_error_raw = active_error.empty() ? 0.0 :
      std::abs(*std::max_element(
        active_error.begin(), active_error.end(),
        [](double a, double b) {return std::abs(a) < std::abs(b);}));
    const double rms_error_bias_corrected = rms(active_error_bias_corrected);
    const double peak_abs_error_bias_corrected = active_error_bias_corrected.empty() ? 0.0 :
      std::abs(*std::max_element(
        active_error_bias_corrected.begin(), active_error_bias_corrected.end(),
        [](double a, double b) {return std::abs(a) < std::abs(b);}));

    file << "experiment_name: " << experiment_name_ << "\n";
    file << "excitation_type: " << excitation_type_ << "\n";
    file << "axis: " << axis_ << "\n";
    file << "samples_total: " << times_.size() << "\n";
    file << "samples_active: " << active_command.size() << "\n";
    file << "pre_hold_sec: " << pre_hold_sec_ << "\n";
    file << "duration_sec: " << duration_sec_ << "\n";
    file << "post_hold_sec: " << post_hold_sec_ << "\n";
    file << "command_span: " << command_span << "\n";
    file << "response_span: " << response_span << "\n";
    file << "span_gain: " << span_gain << "\n";
    file << "mean_command: " << mean_command << "\n";
    file << "mean_response: " << mean_response << "\n";
    file << "mean_error_bias: " << mean_error_bias << "\n";
    file << "bias_corrected: true\n";
    file << "rms_error_raw: " << rms_error_raw << "\n";
    file << "peak_abs_error_raw: " << peak_abs_error_raw << "\n";
    file << "rms_error_bias_corrected: " << rms_error_bias_corrected << "\n";
    file << "peak_abs_error_bias_corrected: " << peak_abs_error_bias_corrected << "\n";

    if (excitation_type_ == "sine") {
      writeSineResult(file, indices);
    } else if (excitation_type_ == "sine_sweep") {
      writeSineSweepResult(file);
    } else if (excitation_type_ == "step") {
      writeStepResult(file, indices);
    } else {
      file << "note: chirp result currently reports tracking summary only\n";
    }

    RCLCPP_INFO(get_logger(), "System ID result written: %s", output_path.c_str());
  }

  void writeSineResult(std::ofstream & file, const std::vector<size_t> & indices) const
  {
    if (indices.empty() || frequency_hz_ <= 0.0) {
      return;
    }

    std::complex<double> command_coeff{0.0, 0.0};
    std::complex<double> response_coeff{0.0, 0.0};
    const double command_mean = mean(commands_);
    const double response_mean = mean(responses_);

    for (const size_t index : indices) {
      const double t = times_[index] - pre_hold_sec_;
      const std::complex<double> basis =
        std::exp(std::complex<double>(0.0, -2.0 * kPi * frequency_hz_ * t));
      command_coeff += (commands_[index] - command_mean) * basis;
      response_coeff += (responses_[index] - response_mean) * basis;
    }

    const double scale = 2.0 / static_cast<double>(indices.size());
    const double command_amp = std::abs(command_coeff) * scale;
    const double response_amp = std::abs(response_coeff) * scale;
    const double frequency_gain = command_amp > 1e-12 ? response_amp / command_amp : 0.0;
    double phase_lag_rad = wrapPhase(std::arg(response_coeff) - std::arg(command_coeff));

    file << "sine_frequency_hz: " << frequency_hz_ << "\n";
    file << "sine_command_amplitude: " << command_amp << "\n";
    file << "sine_response_amplitude: " << response_amp << "\n";
    file << "sine_gain: " << frequency_gain << "\n";
    file << "sine_phase_lag_rad: " << phase_lag_rad << "\n";
    file << "sine_phase_lag_deg: " << phase_lag_rad * 180.0 / kPi << "\n";
  }

  FrequencyResult estimateFrequency(
    double frequency_hz,
    double start_time,
    double end_time,
    double settling_time) const
  {
    FrequencyResult result;
    result.frequency_hz = frequency_hz;
    if (frequency_hz <= 0.0) {
      return result;
    }

    std::vector<size_t> indices;
    for (size_t i = 0; i < times_.size(); ++i) {
      if (times_[i] >= start_time + settling_time && times_[i] <= end_time) {
        indices.push_back(i);
      }
    }
    if (indices.empty()) {
      return result;
    }

    std::vector<double> command_values;
    std::vector<double> response_values;
    command_values.reserve(indices.size());
    response_values.reserve(indices.size());
    for (const size_t index : indices) {
      command_values.push_back(commands_[index]);
      response_values.push_back(responses_[index]);
    }

    const double command_mean = mean(command_values);
    const double response_mean = mean(response_values);
    std::complex<double> command_coeff{0.0, 0.0};
    std::complex<double> response_coeff{0.0, 0.0};

    for (const size_t index : indices) {
      const double local_t = times_[index] - start_time;
      const std::complex<double> basis =
        std::exp(std::complex<double>(0.0, -2.0 * kPi * frequency_hz * local_t));
      command_coeff += (commands_[index] - command_mean) * basis;
      response_coeff += (responses_[index] - response_mean) * basis;
    }

    const double scale = 2.0 / static_cast<double>(indices.size());
    result.command_amplitude = std::abs(command_coeff) * scale;
    result.response_amplitude = std::abs(response_coeff) * scale;
    result.gain = result.command_amplitude > 1e-12 ?
      result.response_amplitude / result.command_amplitude : 0.0;
    result.phase_lag_rad = wrapPhase(std::arg(response_coeff) - std::arg(command_coeff));
    return result;
  }

  void writeSineSweepResult(std::ofstream & file) const
  {
    std::vector<FrequencyResult> results;
    double segment_start = pre_hold_sec_;
    for (const double freq : frequencies_hz_) {
      if (freq <= 0.0) {
        continue;
      }
      const double segment_duration = cycles_per_frequency_ / freq;
      const double settling_time = settling_cycles_ / freq;
      const auto result = estimateFrequency(
        freq, segment_start, segment_start + segment_duration, settling_time);
      results.push_back(result);
      segment_start += segment_duration;
    }

    file << "sine_sweep:\n";
    for (const auto & result : results) {
      file << "  - frequency_hz: " << result.frequency_hz << "\n";
      file << "    command_amplitude: " << result.command_amplitude << "\n";
      file << "    response_amplitude: " << result.response_amplitude << "\n";
      file << "    gain: " << result.gain << "\n";
      file << "    phase_lag_rad: " << result.phase_lag_rad << "\n";
      file << "    phase_lag_deg: " << result.phase_lag_rad * 180.0 / kPi << "\n";
    }

    const double dc_gain = results.empty() ? 0.0 : results.front().gain;
    const double bandwidth_hz = estimateBandwidth(results, dc_gain);
    const auto model = fitSecondOrderModel(results, dc_gain);

    file << "estimated_dc_gain: " << dc_gain << "\n";
    file << "estimated_bandwidth_hz: " << bandwidth_hz << "\n";
    file << "estimated_natural_frequency_hz: " << model.natural_frequency_hz << "\n";
    file << "estimated_natural_frequency_rad_s: " << model.natural_frequency_hz * 2.0 * kPi << "\n";
    file << "estimated_damping_ratio: " << model.damping_ratio << "\n";
    file << "estimated_delay_sec: " << model.delay_sec << "\n";
    file << "second_order_fit_cost: " << model.cost << "\n";
  }

  double estimateBandwidth(const std::vector<FrequencyResult> & results, double dc_gain) const
  {
    if (results.size() < 2 || dc_gain <= 0.0) {
      return -1.0;
    }
    const double threshold = dc_gain / std::sqrt(2.0);
    for (size_t i = 1; i < results.size(); ++i) {
      if (results[i].gain <= threshold) {
        const double f0 = results[i - 1].frequency_hz;
        const double f1 = results[i].frequency_hz;
        const double g0 = results[i - 1].gain;
        const double g1 = results[i].gain;
        if (std::abs(g1 - g0) < 1e-12) {
          return f1;
        }
        const double ratio = (threshold - g0) / (g1 - g0);
        return f0 + ratio * (f1 - f0);
      }
    }
    return -1.0;
  }

  struct SecondOrderFit
  {
    double natural_frequency_hz = -1.0;
    double damping_ratio = -1.0;
    double delay_sec = -1.0;
    double cost = std::numeric_limits<double>::infinity();
  };

  SecondOrderFit fitSecondOrderModel(
    const std::vector<FrequencyResult> & results,
    double dc_gain) const
  {
    SecondOrderFit best;
    if (results.size() < 3 || dc_gain <= 0.0) {
      return best;
    }

    const double min_freq = std::max(0.01, results.front().frequency_hz);
    const double max_freq = std::max(min_freq * 2.0, results.back().frequency_hz * 10.0);

    for (double wn_hz = min_freq; wn_hz <= max_freq; wn_hz *= 1.03) {
      for (double zeta = 0.1; zeta <= 3.0; zeta += 0.02) {
        double cost = 0.0;
        for (const auto & result : results) {
          if (result.gain <= 0.0 || result.frequency_hz <= 0.0) {
            continue;
          }
          const double r = result.frequency_hz / wn_hz;
          const double denom = std::sqrt(
            std::pow(1.0 - r * r, 2.0) + std::pow(2.0 * zeta * r, 2.0));
          const double model_gain = dc_gain / denom;
          const double err = std::log(result.gain) - std::log(std::max(model_gain, 1e-12));
          cost += err * err;
        }
        if (cost < best.cost) {
          best.cost = cost;
          best.natural_frequency_hz = wn_hz;
          best.damping_ratio = zeta;
        }
      }
    }

    best.delay_sec = estimateDelay(results, best.natural_frequency_hz, best.damping_ratio);
    return best;
  }

  double estimateDelay(
    const std::vector<FrequencyResult> & results,
    double wn_hz,
    double zeta) const
  {
    if (wn_hz <= 0.0 || zeta <= 0.0) {
      return -1.0;
    }
    double weighted_delay_sum = 0.0;
    double weight_sum = 0.0;
    for (const auto & result : results) {
      if (result.frequency_hz <= 0.0 || result.gain <= 0.0) {
        continue;
      }
      const double r = result.frequency_hz / wn_hz;
      const double model_phase = std::atan2(-2.0 * zeta * r, 1.0 - r * r);
      const double residual = wrapPhase(result.phase_lag_rad - model_phase);
      const double omega = 2.0 * kPi * result.frequency_hz;
      const double delay = -residual / omega;
      const double weight = result.gain;
      weighted_delay_sum += weight * delay;
      weight_sum += weight;
    }
    return weight_sum > 0.0 ? weighted_delay_sum / weight_sum : -1.0;
  }

  void writeStepResult(std::ofstream & file, const std::vector<size_t> & indices) const
  {
    if (indices.empty()) {
      return;
    }
    const double initial = responses_[indices.front()];
    const double final = mean(std::vector<double>(
      responses_.begin() + static_cast<long>(indices[indices.size() * 8 / 10]),
      responses_.begin() + static_cast<long>(indices.back() + 1)));
    const double delta = final - initial;
    const double abs_delta = std::abs(delta);
    if (abs_delta < 1e-12) {
      return;
    }

    const double y10 = initial + 0.1 * delta;
    const double y90 = initial + 0.9 * delta;
    double t10 = -1.0;
    double t90 = -1.0;
    double peak = initial;
    for (const size_t index : indices) {
      const double y = responses_[index];
      if ((delta > 0.0 && y > peak) || (delta < 0.0 && y < peak)) {
        peak = y;
      }
      if (t10 < 0.0 && ((delta > 0.0 && y >= y10) || (delta < 0.0 && y <= y10))) {
        t10 = times_[index] - pre_hold_sec_;
      }
      if (t90 < 0.0 && ((delta > 0.0 && y >= y90) || (delta < 0.0 && y <= y90))) {
        t90 = times_[index] - pre_hold_sec_;
      }
    }

    const double overshoot = std::max(0.0, std::abs(peak - final) / abs_delta * 100.0);
    const double command_final = mean(std::vector<double>(
      commands_.begin() + static_cast<long>(indices[indices.size() * 8 / 10]),
      commands_.begin() + static_cast<long>(indices.back() + 1)));

    file << "step_initial_response: " << initial << "\n";
    file << "step_final_response: " << final << "\n";
    file << "step_final_command: " << command_final << "\n";
    file << "step_steady_state_error: " << command_final - final << "\n";
    file << "step_overshoot_percent: " << overshoot << "\n";
    file << "step_rise_time_10_90_sec: " << ((t10 >= 0.0 && t90 >= 0.0) ? t90 - t10 : -1.0) << "\n";
  }

  std::string command_pose_topic_;
  std::string current_pose_topic_;
  std::string output_dir_;
  std::string experiment_name_;
  std::string excitation_type_;
  std::string axis_;
  double frequency_hz_ = 0.5;
  std::vector<double> frequencies_hz_;
  double cycles_per_frequency_ = 8.0;
  double settling_cycles_ = 2.0;
  double pre_hold_sec_ = 2.0;
  double duration_sec_ = 20.0;
  double post_hold_sec_ = 2.0;
  double finish_margin_sec_ = 0.5;
  bool auto_shutdown_ = true;
  int axis_index_ = 2;

  bool command_received_ = false;
  bool response_received_ = false;
  bool started_ = false;
  bool finished_ = false;
  double latest_command_ = 0.0;
  double latest_response_ = 0.0;
  rclcpp::Time start_time_;
  std::vector<double> times_;
  std::vector<double> commands_;
  std::vector<double> responses_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemIdNode>());
  rclcpp::shutdown();
  return 0;
}
