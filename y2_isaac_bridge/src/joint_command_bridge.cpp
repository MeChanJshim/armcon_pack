#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

class JointAndFtBridge : public rclcpp::Node
{
public:
  JointAndFtBridge()
  : Node("joint_and_ft_bridge")
  {
    // ----------------------------
    // Parameters
    // ----------------------------
    cmd_input_topic_ = this->declare_parameter<std::string>(
      "cmd_input_topic", "/forward_position_controller/commands");
    cmd_output_topic_ = this->declare_parameter<std::string>(
      "cmd_output_topic", "/isaac_joint_commands");

    ft_input_topic_ = this->declare_parameter<std::string>(
      "ft_input_topic", "/isaac/ftdata");
    ft_output_topic_ = this->declare_parameter<std::string>(
      "ft_output_topic", "/ur10skku/ftdata");

    current_p_topic_ = this->declare_parameter<std::string>(
      "current_p_topic", "/ur10skku/currentP");

    ft_frame_id_ = this->declare_parameter<std::string>(
      "ft_frame_id", "base");

    // FT sign / scale
    fx_sign_ = this->declare_parameter<double>("fx_sign", 1.0);
    fy_sign_ = this->declare_parameter<double>("fy_sign", 1.0);
    fz_sign_ = this->declare_parameter<double>("fz_sign", -1.0);
    mx_sign_ = this->declare_parameter<double>("mx_sign", 1.0);
    my_sign_ = this->declare_parameter<double>("my_sign", 1.0);
    mz_sign_ = this->declare_parameter<double>("mz_sign", 1.0);

    force_scale_ = this->declare_parameter<double>("force_scale", 1.0);
    torque_scale_ = this->declare_parameter<double>("torque_scale", 1.0);

    // FTGetMain style MOV filter
    mov_size_ = this->declare_parameter<int>("mov_size", 2);

    // use_bias = gravity compensation + static Fz offset compensation
    use_bias_ = this->declare_parameter<bool>("use_bias", true);
    bias_samples_ = this->declare_parameter<int>("bias_samples", 200);

    // FTGetMain gravity compensation params
    tool_mass_ = this->declare_parameter<double>("tool_mass", 1.6);
    tool_cog_x_ = this->declare_parameter<double>("tool_cog_x", 0.0);
    tool_cog_y_ = this->declare_parameter<double>("tool_cog_y", 0.0);
    tool_cog_z_ = this->declare_parameter<double>("tool_cog_z", -0.149303);

    // Deadband
    force_deadband_ = this->declare_parameter<double>("force_deadband", 0.05);
    torque_deadband_ = this->declare_parameter<double>("torque_deadband", 0.005);

    // Fz EMA filter
    use_fz_ema_ = this->declare_parameter<bool>("use_fz_ema", true);
    fz_ema_alpha_ = this->declare_parameter<double>("fz_ema_alpha", 0.065);

    joint_names_ = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };

    current_tcp_pose_.fill(0.0);
    setIdentity(ROT_Base2TCP_);

    // FTGetMain의 ROT_TCP2FT와 동일
    ROT_TCP2FT_ = {{
      {{ 1.0,  0.0,  0.0 }},
      {{ 0.0, -1.0,  0.0 }},
      {{ 0.0,  0.0, -1.0 }}
    }};

    mov_sum_.fill(0.0);

    g_ft_init_.fill(0.0);
    g_ft_init_set_ = false;

    static_fz_offset_acc_ = 0.0;
    static_fz_offset_ = 0.0;
    static_fz_offset_count_ = 0;
    static_fz_offset_set_ = false;

    fz_ema_state_ = 0.0;
    fz_ema_initialized_ = false;

    // ----------------------------
    // Publishers
    // ----------------------------
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      cmd_output_topic_, rclcpp::QoS(10));

    ft_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      ft_output_topic_, rclcpp::QoS(50));

    // ----------------------------
    // Subscribers
    // ----------------------------
    joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      cmd_input_topic_,
      rclcpp::QoS(10),
      std::bind(&JointAndFtBridge::jointCommandCallback, this, std::placeholders::_1));

    ft_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      ft_input_topic_,
      rclcpp::QoS(50),
      std::bind(&JointAndFtBridge::ftCallback, this, std::placeholders::_1));

    current_p_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      current_p_topic_,
      rclcpp::QoS(50),
      std::bind(&JointAndFtBridge::currentPCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "==============================================");
    RCLCPP_INFO(this->get_logger(), "JointAndFtBridge started");
    RCLCPP_INFO(this->get_logger(), "Joint cmd in : %s [Float64MultiArray]", cmd_input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Joint cmd out: %s [JointState]", cmd_output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "FT in        : %s [WrenchStamped]", ft_input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "FT out       : %s [WrenchStamped]", ft_output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "CurrentP in  : %s [Float64MultiArray]", current_p_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "FT frame_id  : %s", ft_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "MOV size     : %d", mov_size_);
    RCLCPP_INFO(this->get_logger(), "use_bias (gravity comp + static Fz offset) = %s",
      use_bias_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "bias_samples = %d", bias_samples_);
    RCLCPP_INFO(this->get_logger(), "tool_mass = %.6f [kg]", tool_mass_);
    RCLCPP_INFO(this->get_logger(), "tool_cog  = [%.6f, %.6f, %.6f] [m]",
      tool_cog_x_, tool_cog_y_, tool_cog_z_);
    RCLCPP_INFO(this->get_logger(), "use_fz_ema  = %s", use_fz_ema_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "fz_ema_alpha= %.6f", fz_ema_alpha_);
    RCLCPP_INFO(this->get_logger(), "==============================================");
  }

private:
  using Vec3 = std::array<double, 3>;
  using Vec6 = std::array<double, 6>;
  using Mat3 = std::array<std::array<double, 3>, 3>;

  // ============================================================
  // Joint bridge
  // ============================================================
  void jointCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received joint command size = %zu, expected >= 6. Ignored.",
        msg->data.size());
      return;
    }

    sensor_msgs::msg::JointState out_msg;
    out_msg.header.stamp = this->now();
    out_msg.name = joint_names_;
    out_msg.position.assign(msg->data.begin(), msg->data.begin() + 6);

    joint_pub_->publish(out_msg);
  }

  // ============================================================
  // /ur10skku/currentP callback
  // currentP = [x, y, z, wx, wy, wz]
  // ============================================================
  void currentPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) {
      return;
    }

    for (size_t i = 0; i < 6; ++i) {
      current_tcp_pose_[i] = msg->data[i];
    }

    const double wx = current_tcp_pose_[3];
    const double wy = current_tcp_pose_[4];
    const double wz = current_tcp_pose_[5];

    ROT_Base2TCP_ = spatialVectorToRotation(wx, wy, wz);
  }

  // ============================================================
  // FT bridge
  // 1) 6축 MOV filter
  // 2) Sensor -> TCP
  // 3) FTGetMain style gravity compensation in TCP
  // 4) TCP -> Base
  // 5) static Fz offset compensation in Base
  // 5.5) Fz EMA in Base
  // 6) deadband
  // 7) publish
  // ============================================================
  void ftCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    Vec6 raw{};
    raw[0] = fx_sign_ * force_scale_  * msg->wrench.force.x;
    raw[1] = fy_sign_ * force_scale_  * msg->wrench.force.y;
    raw[2] = fz_sign_ * force_scale_  * msg->wrench.force.z;
    raw[3] = mx_sign_ * torque_scale_ * msg->wrench.torque.x;
    raw[4] = my_sign_ * torque_scale_ * msg->wrench.torque.y;
    raw[5] = mz_sign_ * torque_scale_ * msg->wrench.torque.z;

    // 1) MOV filter
    Vec6 filtered_sensor{};
    for (size_t i = 0; i < 6; ++i) {
      filtered_sensor[i] = movingAverageFilter(i, raw[i]);
    }

    // 2) Sensor frame -> TCP frame
    Vec3 sframe_force  = {filtered_sensor[0], filtered_sensor[1], filtered_sensor[2]};
    Vec3 sframe_moment = {filtered_sensor[3], filtered_sensor[4], filtered_sensor[5]};

    Vec3 tcp_force  = matVecMul(ROT_TCP2FT_, sframe_force);
    Vec3 tcp_moment = matVecMul(ROT_TCP2FT_, sframe_moment);

    // 3) FTGetMain style gravity compensation in TCP frame
    if (use_bias_ && tool_mass_ > 0.0) {
      Vec3 g_base = {0.0, 0.0, -9.81};

      // g_tcp = ROT_Base2TCP^T * g_base
      Vec3 g_tcp = matTransposeVecMul(ROT_Base2TCP_, g_base);

      // 최초 한 번 g_init 저장
      if (!g_ft_init_set_) {
        g_ft_init_ = g_tcp;
        g_ft_init_set_ = true;
      }

      // Δg = g(k) - g(0),  Fg = m * Δg
      Vec3 s_gravity_force{};
      for (int i = 0; i < 3; ++i) {
        const double dg = g_tcp[i] - g_ft_init_[i];
        s_gravity_force[i] = dg * tool_mass_;
      }

      // τg = r × Fg
      Vec3 tool_cog = {tool_cog_x_, tool_cog_y_, tool_cog_z_};
      Vec3 s_gravity_moment = cross3(tool_cog, s_gravity_force);

      // TCP frame에서 중력 보상
      for (int i = 0; i < 3; ++i) {
        tcp_force[i]  -= s_gravity_force[i];
        tcp_moment[i] -= s_gravity_moment[i];
      }
    }

    // 4) TCP -> Base
    Vec3 base_force  = matVecMul(ROT_Base2TCP_, tcp_force);
    Vec3 base_moment = matVecMul(ROT_Base2TCP_, tcp_moment);

    Vec6 base_wrench{};
    base_wrench[0] = base_force[0];
    base_wrench[1] = base_force[1];
    base_wrench[2] = base_force[2];
    base_wrench[3] = base_moment[0];
    base_wrench[4] = base_moment[1];
    base_wrench[5] = base_moment[2];

    // 5) static Fz offset compensation in Base frame
    //    초기 Base Fz 평균을 저장한 뒤 이후 계속 빼줌
    if (use_bias_ && !static_fz_offset_set_) {
      static_fz_offset_acc_ += base_wrench[2];
      static_fz_offset_count_++;

      if (static_fz_offset_count_ >= bias_samples_) {
        static_fz_offset_ =
          static_fz_offset_acc_ / static_cast<double>(static_fz_offset_count_);
        static_fz_offset_set_ = true;

        // EMA state도 offset 제거된 값 기준으로 초기화
        fz_ema_state_ = 0.0;
        fz_ema_initialized_ = false;

        RCLCPP_INFO(this->get_logger(),
          "Static Fz offset estimation complete: Fz_offset=%.6f [N]",
          static_fz_offset_);
      }

      // offset 추정 중에는 Fz를 0으로 보냄
      base_wrench[2] = 0.0;
    } else if (use_bias_ && static_fz_offset_set_) {
      base_wrench[2] -= static_fz_offset_;
    }

    // 5.5) Fz EMA in Base frame
    // static offset compensation 이후의 Fz에 대해 EMA 적용
    if (use_fz_ema_) {
      base_wrench[2] = emaFilterFz(base_wrench[2]);
    }

    // 6) deadband
    for (size_t i = 0; i < 3; ++i) {
      if (std::abs(base_wrench[i]) < force_deadband_) {
        base_wrench[i] = 0.0;
      }
    }
    for (size_t i = 3; i < 6; ++i) {
      if (std::abs(base_wrench[i]) < torque_deadband_) {
        base_wrench[i] = 0.0;
      }
    }

    // 7) publish
    publishWrench(msg->header, base_wrench);
  }

  // ============================================================
  // MOV filter
  // ============================================================
  double movingAverageFilter(size_t axis, double input)
  {
    auto &q = mov_buffer_[axis];
    mov_sum_[axis] += input;
    q.push_back(input);

    if (static_cast<int>(q.size()) > mov_size_) {
      mov_sum_[axis] -= q.front();
      q.pop_front();
    }

    if (q.empty()) {
      return input;
    }

    return mov_sum_[axis] / static_cast<double>(q.size());
  }

  // ============================================================
  // Fz EMA filter
  // y[k] = alpha * x[k] + (1-alpha) * y[k-1]
  // ============================================================
  double emaFilterFz(double input)
  {
    const double alpha = clamp(fz_ema_alpha_, 0.0, 1.0);

    if (!fz_ema_initialized_) {
      fz_ema_state_ = input;
      fz_ema_initialized_ = true;
      return fz_ema_state_;
    }

    fz_ema_state_ = alpha * input + (1.0 - alpha) * fz_ema_state_;
    return fz_ema_state_;
  }

  double clamp(double x, double lo, double hi)
  {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }

  // ============================================================
  // Rodrigues: spatial vector(wx, wy, wz) -> rotation matrix
  // ============================================================
  Mat3 spatialVectorToRotation(double wx, double wy, double wz)
  {
    Mat3 R{};
    setIdentity(R);

    const double theta = std::sqrt(wx * wx + wy * wy + wz * wz);
    if (theta < 1e-12) {
      return R;
    }

    const double kx = wx / theta;
    const double ky = wy / theta;
    const double kz = wz / theta;

    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double v = 1.0 - c;

    R[0][0] = kx*kx*v + c;
    R[0][1] = kx*ky*v - kz*s;
    R[0][2] = kx*kz*v + ky*s;

    R[1][0] = ky*kx*v + kz*s;
    R[1][1] = ky*ky*v + c;
    R[1][2] = ky*kz*v - kx*s;

    R[2][0] = kz*kx*v - ky*s;
    R[2][1] = kz*ky*v + kx*s;
    R[2][2] = kz*kz*v + c;

    return R;
  }

  Vec3 matVecMul(const Mat3 &R, const Vec3 &v)
  {
    Vec3 out{};
    for (int i = 0; i < 3; ++i) {
      out[i] = 0.0;
      for (int j = 0; j < 3; ++j) {
        out[i] += R[i][j] * v[j];
      }
    }
    return out;
  }

  Vec3 matTransposeVecMul(const Mat3 &R, const Vec3 &v)
  {
    Vec3 out{};
    for (int i = 0; i < 3; ++i) {
      out[i] = 0.0;
      for (int j = 0; j < 3; ++j) {
        out[i] += R[j][i] * v[j];
      }
    }
    return out;
  }

  Vec3 cross3(const Vec3 &a, const Vec3 &b)
  {
    Vec3 out{};
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
    return out;
  }

  void setIdentity(Mat3 &R)
  {
    R = {{
      {{1.0, 0.0, 0.0}},
      {{0.0, 1.0, 0.0}},
      {{0.0, 0.0, 1.0}}
    }};
  }

  void publishWrench(const std_msgs::msg::Header &header_in, const Vec6 &w)
  {
    geometry_msgs::msg::WrenchStamped out;
    out.header = header_in;
    out.header.stamp = this->now();
    out.header.frame_id = ft_frame_id_;

    out.wrench.force.x  = w[0];
    out.wrench.force.y  = w[1];
    out.wrench.force.z  = w[2];
    out.wrench.torque.x = w[3];
    out.wrench.torque.y = w[4];
    out.wrench.torque.z = w[5];

    ft_pub_->publish(out);
  }

private:
  // Topics
  std::string cmd_input_topic_;
  std::string cmd_output_topic_;
  std::string ft_input_topic_;
  std::string ft_output_topic_;
  std::string current_p_topic_;
  std::string ft_frame_id_;

  // FT params
  double fx_sign_;
  double fy_sign_;
  double fz_sign_;
  double mx_sign_;
  double my_sign_;
  double mz_sign_;
  double force_scale_;
  double torque_scale_;
  int mov_size_;
  bool use_bias_;
  int bias_samples_;
  double tool_mass_;
  double tool_cog_x_;
  double tool_cog_y_;
  double tool_cog_z_;
  double force_deadband_;
  double torque_deadband_;

  // Fz EMA params/state
  bool use_fz_ema_;
  double fz_ema_alpha_;
  double fz_ema_state_;
  bool fz_ema_initialized_;

  // State
  std::vector<std::string> joint_names_;
  std::array<double, 6> current_tcp_pose_;
  Mat3 ROT_Base2TCP_;
  Mat3 ROT_TCP2FT_;

  std::array<std::deque<double>, 6> mov_buffer_;
  std::array<double, 6> mov_sum_;

  // gravity compensation state
  Vec3 g_ft_init_;
  bool g_ft_init_set_;

  // static Fz offset state
  double static_fz_offset_acc_;
  double static_fz_offset_;
  int static_fz_offset_count_;
  bool static_fz_offset_set_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_p_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointAndFtBridge>());
  rclcpp::shutdown();
  return 0;
}