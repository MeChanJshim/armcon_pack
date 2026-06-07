#include <algorithm>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
std::string normalizeMessageType(std::string type_name)
{
  type_name.erase(
    std::remove(type_name.begin(), type_name.end(), ' '),
    type_name.end());

  if (type_name == "std_msgs/Bool" || type_name == "std_msgs::msg::Bool") {
    return "std_msgs/msg/Bool";
  }
  if (type_name == "std_msgs/Empty" || type_name == "std_msgs::msg::Empty") {
    return "std_msgs/msg/Empty";
  }
  if (type_name == "std_msgs/String" || type_name == "std_msgs::msg::String") {
    return "std_msgs/msg/String";
  }
  if (type_name == "std_msgs/Int32" || type_name == "std_msgs::msg::Int32") {
    return "std_msgs/msg/Int32";
  }
  if (type_name == "std_msgs/Float64" || type_name == "std_msgs::msg::Float64") {
    return "std_msgs/msg/Float64";
  }
  if (
    type_name == "std_msgs/Float64MultiArray" ||
    type_name == "std_msgs::msg::Float64MultiArray")
  {
    return "std_msgs/msg/Float64MultiArray";
  }

  return type_name;
}
}  // namespace

class Y2JoystickNode : public rclcpp::Node
{
public:
  Y2JoystickNode()
  : Node("y2_joystick_node")
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "joy");
    axes_topic_name_ = declare_parameter<std::string>("axes.topic_name", "");
    axes_message_type_ = normalizeMessageType(
      declare_parameter<std::string>("axes.message_type", "std_msgs/msg/Float64MultiArray"));
    axes_count_ = declare_parameter<int>("axes.count", 6);
    mapping_names_ = declare_parameter<std::vector<std::string>>(
      "mappings.names",
      std::vector<std::string>{});

    if (!axes_topic_name_.empty()) {
      if (axes_message_type_ == "std_msgs/msg/Float64MultiArray") {
        axes_pub_ = createTypedPublisher<std_msgs::msg::Float64MultiArray>(axes_topic_name_);
        RCLCPP_INFO(
          get_logger(),
          "Configured axes publisher: topic=%s type=%s count=%d",
          axes_topic_name_.c_str(),
          axes_message_type_.c_str(),
          axes_count_);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "axes.message_type '%s' is unsupported. Only std_msgs/msg/Float64MultiArray is supported.",
          axes_message_type_.c_str());
      }
    }

    if (mapping_names_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "No joystick mappings configured. Fill config/y2_joystick.yaml first.");
    }

    for (const auto & mapping_name : mapping_names_) {
      loadMapping(mapping_name);
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      10,
      std::bind(&Y2JoystickNode::joyCB, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Subscribed to joy topic: %s, loaded mappings: %zu",
      joy_topic_.c_str(),
      mappings_.size());
  }

private:
  using BoolPublisher = rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr;
  using EmptyPublisher = rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr;
  using StringPublisher = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;
  using Int32Publisher = rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr;
  using Float64Publisher = rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr;
  using Float64MultiArrayPublisher =
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr;
  using PublisherVariant = std::variant<
    BoolPublisher,
    EmptyPublisher,
    StringPublisher,
    Int32Publisher,
    Float64Publisher,
    Float64MultiArrayPublisher>;

  struct Mapping
  {
    std::string name;
    int button_index = -1;
    std::string topic_name;
    std::string message_type;
    bool bool_value = true;
    int int32_value = 0;
    double float64_value = 0.0;
    std::string string_value;
    PublisherVariant publisher;
  };

  template<typename T>
  typename rclcpp::Publisher<T>::SharedPtr createTypedPublisher(const std::string & topic_name)
  {
    return create_publisher<T>(topic_name, 10);
  }

  void loadMapping(const std::string & mapping_name)
  {
    const std::string prefix = "mappings." + mapping_name;

    declare_parameter<int>(prefix + ".button_index", -1);
    declare_parameter<std::string>(prefix + ".topic_name", "");
    declare_parameter<std::string>(prefix + ".message_type", "std_msgs/msg/Bool");
    declare_parameter<bool>(prefix + ".bool_value", true);
    declare_parameter<int>(prefix + ".int32_value", 0);
    declare_parameter<double>(prefix + ".float64_value", 0.0);
    declare_parameter<std::string>(prefix + ".string_value", "");

    Mapping mapping;
    mapping.name = mapping_name;
    mapping.button_index = get_parameter(prefix + ".button_index").as_int();
    mapping.topic_name = get_parameter(prefix + ".topic_name").as_string();
    mapping.message_type = normalizeMessageType(
      get_parameter(prefix + ".message_type").as_string());
    mapping.bool_value = get_parameter(prefix + ".bool_value").as_bool();
    mapping.int32_value = get_parameter(prefix + ".int32_value").as_int();
    mapping.float64_value = get_parameter(prefix + ".float64_value").as_double();
    mapping.string_value = get_parameter(prefix + ".string_value").as_string();

    if (mapping.button_index < 0) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping mapping '%s': button_index must be >= 0",
        mapping.name.c_str());
      return;
    }

    if (mapping.topic_name.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping mapping '%s': topic_name is empty",
        mapping.name.c_str());
      return;
    }

    if (mapping.message_type == "std_msgs/msg/Bool") {
      mapping.publisher = createTypedPublisher<std_msgs::msg::Bool>(mapping.topic_name);
    } else if (mapping.message_type == "std_msgs/msg/Empty") {
      mapping.publisher = createTypedPublisher<std_msgs::msg::Empty>(mapping.topic_name);
    } else if (mapping.message_type == "std_msgs/msg/String") {
      mapping.publisher = createTypedPublisher<std_msgs::msg::String>(mapping.topic_name);
    } else if (mapping.message_type == "std_msgs/msg/Int32") {
      mapping.publisher = createTypedPublisher<std_msgs::msg::Int32>(mapping.topic_name);
    } else if (mapping.message_type == "std_msgs/msg/Float64") {
      mapping.publisher = createTypedPublisher<std_msgs::msg::Float64>(mapping.topic_name);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Skipping mapping '%s': unsupported message_type '%s'",
        mapping.name.c_str(),
        mapping.message_type.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded mapping '%s': button=%d -> %s (%s)",
      mapping.name.c_str(),
      mapping.button_index,
      mapping.topic_name.c_str(),
      mapping.message_type.c_str());
    mappings_.push_back(std::move(mapping));
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
  {
    publishAxes(*msg);

    if (previous_buttons_.empty()) {
      previous_buttons_ = msg->buttons;
      return;
    }

    for (const auto & mapping : mappings_) {
      if (mapping.button_index >= static_cast<int>(msg->buttons.size()) ||
          mapping.button_index >= static_cast<int>(previous_buttons_.size()))
      {
        if (invalid_index_warned_.insert(mapping.button_index).second) {
          RCLCPP_WARN(
            get_logger(),
            "Mapping '%s' uses button index %d but current joy message has %zu buttons",
            mapping.name.c_str(),
            mapping.button_index,
            msg->buttons.size());
        }
        continue;
      }

      const int prev_state = previous_buttons_[mapping.button_index];
      const int curr_state = msg->buttons[mapping.button_index];
      const bool released = (prev_state != 0) && (curr_state == 0);

      if (released) {
        publishMapping(mapping);
      }
    }

    previous_buttons_ = msg->buttons;
  }

  void publishMapping(const Mapping & mapping)
  {
    if (mapping.message_type == "std_msgs/msg/Bool") {
      std_msgs::msg::Bool msg;
      msg.data = mapping.bool_value;
      std::get<BoolPublisher>(mapping.publisher)->publish(msg);
    } else if (mapping.message_type == "std_msgs/msg/Empty") {
      std_msgs::msg::Empty msg;
      std::get<EmptyPublisher>(mapping.publisher)->publish(msg);
    } else if (mapping.message_type == "std_msgs/msg/String") {
      std_msgs::msg::String msg;
      msg.data = mapping.string_value;
      std::get<StringPublisher>(mapping.publisher)->publish(msg);
    } else if (mapping.message_type == "std_msgs/msg/Int32") {
      std_msgs::msg::Int32 msg;
      msg.data = mapping.int32_value;
      std::get<Int32Publisher>(mapping.publisher)->publish(msg);
    } else if (mapping.message_type == "std_msgs/msg/Float64") {
      std_msgs::msg::Float64 msg;
      msg.data = mapping.float64_value;
      std::get<Float64Publisher>(mapping.publisher)->publish(msg);
    }

    RCLCPP_INFO(
      get_logger(),
      "Published mapping '%s' on release: topic=%s type=%s",
      mapping.name.c_str(),
      mapping.topic_name.c_str(),
      mapping.message_type.c_str());
  }

  void publishAxes(const sensor_msgs::msg::Joy & joy_msg)
  {
    if (!axes_pub_) {
      return;
    }

    std_msgs::msg::Float64MultiArray msg;
    const std::size_t copy_count = std::min<std::size_t>(
      static_cast<std::size_t>(std::max(axes_count_, 0)),
      joy_msg.axes.size());
    msg.data.reserve(copy_count);
    for (std::size_t i = 0; i < copy_count; ++i) {
      msg.data.push_back(static_cast<double>(joy_msg.axes[i]));
    }
    axes_pub_->publish(msg);
  }

  std::string joy_topic_;
  std::string axes_topic_name_;
  std::string axes_message_type_;
  int axes_count_ = 6;
  std::vector<std::string> mapping_names_;
  std::vector<int32_t> previous_buttons_;
  std::vector<Mapping> mappings_;
  std::set<int> invalid_index_warned_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  Float64MultiArrayPublisher axes_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Y2JoystickNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
