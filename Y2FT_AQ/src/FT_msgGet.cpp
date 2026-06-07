#include "Y2FT_AQ/FT_msgGet.hpp"

#include <algorithm>
#include <utility>

namespace
{
std::string normalizeMessageType(std::string type_name)
{
    type_name.erase(
        std::remove(type_name.begin(), type_name.end(), ' '),
        type_name.end());

    if (type_name == "geometry_msgs/WrenchStamped" ||
        type_name == "geometry_msgs::msg::WrenchStamped")
    {
        return "geometry_msgs/msg/WrenchStamped";
    }

    if (type_name == "geometry_msgs/Wrench" ||
        type_name == "geometry_msgs::msg::Wrench")
    {
        return "geometry_msgs/msg/Wrench";
    }

    if (type_name == "std_msgs/Float64MultiArray" ||
        type_name == "std_msgs::msg::Float64MultiArray")
    {
        return "std_msgs/msg/Float64MultiArray";
    }

    return type_name;
}
}

FT_msgGet::FT_msgGet(rclcpp::Node& node,
                     const std::string& topic_name,
                     const std::string& message_type,
                     const FTMsgArrayFormat& array_format)
    : node_(node)
    , topic_name_(topic_name)
    , message_type_(normalizeMessageType(message_type))
    , array_format_(array_format)
    , init_force_(3, 0.0)
    , init_moment_(3, 0.0)
{
    createSubscription();
}

FTData FT_msgGet::FTGet()
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    FTData ftdata = latest_ftdata_;
    if (init_flag_) {
        ftdata.Fx -= init_force_[0];
        ftdata.Fy -= init_force_[1];
        ftdata.Fz -= init_force_[2];
        ftdata.Mx -= init_moment_[0];
        ftdata.My -= init_moment_[1];
        ftdata.Mz -= init_moment_[2];
    }

    return ftdata;
}

bool FT_msgGet::FT_init(const unsigned int init_count_num)
{
    if (init_count_num == 0) {
        RCLCPP_ERROR(node_.get_logger(),
                     "[FT_msgGet] init_count_num must be greater than zero.");
        return false;
    }

    if (!hasReceivedData()) {
        return false;
    }

    if (init_flag_) {
        return true;
    }

    if (init_count_ == 0) {
        std::fill(init_force_.begin(), init_force_.end(), 0.0);
        std::fill(init_moment_.begin(), init_moment_.end(), 0.0);
    }

    if (init_count_ < init_count_num) {
        const FTData ftdata = FTGet();

        init_force_[0] += ftdata.Fx / static_cast<double>(init_count_num);
        init_force_[1] += ftdata.Fy / static_cast<double>(init_count_num);
        init_force_[2] += ftdata.Fz / static_cast<double>(init_count_num);

        init_moment_[0] += ftdata.Mx / static_cast<double>(init_count_num);
        init_moment_[1] += ftdata.My / static_cast<double>(init_count_num);
        init_moment_[2] += ftdata.Mz / static_cast<double>(init_count_num);

        ++init_count_;
        return false;
    }

    init_flag_ = true;
    return true;
}

bool FT_msgGet::hasReceivedData() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return has_received_data_;
}

const std::string& FT_msgGet::getTopicName() const
{
    return topic_name_;
}

const std::string& FT_msgGet::getMessageType() const
{
    return message_type_;
}

void FT_msgGet::createSubscription()
{
    constexpr std::size_t qos_depth = 10;

    if (message_type_ == "geometry_msgs/msg/WrenchStamped") {
        wrench_stamped_sub_ =
            node_.create_subscription<geometry_msgs::msg::WrenchStamped>(
                topic_name_,
                qos_depth,
                std::bind(&FT_msgGet::wrenchStampedCB, this, std::placeholders::_1));
        return;
    }

    if (message_type_ == "geometry_msgs/msg/Wrench") {
        wrench_sub_ =
            node_.create_subscription<geometry_msgs::msg::Wrench>(
                topic_name_,
                qos_depth,
                std::bind(&FT_msgGet::wrenchCB, this, std::placeholders::_1));
        return;
    }

    if (message_type_ == "std_msgs/msg/Float64MultiArray") {
        float64_multi_array_sub_ =
            node_.create_subscription<std_msgs::msg::Float64MultiArray>(
                topic_name_,
                qos_depth,
                std::bind(&FT_msgGet::float64MultiArrayCB, this, std::placeholders::_1));
        return;
    }

    RCLCPP_ERROR(node_.get_logger(),
                 "[FT_msgGet] Unsupported message type: %s",
                 message_type_.c_str());
}

void FT_msgGet::wrenchStampedCB(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_ftdata_ = FTData(
        msg->wrench.force.x,
        msg->wrench.force.y,
        msg->wrench.force.z,
        msg->wrench.torque.x,
        msg->wrench.torque.y,
        msg->wrench.torque.z);
    has_received_data_ = true;
}

void FT_msgGet::wrenchCB(const geometry_msgs::msg::Wrench::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_ftdata_ = FTData(
        msg->force.x,
        msg->force.y,
        msg->force.z,
        msg->torque.x,
        msg->torque.y,
        msg->torque.z);
    has_received_data_ = true;
}

void FT_msgGet::float64MultiArrayCB(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
    const FTData ftdata = extractFromArray(msg->data);

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_ftdata_ = ftdata;
    has_received_data_ = true;
}

FTData FT_msgGet::extractFromArray(const std::vector<double>& data) const
{
    const auto read_value = [&data](std::size_t index) -> double {
        return index < data.size() ? data[index] : 0.0;
    };

    return FTData(
        read_value(array_format_.fx),
        read_value(array_format_.fy),
        read_value(array_format_.fz),
        read_value(array_format_.mx),
        read_value(array_format_.my),
        read_value(array_format_.mz));
}
