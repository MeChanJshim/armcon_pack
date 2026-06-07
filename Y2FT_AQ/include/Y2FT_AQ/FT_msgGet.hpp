#pragma once

#include <cstddef>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "Y2FT_AQ/FT_EtherGet.hpp"

struct FTMsgArrayFormat
{
    std::size_t fx = 0;
    std::size_t fy = 1;
    std::size_t fz = 2;
    std::size_t mx = 3;
    std::size_t my = 4;
    std::size_t mz = 5;
};

class FT_msgGet
{
public:
    FT_msgGet(rclcpp::Node& node,
              const std::string& topic_name,
              const std::string& message_type = "geometry_msgs/msg/WrenchStamped",
              const FTMsgArrayFormat& array_format = FTMsgArrayFormat());
    ~FT_msgGet() = default;

    FTData FTGet();
    bool FT_init(const unsigned int init_count_num);

    bool hasReceivedData() const;
    const std::string& getTopicName() const;
    const std::string& getMessageType() const;

private:
    void createSubscription();
    void wrenchStampedCB(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg);
    void wrenchCB(const geometry_msgs::msg::Wrench::ConstSharedPtr msg);
    void float64MultiArrayCB(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
    FTData extractFromArray(const std::vector<double>& data) const;

    rclcpp::Node& node_;
    std::string topic_name_;
    std::string message_type_;
    FTMsgArrayFormat array_format_;

    std::vector<double> init_force_;
    std::vector<double> init_moment_;
    bool init_flag_ = false;
    unsigned int init_count_ = 0;

    mutable std::mutex data_mutex_;
    FTData latest_ftdata_;
    bool has_received_data_ = false;

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_stamped_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr float64_multi_array_sub_;
};
