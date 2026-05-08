#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include "ackermann_cmd_mux/ackermann_cmd_subscribers.hpp"
#include "ackermann_cmd_mux/exceptions.hpp"

namespace ackermann_cmd_mux {

class AckermannCmdMuxNode : public rclcpp::Node
{
public:
    // Required by pluginlib
    AckermannCmdMuxNode();
    explicit AckermannCmdMuxNode(const rclcpp::NodeOptions & options);

private:

    using AckermannMsgPtr =
        ackermann_msgs::msg::AckermannDriveStamped::SharedPtr;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr output_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;

    AckermannCmdSubscribers subs_cfg_;

    std::vector<rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr> subscriptions_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;

    std::string yaml_config_file_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    int allowed_ = -1;

    void load_configuration();

    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> & params);

    void ackermann_cmd_callback(AckermannMsgPtr msg, size_t idx);
    void timeout_callback(size_t idx);

    void publish_active(const std::string & name);
};

} // namespace ackermann_cmd_mux
