#include <fstream>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <rclcpp_components/register_node_macro.hpp>

#include "ackermann_cmd_mux/ackermann_cmd_mux_node.hpp"

using namespace std::chrono_literals;

namespace ackermann_cmd_mux
{

    AckermannCmdMuxNode::AckermannCmdMuxNode()
    : AckermannCmdMuxNode(rclcpp::NodeOptions{})
    {}
    

AckermannCmdMuxNode::AckermannCmdMuxNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ackermann_cmd_mux", options)
{
    // Declare runtime-configurable parameter
    this->declare_parameter<std::string>("yaml_config_file", "");

    // Enable live parameter updates
    param_callback_handle_ =
        this->add_on_set_parameters_callback(
            std::bind(&AckermannCmdMuxNode::on_parameter_change, this, std::placeholders::_1)
        );

    // Publisher that announces which channel is active
    active_pub_ = this->create_publisher<std_msgs::msg::String>("active", 1);

    publish_active("idle");

    // Read initial config file path and load configuration
    yaml_config_file_ = this->get_parameter("yaml_config_file").as_string();
    load_configuration();

    RCLCPP_INFO(this->get_logger(), "AckermannCmdMuxNode initialized.");
}

rcl_interfaces::msg::SetParametersResult
AckermannCmdMuxNode::on_parameter_change(const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params)
    {
        if (param.get_name() == "yaml_config_file")
        {
            yaml_config_file_ = param.as_string();
            load_configuration();
        }
    }

    return result;
}

void AckermannCmdMuxNode::load_configuration()
{
    if (yaml_config_file_.empty()) {
        RCLCPP_WARN(this->get_logger(), "yaml_config_file parameter is empty.");
        return;
    }

    std::ifstream ifs(yaml_config_file_);
    if (!ifs.good()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open YAML file: '%s'", yaml_config_file_.c_str());
        return;
    }

    YAML::Node doc = YAML::Load(ifs);

    // Output publisher name (ROS1 legacy behavior)
    std::string output_topic = "output";
    if (doc["publisher"]) {
        output_topic = doc["publisher"].as<std::string>();
    }

    output_pub_ =
        this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            output_topic, rclcpp::QoS(10)
        );

    // Parse subscriber channels from YAML
    try {
        subs_cfg_.configure(doc["subscribers"]);
    }
    catch (const YamlException & e) {
        RCLCPP_ERROR(this->get_logger(), "YAML error: %s", e.what());
        return;
    }

    size_t N = subs_cfg_.list.size();
    subscriptions_.resize(N);
    timers_.resize(N);

    allowed_ = -1;

    // Create all subscribers and timers
    for (size_t i = 0; i < N; ++i)
    {
        auto & cfg = subs_cfg_.list[i];

        // Ackermann command subscribers
        subscriptions_[i] =
            this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
                cfg.topic,
                rclcpp::QoS(10),
                [this, i](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
                {
                    this->ackermann_cmd_callback(msg, i);
                }
            );

        // Timeout timer for each input source
        timers_[i] = this->create_wall_timer(
            std::chrono::duration<double>(cfg.timeout),
            [this, i]() {
                this->timeout_callback(i);
            }
        );

        // Start disabled; only reset when messages arrive
        timers_[i]->cancel();

        RCLCPP_INFO(this->get_logger(),
                    "Subscribed to '%s' (name=%s, prio=%d, timeout=%.2f)",
                    cfg.topic.c_str(), cfg.name.c_str(),
                    cfg.priority, cfg.timeout);
    }

    RCLCPP_INFO(this->get_logger(),
                "AckermannCmdMux: Loaded configuration from '%s'",
                yaml_config_file_.c_str());
}

void AckermannCmdMuxNode::ackermann_cmd_callback(AckermannMsgPtr msg, size_t idx)
{
    auto & cfg = subs_cfg_.list[idx];

    // Reset timer because this channel is alive
    timers_[idx]->reset();

    // Arbitration logic
    bool take_control = false;

    if (allowed_ == -1)
        take_control = true;
    else if (allowed_ == static_cast<int>(idx))
        take_control = true;
    else {
        int current = allowed_;
        if (cfg.priority > subs_cfg_.list[current].priority)
            take_control = true;
    }

    if (take_control)
    {
        if (allowed_ != static_cast<int>(idx)) {
            allowed_ = idx;
            publish_active(cfg.name);
        }

        output_pub_->publish(*msg);
    }
}

void AckermannCmdMuxNode::timeout_callback(size_t idx)
{
    if (allowed_ == static_cast<int>(idx))
    {
        allowed_ = -1;
        publish_active("idle");
    }
}

void AckermannCmdMuxNode::publish_active(const std::string & name)
{
    std_msgs::msg::String msg;
    msg.data = name;
    active_pub_->publish(msg);
}

} // namespace ackermann_cmd_mux
RCLCPP_COMPONENTS_REGISTER_NODE(ackermann_cmd_mux::AckermannCmdMuxNode)

