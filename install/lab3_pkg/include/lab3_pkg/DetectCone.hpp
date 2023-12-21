// DetectCone.hpp

#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lab3_pkg
{
class DetectCone : public BT::ConditionNode
{
public:
    DetectCone(const std::string &name, const BT::NodeConfiguration &config);

    DetectCone() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    void coneCallback(std_msgs::msg::Bool::SharedPtr msg);

    void setupCallback();

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::SubscriptionOptions sub_option_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cone_sub_;
    std::string cone_topic_;
    bool detected_;
};
} // namespace lab3_pkg
