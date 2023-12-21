// DetectCone.cpp

#include "../include/lab3_pkg/DetectCone.hpp"
#include "std_msgs/msg/int32.hpp" // Changed to Int32
#include "rclcpp/rclcpp.hpp"

namespace lab3_pkg
{
    DetectCone::DetectCone(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), cone_topic_("/bt_detect"), detected_(false)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        setupCallback();

        // Create subscription with the specified topic and callback
        cone_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            cone_topic_, rclcpp::SystemDefaultsQoS(),
            std::bind(&DetectCone::coneCallback, this, std::placeholders::_1), sub_option_);
    }

    BT::NodeStatus DetectCone::tick()
    {
        std::cout << "Checking cone detection status: " << (detected_ ? "Detected" : "Not Detected") << std::endl;

        // Process any pending callbacks
        callback_group_executor_.spin_some();

        return (detected_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void DetectCone::coneCallback(std_msgs::msg::Bool::SharedPtr msg)
    {
        detected_ = msg->data;
        std::cout << "MESSAGE DATA " << (msg->data ? "Detected" : "Not Detected") << std::endl;
    }

    void DetectCone::setupCallback()
    {
        // Create a callback group and add it to the executor
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        // Configure subscription options with the callback group
        sub_option_.callback_group = callback_group_;
    }

} // namespace lab3_pkg

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<lab3_pkg::DetectCone>("DetectCone");
}
