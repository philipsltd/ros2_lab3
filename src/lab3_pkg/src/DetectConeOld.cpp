#include "/home/trsa2024/lab3_ws/src/lab3_pkg/include/lab3_pkg/DetectCone.hpp"
#include "stdio.h"
#include "rclcpp/rclcpp.hpp"

DetectCone::DetectCone(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList DetectCone::providedPorts() {
    return {}; // Define any input/output ports if needed
}

BT::NodeStatus DetectCone::tick() {
    // Your cone detection logic goes here
    // For example, simulating detection by returning SUCCESS after a certain time
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Simulate some work

    // Replace this with your actual cone detection logic
    bool coneDetected = /* Your cone detection logic here */ true;

    if (coneDetected) {
        printf("\t\tIN SUCCESS!!!\n");
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

#include <behaviortree_cpp_v3/bt_factory.h>

// BT_REGISTER_NODES(factory) {
//     BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
//         return std::make_unique<DetectConeAction>(name, config);
//     };

//     factory.registerBuilder<DetectConeAction>("DetectCone", builder);
// }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("DetectCone");

    // Create your Behavior Tree and factory
    BT::BehaviorTreeFactory factory;
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<DetectCone>(name, config);
    };

    // Register your action node in the Behavior Tree factory
    factory.registerBuilder<DetectCone>("DetectCone", builder);

    // Create your Behavior Tree instance
    BT::Tree tree = factory.createTreeFromFile("/path/to/your/behavior_tree.xml"); // Replace with actual path

    // Execute your Behavior Tree
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.rootNode()->executeTick(); // Execute the root node of the tree
        rclcpp::spin_some(node); // Process any pending ROS callbacks
    }

    rclcpp::shutdown();
    return 0;
}