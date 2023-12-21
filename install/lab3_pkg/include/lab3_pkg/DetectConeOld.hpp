#ifndef DETECT_CONE_NODE_HPP_
#define DETECT_CONE_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>

class DetectCone : public BT::SyncActionNode {
public:
    DetectCone(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

#endif  // DETECT_CONE_NODE_HPP_
