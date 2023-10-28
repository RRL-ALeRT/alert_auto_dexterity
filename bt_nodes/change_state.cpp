#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace BT;

class ChangeState: public RosServiceNode<lifecycle_msgs::srv::ChangeState>
{
public:
  ChangeState(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosServiceNode<lifecycle_msgs::srv::ChangeState>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("label")});
  }

  bool setRequest(Request::SharedPtr& request) override
  {
    auto label = getInput<std::string>("label");
    request->transition.label = label.value();
    return true;
  };

  void halt() override
  {
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  };

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO( node_->get_logger(), "%s: onResponseReceived.", name().c_str() );
    if (response->success)
      return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  };

  virtual BT::NodeStatus onFailure(ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  };
};

CreateRosNodePlugin(ChangeState, "ChangeState");