#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace BT;

class Trigger: public RosServiceNode<std_srvs::srv::Trigger>
{
public:
  Trigger(const std::string& name,
          const NodeConfig& conf,
          const RosNodeParams& params)
    : RosServiceNode<std_srvs::srv::Trigger>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool setRequest(Request::SharedPtr& request) override
  {
    return true;
  };

  void halt() override
  {
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  };

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO( node_->get_logger(), "%s: onResponseReceived. Message: %s", name().c_str() , response.get()->message);

    if (response.get()->success)
      return NodeStatus::SUCCESS;
      
    return NodeStatus::FAILURE;
  };

  virtual BT::NodeStatus onFailure(ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  };
};

CreateRosNodePlugin(Trigger, "Trigger");