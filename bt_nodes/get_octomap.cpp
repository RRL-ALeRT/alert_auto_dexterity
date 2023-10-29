#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"

using namespace BT;

class GetOctomap: public RosServiceNode<octomap_msgs::srv::GetOctomap>
{
public:
  GetOctomap(const std::string& name,
             const NodeConfig& conf,
             const RosNodeParams& params)
    : RosServiceNode<octomap_msgs::srv::GetOctomap>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({{OutputPort<std::shared_ptr<octomap_msgs::msg::Octomap>>("octomap")}});
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
    RCLCPP_INFO( node_->get_logger(), "%s: onResponseReceived.", name().c_str() );

    auto octomap_ptr = std::make_shared<octomap_msgs::msg::Octomap>>(response->map);
    setOutput<std::shared_ptr<octomap_msgs::msg::Octomap>>("octomap", octomap_ptr);

    return NodeStatus::SUCCESS;
  };

  virtual BT::NodeStatus onFailure(ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  };
};

CreateRosNodePlugin(GetOctomap, "GetOctomap");