#include "see_object_set.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool SeeObjectSet::setGoal(RosActionNode::Goal &goal)
{
  auto location = getInput<std::string>("location");
  goal.location = location.value();
  return true;
}

NodeStatus SeeObjectSet::onResultReceived(const RosActionNode::WrappedResult &wr)
{
  RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived.", name().c_str() );

  return NodeStatus::SUCCESS;
}

NodeStatus SeeObjectSet::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void SeeObjectSet::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
}

// Plugin registration.
// The class SeeObjectSet will self register with name  "SeeObjectSet".
CreateRosNodePlugin(SeeObjectSet, "SeeObjectSet");