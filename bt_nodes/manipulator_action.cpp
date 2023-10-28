#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "alert_auto_dexterity/action/manipulator_action.hpp"

using namespace BT;

class ManipulatorAction: public RosActionNode<alert_auto_dexterity::action::ManipulatorAction>
{
public:
  ManipulatorAction(const std::string& name,
                          const NodeConfig& conf,
                          const RosNodeParams& params)
    : RosActionNode<alert_auto_dexterity::action::ManipulatorAction>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("location")});
  }

  bool setGoal(Goal& goal) override
  {
    auto location = getInput<std::string>("location");
    goal.location = location.value();
    return true;
  };

  void onHalt() override
  {
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  };

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived.", name().c_str() );

    return NodeStatus::SUCCESS;
  };

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  };
};

CreateRosNodePlugin(ManipulatorAction, "ManipulatorAction");