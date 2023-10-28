#include "behaviortree_ros2/bt_action_node.hpp"
#include "alert_auto_dexterity/action/manipulator_manipulation.hpp"

using namespace BT;

class SeeObjectSet: public RosActionNode<alert_auto_dexterity::action::ManipulatorManipulation>
{
public:
  SeeObjectSet(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<alert_auto_dexterity::action::ManipulatorManipulation>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("location")});
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};