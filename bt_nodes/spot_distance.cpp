#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "spot_msgs/action/trajectory.hpp"

using namespace BT;

class SpotDistance : public RosActionNode<spot_msgs::action::Trajectory> {
public:
  SpotDistance(const std::string &name, const NodeConfig &conf,
               const RosNodeParams &params)
      : RosActionNode<spot_msgs::action::Trajectory>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({InputPort<double>("distance")});
  }

  bool setGoal(Goal &goal) override {
    auto distance = getInput<double>("distance");

    goal.target_pose.header.frame_id = "body";
    goal.target_pose.pose.position.x = distance.value();

    goal.duration.sec = 4;
    goal.precise_positioning = true;

    return true;
  };

  void onHalt() override {
    RCLCPP_INFO(node_->get_logger(), "%s: onHalt", name().c_str());
  };

  BT::NodeStatus onResultReceived(const WrappedResult &wr) override {
    RCLCPP_INFO(node_->get_logger(), "%s: onResultReceived.", name().c_str());

    return NodeStatus::SUCCESS;
  };

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override {
    RCLCPP_ERROR(node_->get_logger(), "%s: onFailure with error: %s",
                 name().c_str(), toStr(error));
    return NodeStatus::FAILURE;
  };
};

CreateRosNodePlugin(SpotDistance, "SpotDistance");