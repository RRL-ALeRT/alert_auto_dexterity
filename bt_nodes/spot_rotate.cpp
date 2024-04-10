#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "spot_msgs/action/trajectory.hpp"

#include <tf2/utils.h>

using namespace BT;

class SpotRotate : public RosActionNode<spot_msgs::action::Trajectory> {
public:
  SpotRotate(const std::string &name, const NodeConfig &conf,
             const RosNodeParams &params)
      : RosActionNode<spot_msgs::action::Trajectory>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({InputPort<double>("angle")});
  }

  bool setGoal(Goal &goal) override {
    auto angle = getInput<double>("angle");
    auto z_rot = angle.value();

    double DEG_TO_RAD = M_PI / 180;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, z_rot * DEG_TO_RAD);

    goal.target_pose.header.frame_id = "body";
    goal.target_pose.pose.orientation.x = quat.x();
    goal.target_pose.pose.orientation.y = quat.y();
    goal.target_pose.pose.orientation.z = quat.z();
    goal.target_pose.pose.orientation.w = quat.w();

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

CreateRosNodePlugin(SpotRotate, "SpotRotate");