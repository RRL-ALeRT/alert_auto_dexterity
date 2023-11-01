#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/planning_options.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

using namespace BT;

class ManipulatorJoints: public RosActionNode<moveit_msgs::action::MoveGroup>
{
using MoveGroupAction = moveit_msgs::action::MoveGroup;
using JointConstraint = moveit_msgs::msg::JointConstraint;
using Constraints = moveit_msgs::msg::Constraints;
using MotionPlanRequest = moveit_msgs::msg::MotionPlanRequest;
using PlanningOptions = moveit_msgs::msg::PlanningOptions;

public:
  ManipulatorJoints(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
    : RosActionNode<MoveGroupAction>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("joint_angles")});
  }

  bool setGoal(Goal& goal) override
  {
    std::string input_str;
    getInput("joint_angles", input_str);
    auto split_str = splitString(input_str, ';');

    std::vector<double> target_angles;
    for (const StringView& str : split_str)
    {
      target_angles.push_back(convertFromString<double>(str));
    }

    auto motion_plan_request = MotionPlanRequest();

    // Set up motion plan request and constraints
    motion_plan_request.workspace_parameters.header.stamp = node_->get_clock()->now();
    motion_plan_request.workspace_parameters.header.frame_id = "base_link";
    motion_plan_request.workspace_parameters.min_corner.x = -1.0;
    motion_plan_request.workspace_parameters.min_corner.y = -1.0;
    motion_plan_request.workspace_parameters.min_corner.z = -1.0;
    motion_plan_request.workspace_parameters.max_corner.x = 1.0;
    motion_plan_request.workspace_parameters.max_corner.y = 1.0;
    motion_plan_request.workspace_parameters.max_corner.z = 1.0;
    motion_plan_request.start_state.is_diff = true;

    Constraints constraints;
    // Add joint constraints
    for (size_t i = 0; i < target_angles.size(); ++i) {
      JointConstraint jc;
      jc.joint_name = "joint_" + std::to_string(i + 1);
      jc.position = target_angles[i] * M_PI / 180;
      jc.tolerance_above = 0.001;
      jc.tolerance_below = 0.001;
      jc.weight = 1.0;
      constraints.joint_constraints.push_back(jc);
    }
    motion_plan_request.goal_constraints.push_back(constraints);

    // Set planning options
    motion_plan_request.pipeline_id = "ompl";
    motion_plan_request.group_name = "manipulator";
    motion_plan_request.num_planning_attempts = 4;
    motion_plan_request.allowed_planning_time = 10.0;
    motion_plan_request.max_velocity_scaling_factor = 0.4;
    motion_plan_request.max_acceleration_scaling_factor = 0.4;
    motion_plan_request.max_cartesian_speed = 0.0;

    PlanningOptions planning_options;
    planning_options.plan_only = false;
    planning_options.look_around = true;
    planning_options.look_around_attempts = 5;
    planning_options.max_safe_execution_cost = 0.0;
    planning_options.replan = true;
    planning_options.replan_attempts = 4;
    planning_options.replan_delay = 0.1;

    goal.request = motion_plan_request;
    goal.planning_options = planning_options;

    return true;
  };

  void onHalt() override
  {
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  };

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. %d", name().c_str(), wr.result->error_code.val );

    if (wr.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      return NodeStatus::SUCCESS;
      
    return NodeStatus::FAILURE;
  };

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  };
};

CreateRosNodePlugin(ManipulatorJoints, "ManipulatorJoints");