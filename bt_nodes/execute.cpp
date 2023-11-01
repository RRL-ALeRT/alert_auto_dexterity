#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_ros2/plugins.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace BT;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("Execute");

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;

  std::string bin_directory = ament_index_cpp::get_package_prefix("alert_auto_dexterity") + "/bin/alert_auto_dexterity/";

  params.default_port_value = "/manipulator_action";
  RegisterRosNode(factory, bin_directory + "libmanipulator_action.so", params);

  params.default_port_value = "/trigger";
  RegisterRosNode(factory, bin_directory + "libtrigger.so", params);

  params.default_port_value = "/change_state";
  RegisterRosNode(factory, bin_directory + "libchange_state.so", params);

  params.default_port_value = "/stand_in_front";
  RegisterRosNode(factory, bin_directory + "libstand_in_front.so", params);

  params.default_port_value = "/apply_planning_scene";
  RegisterRosNode(factory, bin_directory + "libapply_planning_scene.so", params);

  params.default_port_value = "/get_octomap";
  RegisterRosNode(factory, bin_directory + "libget_octomap.so", params);

  params.default_port_value = "/manipulator_joints";
  RegisterRosNode(factory, bin_directory + "libmanipulator_joints.so", params);

  params.default_port_value = "/trajectory_to_frame";
  RegisterRosNode(factory, bin_directory + "libtrajectory_to_frame.so", params);

  factory.registerFromPlugin(bin_directory + "libstore_object_list.so");

  auto tree = factory.createTreeFromFile(ament_index_cpp::get_package_share_directory("alert_auto_dexterity") + "/bt/untitled_1.xml");

  // Connect the Groot2Publisher. This will allow Groot2 to
  // get the tree and poll status updates.
  Groot2Publisher publisher(tree);

  for (int i = 0; i < 2; i++)
    tree.tickWhileRunning();

  return 0;
}