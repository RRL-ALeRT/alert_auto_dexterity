#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_ros2/plugins.hpp"


using namespace BT;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("Execute");

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;

  params.default_port_value = "/manipulator_action";
  RegisterRosNode(factory, "/home/skpawar1305/ros2_ws/install/alert_auto_dexterity/bin/alert_auto_dexterity/libmanipulator_action.so", params);

  params.default_port_value = "/trigger";
  RegisterRosNode(factory, "/home/skpawar1305/ros2_ws/install/alert_auto_dexterity/bin/alert_auto_dexterity/libtrigger.so", params);

  params.default_port_value = "/change_state";
  RegisterRosNode(factory, "/home/skpawar1305/ros2_ws/install/alert_auto_dexterity/bin/alert_auto_dexterity/libchange_state.so", params);

  params.default_port_value = "/stand_in_front";
  RegisterRosNode(factory, "/home/skpawar1305/ros2_ws/install/alert_auto_dexterity/bin/alert_auto_dexterity/libstand_in_front.so", params);

  auto tree = factory.createTreeFromFile("/home/skpawar1305/ros2_ws/src/alert_auto_dexterity/bt/untitled_1.xml");

  // Connect the Groot2Publisher. This will allow Groot2 to
  // get the tree and poll status updates.
  Groot2Publisher publisher(tree);

  for(int i=0; i<5; i++){
    tree.tickWhileRunning();
  }

  return 0;
}