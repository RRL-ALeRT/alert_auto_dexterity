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
  params.default_port_value = "/see_with_manipulator";

  RegisterRosNode(factory, "/home/skpawar1305/ros2_ws/install/alert_auto_dexterity/bin/alert_auto_dexterity/libsee_object_set.so", params);

  auto tree = factory.createTreeFromFile("/home/skpawar1305/ros2_ws/src/alert_auto_dexterity/bt/untitled_1.xml");

  // Connect the Groot2Publisher. This will allow Groot2 to
  // get the tree and poll status updates.
  Groot2Publisher publisher(tree);

  for(int i=0; i<5; i++){
    tree.tickWhileRunning();
  }

  return 0;
}