cmake_minimum_required(VERSION 3.8)
project(alert_auto_dexterity)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(image_geometry REQUIRED)

find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

find_package(octomap_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/StandInFront.action"
  "action/ManipulatorAction.action"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(depth_to_pointcloud2 cpp/depth_to_pointcloud2.cpp)
ament_target_dependencies(depth_to_pointcloud2 rclcpp rclcpp_lifecycle lifecycle_msgs image_transport sensor_msgs depth_image_proc image_geometry )

add_library(manipulator_action SHARED bt_nodes/manipulator_action.cpp)
target_compile_definitions(manipulator_action PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(manipulator_action behaviortree_ros2)
target_link_libraries(manipulator_action "${cpp_typesupport_target}")

add_library(stand_in_front SHARED bt_nodes/stand_in_front.cpp)
target_compile_definitions(stand_in_front PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(stand_in_front behaviortree_ros2)
target_link_libraries(stand_in_front "${cpp_typesupport_target}")

add_library(trigger SHARED bt_nodes/trigger.cpp)
target_compile_definitions(trigger PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(trigger behaviortree_ros2 std_srvs)

add_library(change_state SHARED bt_nodes/change_state.cpp)
target_compile_definitions(change_state PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(change_state behaviortree_ros2 lifecycle_msgs)

add_library(apply_planning_scene SHARED bt_nodes/apply_planning_scene.cpp)
target_compile_definitions(apply_planning_scene PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(apply_planning_scene behaviortree_ros2 moveit_msgs geometry_msgs moveit_msgs shape_msgs octomap_msgs)

add_library(get_octomap SHARED bt_nodes/get_octomap.cpp)
target_compile_definitions(get_octomap PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(get_octomap behaviortree_ros2 octomap_msgs)

add_executable(execute_tree bt_nodes/execute.cpp)
target_link_libraries(execute_tree manipulator_action stand_in_front trigger change_state apply_planning_scene get_octomap)
ament_target_dependencies(execute_tree behaviortree_ros2)

install(TARGETS
  manipulator_action
  stand_in_front
  trigger
  change_state
  apply_planning_scene
  get_octomap
  DESTINATION bin/${PROJECT_NAME}
)

install(TARGETS
  depth_to_pointcloud2
  execute_tree
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY python/${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY gpp_agents
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY bt
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(behaviortree_ros2)

ament_package()
