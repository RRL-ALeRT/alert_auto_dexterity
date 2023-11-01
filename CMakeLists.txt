cmake_minimum_required(VERSION 3.16)
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
  "action/TrajectoryToFrame.action"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(depth_to_pointcloud2 cpp/depth_to_pointcloud2.cpp)
ament_target_dependencies(depth_to_pointcloud2 rclcpp rclcpp_lifecycle lifecycle_msgs image_transport sensor_msgs depth_image_proc image_geometry )

function(CompileBTNodes name)
  add_library(${name} SHARED bt_nodes/${name}.cpp )
  target_compile_definitions(${name} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${name}
    behaviortree_ros2
    std_srvs
    lifecycle_msgs
    moveit_msgs
    geometry_msgs
    moveit_msgs
    shape_msgs
    octomap_msgs
  )
  target_link_libraries(${name} "${cpp_typesupport_target}")
endfunction()

set(bt_nodes
  manipulator_action
  stand_in_front
  trigger
  change_state
  apply_planning_scene
  get_octomap
  store_object_list
  manipulator_joints
  trajectory_to_frame
)

foreach(bt_node ${bt_nodes})
  CompileBTNodes(${bt_node})
endforeach()

add_executable(execute_tree bt_nodes/execute.cpp)
target_link_libraries(execute_tree ${bt_nodes})
ament_target_dependencies(execute_tree behaviortree_ros2)

install(TARGETS ${bt_nodes}
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
