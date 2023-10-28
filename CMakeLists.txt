cmake_minimum_required(VERSION 3.8)
project(alert_auto_dexterity)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/StandInFront.action"
  "action/ManipulatorManipulation.action"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

set(dependencies "ament_index_cpp" "geometry_msgs" "rclcpp" "rclcpp_action" "std_msgs" "std_srvs")

add_library(see_object_set SHARED bt_nodes/see_object_set.cpp)
target_compile_definitions(see_object_set PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(see_object_set behaviortree_ros2)
target_link_libraries(see_object_set "${cpp_typesupport_target}")

add_executable(see_object_set_dyn bt_nodes/execute.cpp)
target_link_libraries(see_object_set_dyn see_object_set)
ament_target_dependencies(see_object_set_dyn behaviortree_ros2)

install(TARGETS
  see_object_set
  add_cropped_octomap
  DESTINATION bin/${PROJECT_NAME}
)

install(TARGETS
  see_object_set_dyn
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

ament_export_dependencies(behaviortree_ros2)

ament_package()
