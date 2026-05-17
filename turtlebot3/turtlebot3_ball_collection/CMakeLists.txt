cmake_minimum_required(VERSION 3.5)
project(turtlebot3_ball_collection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(include)

add_executable(density_map_builder_node src/density_map_builder_node.cpp)
ament_target_dependencies(
  density_map_builder_node
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(mission_controller src/mission_controller.cpp)
ament_target_dependencies(
  mission_controller
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
)


add_library(density_aware_astar_planner SHARED src/density_aware_astar_planner.cpp)
ament_target_dependencies(
  density_aware_astar_planner
  rclcpp
  nav_msgs
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  tf2_ros
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core density_aware_astar_planner_plugin.xml)

install(TARGETS
  density_map_builder_node
  mission_controller
  density_aware_astar_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ball_collector.py
  scripts/mission_metrics.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(FILES density_aware_astar_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()