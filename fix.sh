#!/bin/bash
sed -i '318,320c\
  static rclcpp::Time last_hack_time = this->now();\
  if ((this->now() - last_hack_time).seconds() < 2.0) { return; }\
  last_hack_time = this->now();\
' turtlebot3/turtlebot3_ball_collection/src/density_map_builder_node.cpp
