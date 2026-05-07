#!/bin/bash
sed -i 's/if (false) { \/\/ nav_in_progress_ disabled/static rclcpp::Time last_hack_time = this->now(); if ((this->now() - last_hack_time).seconds() < 2.0) { return; } last_hack_time = this->now(); \/\/ HACK/g' turtlebot3/turtlebot3_ball_collection/src/density_map_builder_node.cpp
