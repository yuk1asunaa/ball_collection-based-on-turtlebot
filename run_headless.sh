#!/bin/bash
sudo rm -rf /tmp/.X* /dev/shm/* 2>/dev/null
pkill -9 -f ros2
pkill -9 -f gzserver
pkill -9 -f rviz2

source /home/u22/turtlebot/turtlebot3/install/setup.bash

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/u22/turtlebot/disable_shm.xml
export ROS_LOCALHOST_ONLY=1
export RCUTILS_COLORIZED_OUTPUT=1

echo "Starting Headless Simulation... Please wait for the Gazebo world to load."
xvfb-run -a ros2 launch turtlebot3_ball_collection full_system.launch.py use_sim_time:=true gui:=false
