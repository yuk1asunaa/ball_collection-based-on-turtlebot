# TurtleBot3 Ball Collection Package

This package implements ball collection for TurtleBot3 in simulation with YOLO detection, density-map perception, and Nav2 navigation.

## Features

- YOLO-based multi-target detection from RGB-D topics.
- Gaussian density map construction with time decay.
- Density-aware global planning through a custom Nav2 planner plugin.
- Automatic goal dispatch to `/navigate_to_pose`.

## Runtime Architecture

1. `yolo_detector_node`
- Subscribes camera streams and publishes `/vision/target_poses`.

2. `density_map_builder_node`
- Subscribes `/vision/target_poses`.
- Builds and publishes `/density_map` and `/visualization/density_markers`.
- Sends target goals to Nav2 (`/navigate_to_pose`).

3. Nav2 `planner_server`
- Uses planner ID `GridBased`.
- `GridBased` maps to custom plugin `turtlebot3_ball_collection/GridBased`.
- Plugin implementation class: `turtlebot3_ball_collection::DensityAwareAStarPlanner`.

## Topics

- YOLO input topics (from `launch/ball_collection.launch.py`):
   - `/depth_camera/image_raw`
   - `/depth_camera/depth/image_raw`
   - `/depth_camera/camera_info`
- Vision output:
   - `/vision/target_poses` (`geometry_msgs/msg/PoseArray`)
- Density outputs:
   - `/density_map` (`nav_msgs/msg/OccupancyGrid`)
   - `/visualization/density_markers` (`visualization_msgs/msg/MarkerArray`)

## Dependencies

- ROS 2 Humble
- Nav2
- `ultralytics`
- `opencv`
- `cv_bridge`

## Usage

1. Prepare environment:

```bash
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
source /home/u22/turtlebot/install/setup.bash
ros2 launch turtlebot3_ball_collection full_system.launch.py
```

2. Launch full system:

```bash
ros2 launch turtlebot3_ball_collection full_system.launch.py 2>&1 | tee test.log
```

3. Optional manual teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

4. Stop stack quickly:

```bash
pkill -f 'ros2 launch|gzserver|gazebo|rviz2|slam_toolbox|nav2|yolo'
```
sudo rm -rf /dev/shm/fastrtps*

