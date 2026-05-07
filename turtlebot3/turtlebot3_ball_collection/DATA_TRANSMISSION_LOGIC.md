# Data Transmission Logic for TurtleBot3 Ball Collection

## Overview

This document describes the current data flow and planning split between perception and Nav2.


## System Architecture

1. `yolo_detector_node`
- Reads RGB-D camera topics.
- Publishes detected ball poses in `/vision/target_poses`.

2. `density_map_builder_node`
- Subscribes `/vision/target_poses`.
- Builds Gaussian density map with decay.
- Publishes `/density_map` and markers.
- Sends selected navigation goal to `/navigate_to_pose`.

3. Nav2 `planner_server`
- Receives planning request from BT (`ComputePathToPose`).
- Uses planner ID `GridBased`.
- `GridBased` maps to plugin `turtlebot3_ball_collection/GridBased`.

## Data Flow

1. Camera input to YOLO:
- `color_topic: /depth_camera/image_raw`
- `depth_topic: /depth_camera/depth/image_raw`
- `camera_info_topic: /depth_camera/camera_info`

2. Vision output:
- `/vision/target_poses` (`geometry_msgs/msg/PoseArray`)

3. Density outputs:
- `/density_map` (`nav_msgs/msg/OccupancyGrid`)
- `/visualization/density_markers` (`visualization_msgs/msg/MarkerArray`)

4. Navigation request path:
- `density_map_builder_node` -> `/navigate_to_pose`
- BT -> `ComputePathToPose(planner_id=GridBased)`
- `planner_server` -> custom density-aware A*

## Density-Aware A* Cost Model

For expansion `i -> j`:

`c(i, j) = d_step(i, j) + w_obs * obs_cost(j) - w_den * den_norm(j)`

Where:
- `d_step`: geometric step distance
- `obs_cost`: normalized obstacle or unknown-space penalty
- `den_norm`: normalized density reward in `[0, 1]`

Update equations:
- `g(j) = g(i) + c(i, j)`
- `f(j) = g(j) + h(j)`

Heuristic:
- `h(j)` is Euclidean distance to goal cell.

## Parameter Guidance

- `obstacle_weight`: increase to be more conservative around obstacles.
- `density_weight`: increase to bias paths through high-density regions.
- `unknown_cost_penalty`: increase if unknown space should be strongly avoided.
- `min_step_cost`: keep positive to preserve A* assumptions.

## Launch Notes

- `launch/ball_collection.launch.py`: perception + density builder + collector.
- `launch/slam_navigation.launch.py`: SLAM + Nav2.
- `launch/full_system.launch.py`: full integration entry point.

## Conclusion

Current implementation is: detection and density estimation in custom nodes, global path search in Nav2 plugin (`GridBased`) with density reward.
