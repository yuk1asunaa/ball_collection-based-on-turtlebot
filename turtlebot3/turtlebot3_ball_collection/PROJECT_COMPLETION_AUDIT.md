# Project Completion Audit (TurtleBot3 Ball Collection)

This checklist defines what "project complete" means for this repository and provides concrete commands to verify each requirement.

## Scope

- Workspace root: `/home/u22/turtlebot`
- Main launch: `turtlebot3_ball_collection/launch/full_system.launch.py`
- Minimal world launch: `turtlebot3_gazebo/launch/turtlebot3_world.launch.py`
- Vision launch: `turtlebot3_ball_collection/launch/ball_collection.launch.py`

## Pre-Check (Run Once)

```bash
source /opt/ros/humble/setup.bash
source /home/u22/turtlebot/install/setup.bash
export TURTLEBOT3_MODEL=waffle
```

Kill stale processes before each audit run:

```bash
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f spawn_entity.py || true
pkill -9 -f robot_state_publisher || true
pkill -9 -f yolo_detector_node || true
pkill -9 -f rgbd_yolo_node || true
pkill -9 -f density_map_builder_node || true
pkill -9 -f nav2_ || true
pkill -9 -f slam_toolbox || true
```

---

## R1. Build and Install Integrity

Requirement: All required packages build from source in this workspace.

Command:

```bash
cd /home/u22/turtlebot
colcon build --packages-select turtlebot3_gazebo turtlebot3_vision turtlebot3_ball_collection
```

Pass criteria:

- Build exits with code `0`.
- No package in the command fails.

Evidence to save:

- Build summary lines.

---

## R2. Workspace Package Precedence

Requirement: Runtime uses workspace-installed packages, not system `/opt/ros` copies.

Commands:

```bash
ros2 pkg prefix turtlebot3_gazebo
ros2 pkg prefix turtlebot3_vision
ros2 pkg prefix turtlebot3_ball_collection
```

Pass criteria:

- Prefixes point to `/home/u22/turtlebot/install/...`.

Evidence to save:

- The three printed prefixes.
/home/u22/turtlebot/install/turtlebot3_gazebo
/home/u22/turtlebot/install/turtlebot3_vision
/home/u22/turtlebot/install/turtlebot3_ball_collection
---

## R3. Single-Instance Bringup Stability

Requirement: System starts cleanly with one instance per critical node.

Command (full stack):

```bash
ros2 launch turtlebot3_ball_collection full_system.launch.py
```

Verification commands:

```bash
ros2 node list
ros2 node list | grep -E 'yolo_detector|density_map_builder|robot_state_publisher|amcl|bt_navigator|planner_server|controller_server' -n
```

Pass criteria:

- Each required node appears once
- No repeated crash/restart logs in first 3 minutes.

Evidence to save:

- `ros2 node list` output and launch log snippets.
/behavior_server
/bt_navigator
/bt_navigator_navigate_through_poses_rclcpp_node
/bt_navigator_navigate_to_pose_rclcpp_node
/camera_driver
/controller_server
/density_map_builder
/depth_camera_driver
/gazebo
/global_costmap/global_costmap
/lifecycle_manager_navigation
/local_costmap/local_costmap
/planner_server
/robot_state_publisher
/rviz2
/rviz_navigation_dialog_action_client
/slam_toolbox
/smoother_server
/transform_listener_impl_56a0cb1fa530
/transform_listener_impl_58fc63d7f960
/transform_listener_impl_59db801d51a0
/transform_listener_impl_59e932baaca0
/transform_listener_impl_5d969ae5f580
/transform_listener_impl_608f4fdf7d70
/transform_listener_impl_63b87de91090
/turtlebot3_diff_drive
/turtlebot3_imu
/turtlebot3_joint_state
/turtlebot3_laserscan
/velocity_smoother
/waypoint_follower
/yolo_detector
---

2:/bt_navigator
3:/bt_navigator_navigate_through_poses_rclcpp_node
4:/bt_navigator_navigate_to_pose_rclcpp_node
6:/controller_server
7:/density_map_builder
13:/planner_server
14:/robot_state_publisher
32:/yolo_detector
---

## R4. Sim Time and Clock Consistency

Requirement: Core nodes use simulation clock.

Commands:

```bash
ros2 topic echo /clock --once
ros2 param get /yolo_detector use_sim_time
ros2 param get /density_map_builder use_sim_time
```

Pass criteria:

- `/clock` is publishing.
- `use_sim_time` is `true` for all simulation-time nodes.

Evidence to save:

- Clock message and parameter outputs.
Boolean value is: True
---

## R5. TF Tree Health

Requirement: TF chain is valid and stable.

Commands:

```bash
ros2 run tf2_tools view_frames
ros2 topic hz /tf
ros2 topic hz /tf_static
```

Pass criteria:

- `frames.pdf` generated successfully.
- Expected frame chain exists: `map -> odom -> base_footprint/base_link` and camera frames.
- No repeated `OLD_DATA` / extrapolation warnings during steady-state run.

Evidence to save:

- `frames.pdf` and terminal warnings (or absence of warnings).

---

## R6. Camera Publish/Subscribe Contract

Requirement: RGB, Depth, CameraInfo are actively published and consumed by YOLO.

Current expected topics in this repo:

- RGB: `/depth_camera/image_raw`
- Depth: `/depth_camera/depth/image_raw`
- CameraInfo: `/depth_camera/camera_info`

Commands:

```bash
ros2 topic info /depth_camera/image_raw
ros2 topic info /depth_camera/depth/image_raw
ros2 topic info /depth_camera/camera_info
ros2 node info /yolo_detector
```

Pass criteria:

- RGB and Depth have `Publisher count >= 1`.
- `/yolo_detector` subscribes to those exact topics.

Evidence to save:

- Topic info outputs and node subscription list.
/depth_camera/image_raw
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 1

/depth_camera/depth/image_raw
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 1

/depth_camera/camera_info
Type: sensor_msgs/msg/CameraInfo
Publisher count: 1
Subscription count: 1

/yolo_detector
  Subscribers:
    /depth_camera/camera_info: sensor_msgs/msg/CameraInfo
    /depth_camera/image_raw: sensor_msgs/msg/Image
    /clock: rosgraph_msgs/msg/Clock
    /depth_camera/depth/image_raw: sensor_msgs/msg/Image
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /vision/target_poses: geometry_msgs/msg/PoseArray
  Service Servers:
    /yolo_detector/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /yolo_detector/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /yolo_detector/get_parameters: rcl_interfaces/srv/GetParameters
    /yolo_detector/list_parameters: rcl_interfaces/srv/ListParameters
    /yolo_detector/set_parameters: rcl_interfaces/srv/SetParameters
    /yolo_detector/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
---

## R7. Vision Processing Validity

Requirement: YOLO node processes synchronized frames without depth encoding errors.

Commands:

```bash
ros2 topic echo /vision/target_poses --once
ros2 topic hz /vision/target_poses
```

Pass criteria:

- Node stays alive and keeps publishing (can be empty poses if no target).
- No repeating runtime errors for unsupported depth encoding.

Evidence to save:

vision/target_poses --once
header:
  stamp:
    sec: 3023
    nanosec: 890000000
  frame_id: camera_rgb_frame
poses: []

---

## R8. Density Map Update and Decay

Requirement: Density map logic updates from detections and decays over time.

Commands:

```bash
ros2 node info /density_map_builder
ros2 topic list | grep density
```

If the package exposes debug/map topics, capture them:

```bash
ros2 topic echo <density_debug_topic> --once
```

Pass criteria:

- `/density_map_builder` is running and receiving `/vision/target_poses`.
- Observed map/density changes after detections and with time decay.

Evidence to save:

/density_map_builder
  Subscribers:
    /clock: rosgraph_msgs/msg/Clock
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /vision/target_poses: geometry_msgs/msg/PoseArray
  Publishers:
    /density_map: nav_msgs/msg/OccupancyGrid
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /visualization/density_markers: visualization_msgs/msg/MarkerArray
  Service Servers:
    /density_map_builder/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /density_map_builder/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /density_map_builder/get_parameters: rcl_interfaces/srv/GetParameters
    /density_map_builder/list_parameters: rcl_interfaces/srv/ListParameters
    /density_map_builder/set_parameters: rcl_interfaces/srv/SetParameters
    /density_map_builder/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
    /navigate_to_pose: nav2_msgs/action/NavigateToPose


grep density
/density_map
/visualization/density_markers
---

## R9. Navigation Behavior

Requirement: Robot navigation behavior is stable and goal-driven (no random drift/freeze loops).

Commands:

```bash
ros2 topic echo /cmd_vel --once
ros2 topic hz /cmd_vel
ros2 action list
ros2 action info /navigate_to_pose
ros2 param get /planner_server planner_plugins
```

Pass criteria:

- Nav action server is available in full-system mode.
- `/cmd_vel` is finite and behavior matches planning logic.
- `planner_plugins` contains `GridBased`.

Evidence to save:
- Action info and `/cmd_vel` samples.

/backup
/compute_path_through_poses
/compute_path_to_pose
/drive_on_heading
/follow_path
/follow_waypoints
/navigate_through_poses
/navigate_to_pose
/smooth_path
/spin
/wait

Action: /navigate_to_pose
Action clients: 5
    /density_map_builder
    /bt_navigator
    /waypoint_follower
    /rviz2
    /rviz_navigation_dialog_action_client
Action servers: 1
    /bt_navigator
---

## R10. Documentation-Implementation Consistency

Requirement: Docs match actual running implementation.

Files to inspect:

- `turtlebot3_ball_collection/DATA_TRANSMISSION_LOGIC.md`
- `turtlebot3_ball_collection/launch/ball_collection.launch.py`
- `turtlebot3_vision/turtlebot3_vision/yolo_detector_node.py`

Pass criteria:

- Topic names in docs match code and runtime.
- "Current implemented" and "future planned" logic are clearly separated.

Evidence to save:

- Short diff or note confirming alignment.

---
