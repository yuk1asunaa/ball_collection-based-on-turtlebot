# Project Completion Audit (TurtleBot3 Ball Collection)

This checklist defines what "project complete" means for this repository and provides concrete commands to verify each requirement.

## Scope

- Workspace root: `/home/u22/turtlebot`
- Main launch: `turtlebot3_ball_collection/launch/full_system.launch.py`

## Architecture Summary

The system follows a three-layer architecture:

- Perception: Camera to YOLO to DensityMapBuilder, outputting density map and peaks. BallCollector feeds back collected positions via `/ball_collected`.
- Decision: MissionController runs a Behavior Tree that selects targets, triggers exploration, and sends navigation goals.
- Execution: Nav2 with DensityAwareAStarPlanner handles path planning and control, outputting velocity commands.
- Collection: BallCollector deletes Gazebo entities and publishes collection confirmations.
- Metrics: MissionMetrics independently tracks coverage, time, path length, and efficiency.

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
pkill -9 -f density_map_builder_node || true
pkill -9 -f mission_controller || true
pkill -9 -f nav2_ || true
pkill -9 -f slam_toolbox || true
pkill -9 -f ball_collector || true
```

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
- The `mission_controller` executable is produced(check `install/turtlebot3_ball_collection/lib/turtlebot3_ball_collection/`).
- The `task_manager` executable is NOT present(removed and replaced by mission_controller).

## R2. Workspace Package Precedence

Requirement: Runtime uses workspace-installed packages, not system `/opt/ros` copies.

Commands:

```bash
ros2 pkg prefix turtlebot3_gazebo
ros2 pkg prefix turtlebot3_vision
ros2 pkg prefix turtlebot3_ball_collection
```

Pass criteria: Prefixes point to `/home/u22/turtlebot/install/...`.

## R3. Single-Instance Bringup Stability

Requirement: System starts cleanly with one instance per critical node.

Command(full stack):

```bash
ros2 launch turtlebot3_ball_collection full_system.launch.py
```

Verification commands:

```bash
ros2 node list
ros2 node list | grep -E 'yolo_detector|density_map_builder|mission_controller|robot_state_publisher|bt_navigator|planner_server|controller_server|slam_toolbox|ball_collector|mission_metrics' -n
```

Pass criteria:
- Each required node appears once.
- `mission_controller` is present(NOT `task_manager`).
- No repeated crash or restart logs in first 3 minutes.

Expected nodes(minimum):
- /yolo_detector
- /density_map_builder
- /mission_controller
- /ball_collector
- /mission_metrics
- /bt_navigator
- /planner_server
- /controller_server
- /slam_toolbox
- /robot_state_publisher

## R4. Sim Time and Clock Consistency

Requirement: Core nodes use simulation clock.

Commands:

```bash
ros2 topic echo /clock --once
ros2 param get /yolo_detector use_sim_time
ros2 param get /density_map_builder use_sim_time
ros2 param get /mission_controller use_sim_time
```

Pass criteria:
- `/clock` is publishing.
- `use_sim_time` is `true` for all three nodes.

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
- Expected frame chain exists: `map` to `odom` to `base_footprint` or `base_link`, plus camera frames.
- No repeated `OLD_DATA` or extrapolation warnings during steady-state run.

## R6. Camera Publish and Subscribe Contract

Requirement: RGB, Depth, and CameraInfo are actively published and consumed by YOLO.

Current expected topics:
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
- RGB and Depth have Publisher count at least 1.
- `/yolo_detector` subscribes to those exact topics.

## R7. Vision Processing Validity

Requirement: YOLO node processes synchronized frames without depth encoding errors.

Commands:

```bash
ros2 topic echo /vision/target_poses --once
ros2 topic hz /vision/target_poses
```

Pass criteria:
- Node stays alive and keeps publishing(can be empty poses if no target).
- No repeating runtime errors for unsupported depth encoding.

## R8. Density Map Update and Decay

Requirement: Density map logic updates from detections, decays over time, and responds to explicit ball collection removal.

Commands:

```bash
ros2 node info /density_map_builder
ros2 topic list | grep density
ros2 topic list | grep ball_collected
```

Pass criteria:
- `/density_map_builder` subscribes to `/vision/target_poses` AND `/ball_collected`.
- Publishes `/density_map`, `/density/peaks`, `/visualization/density_markers`.
- `/density_map` is latched(transient_local QoS).
- Does NOT have navigation or spin action clients(moved to mission_controller).
- Density grows with detections, decays over time, and cells are removed when `/ball_collected` is received.

## R9. Mission Controller (Behavior Tree)

Requirement: BT-based mission controller orchestrates the full task.

Commands:

```bash
ros2 node info /mission_controller
ros2 action list | grep -E 'navigate_to_pose|spin'
```

Pass criteria:
- `/mission_controller` subscribes to `/density_map`, `/density/peaks`, `/map`.
- `/mission_controller` has action clients for `/navigate_to_pose` and `/spin`.
- BT initialization logs appear(e.g. "BT initialized, starting mission").
- Node does NOT crash within first 5 minutes.
- BT cycles through: WaitForMap to WaitForNav2 to the collect or explore loop.

Task flow validation:
1. Robot waits for SLAM map and Nav2 servers, then enters the main collect/explore loop.
2. If balls detected, navigates to highest-scored target using the density times distance scoring function.
3. At target, ball_collector automatically collects nearby balls and triggers `/ball_collected`.
4. Collected balls trigger `/ball_collected`, density map removes them.
5. If no detections, explores frontiers; if no frontiers, idles until new detections appear.
6. Mission ends when no balls remain for `mission_timeout_sec`.

## R10. Ball Collection Feedback Loop

Requirement: When a ball is collected in simulation, its position is published to `/ball_collected` and the density map removes it.

Commands:

```bash
ros2 node info /ball_collector
ros2 topic echo /ball_collected --once
```

Pass criteria:
- `/ball_collector` publishes to `/ball_collected`(PointStamped).
- `/ball_collector` uses Gazebo `/delete_entity` service.
- `/density_map_builder` subscribes to `/ball_collected`.
- After collection, the corresponding Gaussian blob is removed from the density map.

## R11. Navigation Behavior

Requirement: Robot navigation is stable and goal-driven.

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
- `planner_plugins` includes `DensityAwareAStarPlanner`(turtlebot3_ball_collection/DensityAwareAStarPlanner).
- Action clients on `/navigate_to_pose` include `mission_controller`.
- Paths pass through high-density areas when possible(observable in RViz).

## R12. Metrics Output

Requirement: Mission metrics are correctly tracked and exported.

Commands:

```bash
ros2 node info /mission_metrics
cat /tmp/ball_collection_metrics.csv | tail -5
```

Pass criteria:
- `/mission_metrics` subscribes to `/gazebo/model_states`.
- CSV contains: `balls_total_seen`, `balls_collected`, `balls_remaining`, `coverage_percent`, `path_length_m`, `elapsed_sec`, `efficiency_balls_per_m`.
- Coverage approaches 100 percent when all balls are collected.
- Final row has `final=1`.

## R13. Documentation-Implementation Consistency

Requirement: Docs match actual running implementation.

Files to inspect:
- `turtlebot3_ball_collection/README.md`: architecture, nodes, topics, BT structure, parameters.
- `turtlebot3_ball_collection/DATA_TRANSMISSION_LOGIC.md`: data flow, BT logic, cost model, tuning guide.
- `turtlebot3_ball_collection/launch/ball_collection.launch.py`: node parameters.
- `turtlebot3_ball_collection/CMakeLists.txt`: build targets(no task_manager, yes mission_controller).

Pass criteria:
- README describes three-layer architecture(Perception, Decision, Execution).
- DATA_TRANSMISSION_LOGIC describes BT node data flow and cost functions.
- Launch file parameters match documented defaults.
- No stale references to `task_manager` node anywhere.
