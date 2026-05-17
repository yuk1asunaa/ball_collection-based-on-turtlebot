# Data Transmission Logic for TurtleBot3 Ball Collection

## Architecture Overview

The system is architected in three cleanly separated layers: Perception, Decision, and Execution.

## Layer 1: Perception

### Camera to YOLO Detector

The three camera topics(`/depth_camera/image_raw`, `/depth_camera/depth/image_raw`, `/depth_camera/camera_info`) feed into the yolo_detector_node, which runs YOLO inference and publishes detected ball poses to `/vision/target_poses` as a PoseArray in the camera_rgb_frame coordinate frame.

Key details:
- YOLO runs inference every N frames (configurable via `process_every_n_frames`).
- Each detection yields a 3D pose in the camera optical frame.
- Even when no balls are visible, an empty PoseArray is published as a heartbeat.
- The depth encoding must match the camera driver output to avoid runtime errors.

### YOLO to Density Map Builder

The density_map_builder_node receives `/vision/target_poses` and transforms each ball position from camera_rgb_frame to the map frame using TF2. For each transformed point, it applies a 2D Gaussian kernel to accumulate density on the internal grid. A periodic timer multiplies all cells by a decay factor. The node publishes three outputs:
- `/density_map`: OccupancyGrid in map frame with transient_local QoS(latched), representing the accumulated density belief.
- `/density/peaks`: PoseArray containing the top-N highest-density cell coordinates.
- `/visualization/density_markers`: MarkerArray for RViz visualization.

Density accumulation formula:

density(x,y) = density(x,y) + increment * exp(-r_squared / (2 * sigma_squared))

where sigma is `gaussian_sigma`(default 0.5m) and increment is `density_increment`(default 10).

Time decay: every `decay_period_sec`(default 1.0s), every cell value is multiplied by `time_decay_factor`(default 0.99). This causes old detections that are not re-observed to gradually fade.

### Ball Collector to Density Feedback

When ball_collector.py confirms a ball has been collected in simulation (by calling the Gazebo `/delete_entity` service), it publishes the ball's world position to `/ball_collected` as a PointStamped in the map frame. The density_map_builder_node subscribes to this topic and calls `remove_gaussian_at()` to subtract the corresponding Gaussian distribution from the density grid. This provides immediate, precise feedback compared to relying solely on time decay.

## Layer 2: Decision (Behavior Tree)

### Mission Controller Node

The mission_controller node is the central decision-making component. It uses BehaviorTree.CPP v3 to compose task behaviors. The BT is initialized once both `/map` and `/density_map` have been received at least once, and it runs on a 50ms wall timer.

BT XML structure:

```xml
<Sequence name="Mission">
  <WaitForMap timeout="30"/>
  <WaitForNav2 timeout="120"/>
  <KeepRunningUntilFailure>
    <Sequence name="MainLoop">
      <CheckBallsRemaining timeout="30"/>
      <Fallback name="SelectTarget">
        <Sequence name="DensityTarget">
          <HasDetections/>
          <SelectBestTarget/>
        </Sequence>
        <ExploreAction/>
      </Fallback>
      <NavigateToTarget/>
    </Sequence>
  </KeepRunningUntilFailure>
</Sequence>
```

Blackboard data flow between BT nodes:
- SelectBestTarget writes the chosen navigation goal to blackboard key `target_pose`(PoseStamped).
- ExploreAction writes a frontier target or current position to the same `target_pose` key.
- NavigateToTarget reads `target_pose` from blackboard and sends the Nav2 action.

### Target Selection Cost Function

SelectBestTarget evaluates each candidate target using a weighted score:

score(p) = alpha * density(p) - beta * distance(p, robot) / 10 - gamma * visit_penalty(p)

visit_penalty(p) = sum over all visited goals v_i of max(0, (R - distance(p, v_i)) / R)

Parameter details:
- alpha(density_weight, default 1.0): Reward for high ball density at the target location. Higher values make the robot strongly prefer dense clusters.
- beta(distance_weight, default 0.5): Penalty for travel distance from the robot's current position. Higher values make the robot favor nearby targets.
- gamma(visit_penalty_weight, default 0.3): Penalty for re-visiting areas close to previously visited goals. Higher values encourage thorough coverage.
- R(visit_radius, default 1.0m): Radius within which the visit penalty is applied. Goals within this distance of any visited point receive a penalty proportional to proximity.

### Exploration Strategy

When no balls are detected, the ExploreAction node does frontier exploration:

Frontier exploration(when `/map` is available): Finds frontier cells(free cells adjacent to unknown) in the SLAM occupancy grid, clusters them using BFS, and selects the nearest frontier cluster. A density bias is applied: clusters in directions with higher density get a slight bonus, naturally favoring exploration toward areas where balls were previously detected.

No-frontier fallback: When no map is available or no frontiers exist, sets target_pose to the robot's current position and returns SUCCESS. NavigateToTarget sees the robot is within 0.3m of the target and returns SUCCESS immediately, causing the BT to loop back and re-check HasDetections — this way newly detected balls are picked up without spinning.

## Layer 3: Execution (Nav2)

### Mission Controller to Nav2

The mission_controller sends navigation action requests to Nav2:
- `/navigate_to_pose`: Navigate to a target pose. Server is bt_navigator.
- `/spin`: Used only by WaitForNav2 for a probe goal to verify server readiness, and by the spin_client infrastructure. The BT does not use spin for exploration or collection.

### DensityAwareAStarPlanner

A custom Nav2 global planner plugin(registered as `turtlebot3_ball_collection/DensityAwareAStarPlanner`) that incorporates density into A* path costs.

Edge cost model:

c(i to j) = d_step(i,j) + w_obs * obs_cost(j) - w_den * density(j)

Cost term breakdown:
- d_step(i,j): Geometric step distance. The base travel cost between adjacent cells(1.0 for cardinal, sqrt(2) for diagonal).
- w_obs * obs_cost(j): Obstacle weight(default 1.0, configurable) multiplied by normalized obstacle cost(0 to 1, with NO_INFORMATION cells using unknown_cost_penalty). Penalizes paths through obstacle-inflated or unknown space.
- w_den * density(j): Density weight(default 0.6) multiplied by normalized density value in [0,1]. A negative term that rewards paths passing through high-density areas.

The A* heuristic is the Euclidean distance to the goal cell multiplied by min_step_cost.

The planner subscribes to `/density_map` and converts raw density values(0-100) to a normalized [0,1] range. High density reduces edge cost, making paths through ball-rich regions cheaper and thus preferred by the planner.

Critical constraint: The final step cost is clamped to be at least `min_step_cost * d_step` to ensure all edges have strictly positive cost, preserving the A* admissibility guarantee.

### Navigation Flow

When NavigateToTarget sends a `/navigate_to_pose` goal:
1. bt_navigator(Nav2's behavior tree) receives the goal and calls ComputePathToPose.
2. planner_server invokes the DensityAwareAStarPlanner, which reads the latest `/density_map` for cost calculation.
3. The planned path is published as `/plan` and passed through smoother_server.
4. controller_server executes the path using DWB local planner, outputting velocity commands to `/cmd_vel`.
5. turtlebot3_diff_drive receives `/cmd_vel` and drives the robot.

The `/spin` action follows a similar flow but bypasses the planner, going directly to the controller for in-place rotation.

## Complete Data Flow

The end-to-end data pipeline works as follows:

The camera driver publishes RGB, depth, and camera info on three separate topics. The yolo_detector_node subscribes to all three, performs time-synchronized inference, and publishes detected ball poses in the camera optical frame.

The density_map_builder_node receives these detections, transforms them to the map frame using TF2 lookups, and accumulates Gaussian blobs on an internal grid. The grid is published as a latched OccupancyGrid, along with top-N peaks and RViz markers. A periodic timer decays all cells. When a `/ball_collected` message arrives(from ball_collector after a successful Gazebo entity deletion), the corresponding Gaussian is explicitly subtracted.

In parallel, SLAM toolbox builds the occupancy grid map from laser scan data and publishes it as `/map`.

The mission_controller node receives the density map, peaks, and SLAM map. Once both `/map` and `/density_map` topics have been received at least once, the behavior tree is initialized. The BT engine ticks at 50ms intervals, alternating between evaluating conditions and executing actions across the mission lifecycle: waiting for the map, performing an initial spin, then looping through target selection, navigation, and collection until no balls remain.

## Parameter Tuning Guide

### Density Map Parameters

When balls are far apart(more than 2m), increase `gaussian_sigma` to 0.8 to 1.0 for wider coverage so nearby detections still form connected clusters.

When balls are close together, decrease `gaussian_sigma` to 0.3 for sharper, more distinct peaks that allow the planner to differentiate individual clusters.

If fast decay is needed(for dynamic environments where balls may move), decrease `time_decay_factor` to 0.95 for faster fading of stale detections.

For a slow, stable belief map, increase `time_decay_factor` to 0.995 for more persistent density that survives brief occlusions when the robot turns away.

### Target Selection Parameters

To prefer dense clusters strongly over distance, increase `density_weight` to 2.0. The robot will travel farther to reach areas with many balls.

To minimize total travel distance, increase `distance_weight` to 1.0. The robot will favor nearby targets even if they have fewer balls.

For thorough coverage with no revisits, increase `visit_penalty_weight` to 0.5. The robot will strongly avoid returning to previously visited areas.

### Planner Parameters

To be more conservative around obstacles, increase `obstacle_weight` to 1.5. Paths will stay farther from walls and obstacles.

To aggressively route through high-density areas, increase the planner's `density_weight` to 0.8. The path will take detours through regions with detected balls.

To avoid unmapped space, increase `unknown_cost_penalty` to 1.5. The planner will prefer known free space even if it means a longer path.
