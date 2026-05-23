# TurtleBot3 Ball Collection Package

基于 Behavior Tree 的未知环境球体收集系统。使用 SLAM toolbox 在线建图、YOLO 视觉检测球体坐标、高斯密度图建模、Nav2 执行导航，通过 BT 决策优先前往高密度区域遍历回收所有球体。

## 系统架构

系统分为三层: 感知层、决策层、执行层。

感知层负责从传感器数据中提取球体位置信息。Camera 采集 RGB-D 图像，YOLO 检测器识别球体并发布三维坐标到 `/vision/target_poses`。DensityMapBuilder 接收这些坐标，用高斯核函数叠加构建密度图，同时施加时间衰减，输出 `/density_map` 密度栅格和 `/density/peaks` 密度峰值。BallCollector 在模拟器中检测机器人近距离范围内的球体并删除，同时向 `/ball_collected` 发布已收集球体的世界坐标，DensityMapBuilder 据此精确移除对应的高斯分布。

决策层是系统的核心大脑，由 MissionController 节点承载，基于 BehaviorTree.CPP v3 引擎运行。行为树结构为: WaitForMap 等待 SLAM 地图就绪，InitialSpin 执行初始 360 度旋转播种密度图，然后进入 KeepRunningUntilFailure 主循环。每轮循环中，CheckBallsRemaining 检查是否还有球体待收集(密度图为空且超时无新检测则终止任务)，然后通过 Fallback 选择目标: 若有检测(HasDetections)则 SelectBestTarget 按密度乘距离评分选出最优目标，否则 ExploreAction 执行 frontier 探索或 spin 搜索。选定目标后 NavigateToTarget 调用 Nav2 导航，到达后 SpinCollect 旋转收集。

执行层由 Nav2 负责。全局规划器使用自定义的 DensityAwareAStarPlanner 插件，路径代价中包含密度奖励项，使规划出的路径自然偏置穿行高密度区域，增加收集球体的效率。

## 节点清单

**yolo_detector** — 可执行文件 `turtlebot3_vision`
RGB-D 图像输入，YOLO 模型推理，输出 `/vision/target_poses`。

**density_map_builder** — 可执行文件 `density_map_builder_node`
订阅 `/vision/target_poses` 和 `/ball_collected`，高斯核叠加构建密度图，定时衰减，发布 `/density_map`、`/density/peaks`、`/visualization/density_markers`。纯粹感知角色，不做导航决策。

**mission_controller** — 可执行文件 `mission_controller`
BT 任务编排节点。订阅 `/density_map`、`/density/peaks`、`/map`，行为树引擎以 50ms 周期 tick。通过 blackboard 在 BT 节点间传递 `target_pose`。

**ball_collector** — 可执行文件 `ball_collector.py`
模拟器专用。订阅 `/gazebo/model_states`，检测距离机器人 0.75m 内的球体，调用 `/delete_entity` 服务删除，发布 `/ball_collected`。

**mission_metrics** — 可执行文件 `mission_metrics.py`
独立统计节点。订阅 `/gazebo/model_states`，追踪球体总数、已收集数、路径长度、耗时、效率，输出 CSV 到 `/tmp/ball_collection_metrics.csv`。

**planner_server** — Nav2 内置
加载 `DensityAwareAStarPlanner` 插件(plugin ID: `turtlebot3_ball_collection/DensityAwareAStarPlanner`)，实现密度感知 A* 全局路径规划。

## Behavior Tree 结构

行为树从根 Sequence 开始依次执行:

第一步: WaitForMap 条件节点，阻塞等待 `/map` 到达，超时 30 秒则任务失败。

第二步: WaitForNav2 条件节点，阻塞等待 Nav2 的 `/navigate_to_pose` 和 `/spin` action server 完全就绪(发送探测 spin 目标验证可接受目标)，超时 120 秒则任务失败。

第三步: KeepRunningUntilFailure 装饰器包裹主循环。其子节点 Sequence 每轮执行以下步骤:

- CheckBallsRemaining 条件节点，检查密度图是否仍有非零单元。密度图为空且超过 `mission_timeout_sec`(默认 60 秒)无新检测时返回 FAILURE，终止任务。
- Fallback 选择器，优先尝试密度目标分支，失败则走探索分支。
- 密度目标分支: HasDetections 条件节点判断是否有可用目标，SelectBestTarget 同步动作节点按评分函数选出最优目标并写入 blackboard 的 `target_pose` 键。
- 探索分支: ExploreAction 动作节点。有 SLAM 地图时做 frontier 探索(寻找已知空闲与未知区域的边界，聚类后选择最近 frontier，施加密度偏置); 无地图或无边界的 fallback 为设置目标为当前位置(使 NavigateToTarget 跳过)，让 BT 循环等待新检测或新 frontier。
- NavigateToTarget 动作节点，从 blackboard 读取 `target_pose`，调用 Nav2 `/navigate_to_pose` action。包含去重逻辑: 若目标距离机器人不足 0.3m 或靠近已访问目标，直接返回 SUCCESS 跳过。到达后 ball_collector 自动收集范围内的球体。

### BT 节点速览

WaitForMap: Condition 节点，输入端口 `timeout`(double,默认30)，等待 SLAM 地图就绪。

WaitForNav2: Condition 节点，输入端口 `timeout`(double,默认120)，等待 Nav2 action server 完全激活。

CheckBallsRemaining: Condition 节点，输入端口 `timeout`(double,默认30)，判断是否还有球待收集。

HasDetections: Condition 节点，检查密度图或 peaks 列表是否非空。

SelectBestTarget: SyncAction 节点，输入端口 `density_weight`、`distance_weight`、`visit_penalty`、`visit_radius`(均为 double)，评分选最优目标写入 blackboard。

NavigateToTarget: StatefulAction 节点，读 blackboard 的 `target_pose`，调 Nav2 导航。含已到达检测和已访问去重。

ExploreAction: StatefulAction 节点，输入端口 `explore_duration`(double,默认120.0)，frontier 探索，无 frontier 时设置当前位置为目标让 BT 循环。

## 核心代价模型

### SelectBestTarget 评分函数

目标评分综合考虑三个因素: 该位置的球体密度(越高越好)、距离机器人的路径长度(越近越好)、是否最近访问过(避免重复)。

score = density_weight * density(p) - distance_weight * distance(p) / 10 - visit_penalty_weight * visit_penalty(p)

其中 visit_penalty(p) = sum over all visited goals v_i of max(0, (visit_radius - distance(p, v_i)) / visit_radius)

- density_weight 默认 1.0，密度评分权重 α
- distance_weight 默认 0.5，距离惩罚权重 β
- visit_penalty_weight 默认 0.3，重复访问惩罚 γ
- visit_radius 默认 1.0m，访问惩罚半径 R

### DensityAwareAStarPlanner 边代价

A* 规划器在标准代价基础上减去密度项，使路径偏置穿过高密度区域:

c(i 到 j) = d_step(i,j) + w_obs * obs_cost(j) - w_den * density(j)

- d_step(i,j): 几何步长，基础移动代价
- w_obs * obs_cost(j): 障碍物权重(默认1.2)乘以归一化障碍代价
- w_den * density(j): 密度权重(默认0.7)乘以归一化密度值[0,1]

步长代价下界约束为 min_step_cost * d_step，确保不出现负边破坏 A* 假设。

## 数据流

Camera 的三个 topic(`/depth_camera/image_raw`、`/depth_camera/depth/image_raw`、`/depth_camera/camera_info`)进入 yolo_detector 节点，经 YOLO 推理后输出 `/vision/target_poses`(PoseArray，坐标系为 camera_rgb_frame)。

density_map_builder 订阅 `/vision/target_poses`，通过 TF2 将球体坐标从 camera_rgb_frame 变换到 map 坐标系，对每个坐标施加二维高斯分布叠加到密度栅格上。同时以 1 秒为周期对所有 cell 乘以 time_decay_factor(默认 0.99)进行衰减。密度图通过 `/density_map`(OccupancyGrid, latched)发布，Top-N 峰值通过 `/density/peaks` 发布，可视化标记通过 `/visualization/density_markers` 发布。

ball_collector 在模拟器中检测到球体进入收集范围(0.75m)后，调用 Gazebo 的 `/delete_entity` 服务删除球体模型，同时向 `/ball_collected` 发布该球的世界坐标。density_map_builder 接收后精确移除对应位置的高斯分布，实现对已收集球体的即时反馈。

mission_controller 订阅 `/density_map`、`/density/peaks` 和 `/map`，在 BT 引擎中运行决策循环。SelectBestTarget 或 ExploreAction 将选中的导航目标(PoseStamped)写入 blackboard 的 `target_pose` 键，NavigateToTarget 读取后调用 Nav2 `/navigate_to_pose` action。Nav2 的 bt_navigator 接收 goal，planner_server 使用 DensityAwareAStarPlanner 规划全局路径(规划时订阅 `/density_map` 获取密度代价)，controller_server 执行局部控制，最终通过 `/cmd_vel` 驱动 turtlebot3_diff_drive。

Spin 动作(InitialSpin、SpinCollect、ExploreAction 的 spin fallback)通过 Nav2 `/spin` action 实现。

## Topics

`/vision/target_poses` — PoseArray，YOLO 发给 DensityMapBuilder，球体坐标在 camera_rgb_frame 下。

`/density_map` — OccupancyGrid，DensityMapBuilder 发给 MissionController，高斯密度图，map 坐标系，transient_local QoS(latched)。

`/density/peaks` — PoseArray，DensityMapBuilder 发给 MissionController，密度 Top-N 峰值坐标。

`/visualization/density_markers` — MarkerArray，DensityMapBuilder 发给 RViz，密度可视化。

`/ball_collected` — PointStamped，BallCollector 发给 DensityMapBuilder，已收集球的世界坐标。

`/map` — OccupancyGrid，SLAM 发给 MissionController，占用栅格地图。

## Actions

`/navigate_to_pose` — mission_controller 作为客户端调用，bt_navigator(Nav2)提供服务。

`/spin` — mission_controller 作为客户端调用，bt_navigator(Nav2)提供服务。

## 参数

### density_map_builder_node

- resolution，默认 0.1，密度图分辨率(m/cell)
- width 和 height，默认 100，密度图尺寸(cells)
- origin_x 和 origin_y，默认 -5.0，密度图原点坐标
- gaussian_sigma，默认 0.5，高斯核标准差
- density_increment，默认 10，每次检测的密度增量
- max_density，默认 100，密度值上限
- time_decay_factor，默认 0.99，每周期衰减乘数
- decay_period_sec，默认 1.0，衰减周期(秒)
- peak_count，默认 5，发布的峰值数量

### mission_controller

- density_weight，默认 1.0，密度评分权重
- distance_weight，默认 0.5，距离惩罚权重
- visit_penalty_weight，默认 0.3，重复访问惩罚
- visit_radius，默认 1.0，访问半径(m)
- spin_angle，默认 6.283，spin 旋转角度(rad)
- collect_duration_sec，默认 15.0，收集阶段持续时长(秒)
- explore_duration_sec，默认 120.0，探索最大时长(秒)
- mission_timeout_sec，默认 60.0，无检测多久后终止任务(秒)
- goal_min_separation，默认 0.5，连续目标最小间距(m)

## 指标

mission_metrics.py 输出 CSV 到 `/tmp/ball_collection_metrics.csv`，包含以下字段:

- balls_total_seen: 历史最大可见球数(个)
- balls_collected: 已收集球数(个)
- coverage_percent: 覆盖率，等于已收集除以最大可见(%)
- path_length_m: 累积路径长度(m)
- elapsed_sec: 从首次检测到完成的总时间(秒)
- efficiency: 效率，等于收集数除以路径长度(balls/m)

## 依赖

- ROS 2 Humble
- Nav2 (navigation2)
- BehaviorTree.CPP v3
- SLAM Toolbox
- YOLO (ultralytics)
- Gazebo (模拟环境)
- turtlebot3_gazebo
- turtlebot3_vision

## 启动

```bash
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
source /home/u22/turtlebot/install/setup.bash
ros2 launch turtlebot3_ball_collection full_system.launch.py 2>&1 | tee test.log
```
ros2 run tf2_tools view_frames
   
## 任务流程

1. 系统启动，Gazebo 加载网球场世界和 TurtleBot3 模型，SLAM toolbox 开始在线建图。
2. mission_controller 等待 `/map` 和 `/density_map` 均收到后初始化行为树。
3. 初始阶段: 机器人执行 360 度原地旋转，YOLO 检测视野中的球体，密度图开始累积高斯分布。
4. 主循环:
   - 有检测目标时，按密度乘距离评分选择最优球体簇，导航前往，到达后旋转收集。
   - 无检测目标时，执行 frontier 探索(向 SLAM 地图的未知边界移动，方向偏好密度较高的区域)或原地 spin 搜索。
5. ball_collector 在机器人经过球体附近时自动删除球体模型，发布 `/ball_collected` 通知密度图精确移除。
6. 密度图归零且超过 mission_timeout_sec 无新检测，任务完成，mission_metrics 输出最终指标。
