#include "mission_controller.hpp"
#include <cmath>
#include <queue>
#include <set>
#include <string>
#include <behaviortree_cpp_v3/bt_factory.h>

using namespace std::chrono_literals;

// ── Helpers ──

static std::shared_ptr<MissionContext> ctxFromBB(BT::TreeNode* node)
{
  auto bb = node->config().blackboard;
  std::shared_ptr<MissionContext> ctx;
  bb->get("mission_ctx", ctx);
  return ctx;
}

static const char* kDefaultBTXML = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="Mission">
      <WaitForMap timeout="30"/>
      <WaitForNav2 timeout="120"/>
      <InitialSpin spin_angle="6.283"/>
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
  </BehaviorTree>
</root>
)";

// ── MissionControllerNode ──

MissionControllerNode::MissionControllerNode()
  : Node("mission_controller")
{
  ctx_ = std::make_shared<MissionContext>();
  ctx_->node = this;

  this->declare_parameter<double>("density_weight", 1.0);
  this->declare_parameter<double>("distance_weight", 0.5);
  this->declare_parameter<double>("visit_penalty_weight", 0.3);
  this->declare_parameter<double>("visit_radius", 1.0);
  this->declare_parameter<double>("spin_angle", 2.0 * M_PI);
  this->declare_parameter<double>("collect_duration_sec", 15.0);
  this->declare_parameter<double>("explore_duration_sec", 120.0);
  this->declare_parameter<double>("mission_timeout_sec", 60.0);
  this->declare_parameter<double>("goal_min_separation", 0.5);

  this->get_parameter("density_weight", ctx_->density_weight);
  this->get_parameter("distance_weight", ctx_->distance_weight);
  this->get_parameter("visit_penalty_weight", ctx_->visit_penalty_weight);
  this->get_parameter("visit_radius", ctx_->visit_radius);
  this->get_parameter("spin_angle", ctx_->spin_angle);
  this->get_parameter("collect_duration_sec", ctx_->collect_duration_sec);
  this->get_parameter("explore_duration_sec", ctx_->explore_duration_sec);
  this->get_parameter("mission_timeout_sec", ctx_->mission_timeout_sec);
  this->get_parameter("goal_min_separation", ctx_->goal_min_separation);

  ctx_->nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "/navigate_to_pose");
  ctx_->spin_client = rclcpp_action::create_client<nav2_msgs::action::Spin>(
    this, "/spin");

  ctx_->density_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/density_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
    std::bind(&MissionControllerNode::density_callback, this, std::placeholders::_1));

  ctx_->peaks_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/density/peaks", 10,
    std::bind(&MissionControllerNode::peaks_callback, this, std::placeholders::_1));

  ctx_->mark_pub = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/density/mark_collected", 10);

  ctx_->tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  ctx_->tf_listener = std::make_shared<tf2_ros::TransformListener>(*ctx_->tf_buffer);

  ctx_->last_detection_time = this->now();

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
    std::bind(&MissionControllerNode::map_callback, this, std::placeholders::_1));

  // Register BT nodes
  factory_.registerNodeType<WaitForMap>("WaitForMap");
  factory_.registerNodeType<WaitForNav2>("WaitForNav2");
  factory_.registerNodeType<CheckBallsRemaining>("CheckBallsRemaining");
  factory_.registerNodeType<HasDetections>("HasDetections");
  factory_.registerNodeType<InitialSpin>("InitialSpin");
  factory_.registerNodeType<SelectBestTarget>("SelectBestTarget");
  factory_.registerNodeType<NavigateToTarget>("NavigateToTarget");
  factory_.registerNodeType<SpinCollect>("SpinCollect");
  factory_.registerNodeType<ExploreAction>("ExploreAction");

  RCLCPP_INFO(this->get_logger(),
    "MissionController ready: density_w=%.2f distance_w=%.2f visit_w=%.2f timeout=%.1fs",
    ctx_->density_weight, ctx_->distance_weight, ctx_->visit_penalty_weight,
    ctx_->mission_timeout_sec);
}

void MissionControllerNode::density_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(ctx_->mtx);
  ctx_->latest_density = msg;

  // Check if density map has any non-zero cells
  bool has_content = false;
  for (auto v : msg->data) {
    if (v > 0) { has_content = true; break; }
  }
  if (has_content) {
    ctx_->last_detection_time = this->now();
    ctx_->has_seen_detection = true;
  }

  // Initialize BT on first density map (map is available, system is ready)
  if (!bt_initialized_ && ctx_->latest_map) {
    bt_initialized_ = true;
    auto bb = BT::Blackboard::create();
    bb->set("mission_ctx", ctx_);
    tree_ = factory_.createTreeFromText(kDefaultBTXML, bb);
    RCLCPP_INFO(this->get_logger(), "BT initialized, starting mission");

    bt_timer_ = this->create_wall_timer(50ms,
      std::bind(&MissionControllerNode::tick_tree, this));
  }
}

void MissionControllerNode::peaks_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(ctx_->mtx);
  ctx_->latest_peaks = *msg;
}

void MissionControllerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(ctx_->mtx);
  ctx_->latest_map = msg;

  // Initialize BT on first map if density already received
  if (!bt_initialized_ && ctx_->latest_density) {
    bt_initialized_ = true;
    auto bb = BT::Blackboard::create();
    bb->set("mission_ctx", ctx_);
    tree_ = factory_.createTreeFromText(kDefaultBTXML, bb);
    RCLCPP_INFO(this->get_logger(), "BT initialized (from map), starting mission");

    bt_timer_ = this->create_wall_timer(50ms,
      std::bind(&MissionControllerNode::tick_tree, this));
  }
}

void MissionControllerNode::tick_tree()
{
  if (!bt_initialized_) return;

  BT::NodeStatus status = tree_.tickRoot();

  if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
    RCLCPP_INFO(this->get_logger(),
      "Mission ended: %s",
      status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
    ctx_->mission_complete = true;
    bt_timer_->cancel();

    // Output final stats
    std::lock_guard<std::mutex> lock(ctx_->mtx);
    RCLCPP_INFO(this->get_logger(),
      "=== MISSION COMPLETE === visited_goals=%zu seen_detections=%s",
      ctx_->visited_goals.size(),
      ctx_->has_seen_detection ? "yes" : "no");
  }
}

// ── Condition: WaitForMap ──

WaitForMap::WaitForMap(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList WaitForMap::providedPorts()
{
  return { BT::InputPort<double>("timeout", 30.0, "Timeout in seconds") };
}

BT::NodeStatus WaitForMap::tick()
{
  auto ctx = ctxFromBB(this);
  if (!ctx) return BT::NodeStatus::FAILURE;

  double timeout_sec;
  getInput("timeout", timeout_sec);

  std::lock_guard<std::mutex> lock(ctx->mtx);
  if (ctx->latest_map) {
    RCLCPP_INFO(ctx->node->get_logger(), "SLAM map received, proceeding");
    return BT::NodeStatus::SUCCESS;
  }

  if (!started_) {
    start_time_ = ctx->node->now();
    started_ = true;
  }

  auto elapsed = (ctx->node->now() - start_time_).seconds();
  if (elapsed > timeout_sec) {
    RCLCPP_WARN(ctx->node->get_logger(), "Timeout waiting for SLAM map (%.1fs)", elapsed);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

// ── Condition: WaitForNav2 ──
// Nav2 lifecycle nodes advertise action servers during on_configure, so
// wait_for_action_server() returns true before the node is active. Goals
// sent to an inactive server are silently dropped.  This node sends probe
// goals to both spin (behavior_server) and navigate_to_pose (bt_navigator)
// to verify both servers can actually accept goals before proceeding.

WaitForNav2::WaitForNav2(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList WaitForNav2::providedPorts()
{
  return { BT::InputPort<double>("timeout", 120.0, "Timeout in seconds") };
}

BT::NodeStatus WaitForNav2::tick()
{
  auto ctx = ctxFromBB(this);
  if (!ctx) return BT::NodeStatus::FAILURE;

  double timeout_sec;
  getInput("timeout", timeout_sec);

  if (!started_) {
    start_time_ = ctx->node->now();
    started_ = true;
    RCLCPP_INFO(ctx->node->get_logger(), "Waiting for Nav2 servers to be fully ready...");
  }

  // Phase 1 – wait for action servers to exist
  if (!nav_exists_) {
    nav_exists_ = ctx->nav_client->wait_for_action_server(0s);
  }
  if (!spin_exists_) {
    spin_exists_ = ctx->spin_client->wait_for_action_server(0s);
  }

  if (nav_exists_ && spin_exists_) {
    // Phase 2 – probe spin (verifies behavior_server is active)
    if (!spin_ready_ && !probe_sent_) {
      nav2_msgs::action::Spin::Goal probe;
      probe.target_yaw = 0.01;
      probe.time_allowance = rclcpp::Duration::from_seconds(0.5);

      auto opts = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
      opts.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr gh) {
        if (gh) spin_ready_ = true;
        probe_done_ = true;
      };
      opts.result_callback = [this](auto) { probe_done_ = true; };

      ctx->spin_client->async_send_goal(probe, opts);
      probe_sent_ = true;
    }

    if (probe_done_ && !spin_ready_) {
      probe_sent_ = false;
      probe_done_ = false;
      RCLCPP_DEBUG(ctx->node->get_logger(), "Nav2 spin probe rejected, retrying...");
    }

    // Phase 3 – probe navigate_to_pose (verifies bt_navigator is active)
    if (spin_ready_ && !nav_probe_sent_) {
      nav2_msgs::action::NavigateToPose::Goal nav_probe;
      nav_probe.pose.header.frame_id = "map";
      nav_probe.pose.header.stamp = rclcpp::Time(0, 0, ctx->node->get_clock()->get_clock_type());
      nav_probe.pose.pose.orientation.w = 1.0;

      auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      opts.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr gh) {
        if (gh) {
          nav_ready_ = true;
        }
        nav_probe_done_ = true;
      };
      opts.result_callback = [this](auto) { nav_probe_done_ = true; };

      ctx->nav_client->async_send_goal(nav_probe, opts);
      nav_probe_sent_ = true;
    }

    if (nav_probe_done_ && !nav_ready_) {
      nav_probe_sent_ = false;
      nav_probe_done_ = false;
      RCLCPP_DEBUG(ctx->node->get_logger(), "Nav2 nav probe rejected, retrying...");
    }

    if (spin_ready_ && nav_ready_) {
      RCLCPP_INFO(ctx->node->get_logger(), "Nav2 action servers fully ready, proceeding");
      return BT::NodeStatus::SUCCESS;
    }
  }

  auto elapsed = (ctx->node->now() - start_time_).seconds();
  if (elapsed > timeout_sec) {
    RCLCPP_WARN(ctx->node->get_logger(),
      "Timeout waiting for Nav2 servers (%.1fs)", elapsed);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

// ── Condition: CheckBallsRemaining ──

CheckBallsRemaining::CheckBallsRemaining(const std::string& name,
                                         const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList CheckBallsRemaining::providedPorts()
{
  return { BT::InputPort<double>("timeout", 30.0, "Seconds without detections before giving up") };
}

BT::NodeStatus CheckBallsRemaining::tick()
{
  auto ctx = ctxFromBB(this);
  if (!ctx) return BT::NodeStatus::FAILURE;

  double timeout_sec;
  getInput("timeout", timeout_sec);

  std::lock_guard<std::mutex> lock(ctx->mtx);

  if (!ctx->has_seen_detection) {
    return BT::NodeStatus::SUCCESS;  // haven't started yet, keep going
  }

  // Check density map for remaining content
  if (ctx->latest_density) {
    for (auto v : ctx->latest_density->data) {
      if (v > 0) {
        return BT::NodeStatus::SUCCESS;  // still have targets
      }
    }
  }

  // Density is empty — check if we've timed out
  double elapsed = (ctx->node->now() - ctx->last_detection_time).seconds();
  if (elapsed > timeout_sec) {
    RCLCPP_INFO(ctx->node->get_logger(),
      "No targets for %.1fs, mission complete", elapsed);
    return BT::NodeStatus::FAILURE;
  }

  // Still waiting
  return BT::NodeStatus::SUCCESS;
}

// ── Condition: HasDetections ──

HasDetections::HasDetections(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList HasDetections::providedPorts()
{
  return {};
}

BT::NodeStatus HasDetections::tick()
{
  auto ctx = ctxFromBB(this);
  if (!ctx) return BT::NodeStatus::FAILURE;

  std::lock_guard<std::mutex> lock(ctx->mtx);

  if (!ctx->latest_peaks.poses.empty()) {
    return BT::NodeStatus::SUCCESS;
  }

  // Fallback: check raw density map
  if (ctx->latest_density) {
    for (auto v : ctx->latest_density->data) {
      if (v > 0) return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}

// ── Action: InitialSpin ──

InitialSpin::InitialSpin(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config) {}

BT::PortsList InitialSpin::providedPorts()
{
  return { BT::InputPort<double>("spin_angle", 6.283, "Spin angle in rad") };
}

std::shared_ptr<MissionContext> InitialSpin::getContext()
{
  return ctxFromBB(this);
}

BT::NodeStatus InitialSpin::onStart()
{
  auto ctx = getContext();
  if (!ctx) return BT::NodeStatus::FAILURE;

  getInput("spin_angle", spin_angle_);

  if (!ctx->spin_client->wait_for_action_server(5s)) {
    RCLCPP_WARN(ctx->node->get_logger(), "Spin server not available for initial spin");
    return BT::NodeStatus::FAILURE;
  }

  nav2_msgs::action::Spin::Goal goal;
  goal.target_yaw = spin_angle_;
  goal.time_allowance = rclcpp::Duration::from_seconds(30.0);

  auto opts = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
  opts.result_callback = [this](auto) {
    spin_done_ = true;
    spin_success_ = true;
  };
  opts.goal_response_callback = [this](auto future) {
    if (!future.get()) {
      RCLCPP_WARN(rclcpp::get_logger("InitialSpin"), "Spin goal rejected by server");
      spin_done_ = true;
      spin_success_ = false;
    }
  };

  spin_done_ = false;
  spin_success_ = false;
  spin_start_ = ctx->node->now();

  ctx->spin_client->async_send_goal(goal, opts);
  RCLCPP_INFO(ctx->node->get_logger(),
    "InitialSpin: starting %.2f rad rotation", spin_angle_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus InitialSpin::onRunning()
{
  auto ctx = getContext();
  if (!ctx) return BT::NodeStatus::FAILURE;

  if (spin_done_) {
    RCLCPP_INFO(ctx->node->get_logger(), "InitialSpin: done");
    return spin_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  // Timeout after 20s
  if ((ctx->node->now() - spin_start_).seconds() > 20.0) {
    RCLCPP_WARN(ctx->node->get_logger(), "InitialSpin: timeout");
    return BT::NodeStatus::SUCCESS;  // proceed anyway
  }

  return BT::NodeStatus::RUNNING;
}

void InitialSpin::onHalted()
{
  auto ctx = getContext();
  if (ctx) {
    ctx->spin_client->async_cancel_all_goals();
  }
}

// ── Action: SelectBestTarget ──

SelectBestTarget::SelectBestTarget(const std::string& name,
                                   const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList SelectBestTarget::providedPorts()
{
  return {
    BT::InputPort<double>("density_weight", 1.0, "Weight for density score"),
    BT::InputPort<double>("distance_weight", 0.5, "Weight for distance penalty"),
    BT::InputPort<double>("visit_penalty", 0.3, "Weight for visit penalty"),
    BT::InputPort<double>("visit_radius", 1.0, "Radius for visit penalty in meters"),
  };
}

std::shared_ptr<MissionContext> SelectBestTarget::getContext() const
{
  return ctxFromBB(const_cast<SelectBestTarget*>(this));
}

double SelectBestTarget::density_at_world(const geometry_msgs::msg::Point& p,
  const nav_msgs::msg::OccupancyGrid& map) const
{
  double rx = (p.x - map.info.origin.position.x) / map.info.resolution;
  double ry = (p.y - map.info.origin.position.y) / map.info.resolution;
  if (rx < 0 || ry < 0 || rx >= map.info.width || ry >= map.info.height) return 0.0;

  int idx = static_cast<int>(ry) * map.info.width + static_cast<int>(rx);
  return static_cast<double>(map.data[idx]) / 100.0;
}

double SelectBestTarget::visit_penalty(const geometry_msgs::msg::Point& p) const
{
  auto ctx = getContext();
  if (!ctx) return 0.0;

  double penalty = 0.0;
  for (const auto& v : ctx->visited_goals) {
    double d = std::hypot(p.x - v.x, p.y - v.y);
    if (d < ctx->visit_radius) {
      penalty += (ctx->visit_radius - d) / ctx->visit_radius;
    }
  }
  return penalty;
}

bool SelectBestTarget::is_far_enough(const geometry_msgs::msg::Point& p) const
{
  auto ctx = getContext();
  if (!ctx || ctx->visited_goals.empty()) return true;
  const auto& last = ctx->visited_goals.back();
  return std::hypot(p.x - last.x, p.y - last.y) >= ctx->goal_min_separation;
}

BT::NodeStatus SelectBestTarget::tick()
{
  auto ctx = getContext();
  if (!ctx) return BT::NodeStatus::FAILURE;

  double density_weight, distance_weight, visit_penalty_weight, visit_radius;
  getInput("density_weight", density_weight);
  getInput("distance_weight", distance_weight);
  getInput("visit_penalty", visit_penalty_weight);
  getInput("visit_radius", visit_radius);

  ctx->density_weight = density_weight;
  ctx->distance_weight = distance_weight;
  ctx->visit_penalty_weight = visit_penalty_weight;
  ctx->visit_radius = visit_radius;

  {
    std::lock_guard<std::mutex> lock(ctx->mtx);

    // Get robot position from TF
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.pose.orientation.w = 1.0;
    try {
      auto t = ctx->tf_buffer->lookupTransform("map", "base_link",
        tf2::TimePointZero, tf2::durationFromSec(0.5));
      robot_pose.pose.position.x = t.transform.translation.x;
      robot_pose.pose.position.y = t.transform.translation.y;
    } catch (tf2::TransformException&) {
      robot_pose.pose.position.x = 0;
      robot_pose.pose.position.y = 0;
    }

    // Build candidate list from peaks + density map
    struct Candidate {
      geometry_msgs::msg::Point pt;
      double density;
      double distance;
      double score;
    };
    std::vector<Candidate> candidates;

    // From peaks
    for (const auto& pose : ctx->latest_peaks.poses) {
      Candidate c;
      c.pt = pose.position;
      c.density = density_at_world(c.pt, *ctx->latest_density);
      c.distance = std::hypot(
        c.pt.x - robot_pose.pose.position.x,
        c.pt.y - robot_pose.pose.position.y);
      c.score = density_weight * c.density
              - distance_weight * c.distance / 10.0
              - visit_penalty_weight * visit_penalty(c.pt);
      candidates.push_back(c);
    }

    // If no peaks, scan density map
    if (candidates.empty() && ctx->latest_density) {
      const auto& map = *ctx->latest_density;
      for (int i = 0; i < static_cast<int>(map.data.size()); ++i) {
        if (map.data[i] <= 0) continue;
        int x = i % map.info.width;
        int y = i / map.info.width;
        Candidate c;
        c.pt.x = map.info.origin.position.x + (x + 0.5) * map.info.resolution;
        c.pt.y = map.info.origin.position.y + (y + 0.5) * map.info.resolution;
        c.pt.z = 0;
        c.density = map.data[i] / 100.0;
        c.distance = std::hypot(
          c.pt.x - robot_pose.pose.position.x,
          c.pt.y - robot_pose.pose.position.y);
        c.score = density_weight * c.density
                - distance_weight * c.distance / 10.0
                - visit_penalty_weight * visit_penalty(c.pt);
        candidates.push_back(c);
      }
    }

    if (candidates.empty()) {
      return BT::NodeStatus::FAILURE;
    }

    // Pick best
    std::sort(candidates.begin(), candidates.end(),
      [](const Candidate& a, const Candidate& b) { return a.score > b.score; });

    // Filter by minimum separation from last goal
    for (auto& c : candidates) {
      if (is_far_enough(c.pt)) {
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "map";
        target.pose.position = c.pt;
        target.pose.orientation.w = 1.0;
        config().blackboard->set("target_pose", target);

        RCLCPP_INFO(ctx->node->get_logger(),
          "Selected target (%.2f,%.2f) density=%.2f dist=%.2fm score=%.2f",
          c.pt.x, c.pt.y, c.density, c.distance, c.score);
        return BT::NodeStatus::SUCCESS;
      }
    }

    return BT::NodeStatus::FAILURE;
  }
}

// ── Action: NavigateToTarget ──

NavigateToTarget::NavigateToTarget(const std::string& name,
                                   const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config) {}

BT::PortsList NavigateToTarget::providedPorts()
{
  return {};
}

std::shared_ptr<MissionContext> NavigateToTarget::getContext()
{
  return ctxFromBB(this);
}

void NavigateToTarget::sendGoal()
{
  auto ctx = getContext();
  if (!ctx) return;

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = rclcpp::Time(0, 0, ctx->node->get_clock()->get_clock_type());
  goal.pose.pose.position = current_goal_;
  goal.pose.pose.orientation.w = 1.0;

  auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  opts.result_callback = [this](auto result) {
    nav_done_ = true;
    nav_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
  };
  opts.goal_response_callback = [this](auto future) {
    if (!future.get()) {
      goal_rejected_ = true;
    }
    goal_sent_ = true;
  };

  ctx->nav_client->async_send_goal(goal, opts);
}

BT::NodeStatus NavigateToTarget::onStart()
{
  auto ctx = getContext();
  if (!ctx) return BT::NodeStatus::FAILURE;

  geometry_msgs::msg::PoseStamped target;
  if (!config().blackboard->get("target_pose", target)) {
    RCLCPP_WARN(ctx->node->get_logger(), "NavigateToTarget: no target_pose in blackboard");
    return BT::NodeStatus::FAILURE;
  }

  current_goal_ = target.pose.position;

  {
    std::lock_guard<std::mutex> lock(ctx->mtx);

    // Check if robot is already at target
    double rx = current_goal_.x, ry = current_goal_.y;
    try {
      auto t = ctx->tf_buffer->lookupTransform("map", "base_link",
        tf2::TimePointZero, tf2::durationFromSec(0.3));
      rx = t.transform.translation.x;
      ry = t.transform.translation.y;
    } catch (...) {}
    if (std::hypot(current_goal_.x - rx, current_goal_.y - ry) < 0.3) {
      RCLCPP_INFO(ctx->node->get_logger(), "NavigateToTarget: already at (%.2f,%.2f), skipping",
        current_goal_.x, current_goal_.y);
      return BT::NodeStatus::SUCCESS;
    }

    // Check if target is too close to any recently visited goal
    for (auto it = ctx->visited_goals.rbegin(); it != ctx->visited_goals.rend(); ++it) {
      if (std::hypot(current_goal_.x - it->x, current_goal_.y - it->y) < ctx->goal_min_separation) {
        RCLCPP_INFO(ctx->node->get_logger(), "NavigateToTarget: target near visited goal, skipping");
        return BT::NodeStatus::SUCCESS;
      }
    }
  }

  if (!ctx->nav_client->wait_for_action_server(5s)) {
    RCLCPP_WARN(ctx->node->get_logger(), "Nav2 action server not available");
    return BT::NodeStatus::FAILURE;
  }

  nav_done_ = false;
  nav_success_ = false;
  goal_sent_ = false;
  goal_rejected_ = false;
  retry_count_ = 0;

  sendGoal();

  RCLCPP_INFO(ctx->node->get_logger(),
    "NavigateToTarget: (%.2f, %.2f)", current_goal_.x, current_goal_.y);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToTarget::onRunning()
{
  auto ctx = getContext();
  if (!ctx) return BT::NodeStatus::FAILURE;

  // Handle goal rejection — retry with backoff
  if (goal_rejected_) {
    goal_rejected_ = false;
    goal_sent_ = false;
    retry_count_++;

    if (retry_count_ > kMaxRetries) {
      RCLCPP_WARN(ctx->node->get_logger(),
        "NavigateToTarget: goal rejected %d times, giving up", retry_count_);
      return BT::NodeStatus::FAILURE;
    }

    last_reject_time_ = ctx->node->now();
    RCLCPP_INFO(ctx->node->get_logger(),
      "NavigateToTarget: goal rejected (retry %d/%d), re-sending...",
      retry_count_, kMaxRetries);
    sendGoal();
    return BT::NodeStatus::RUNNING;
  }

  if (nav_done_) {
    {
      std::lock_guard<std::mutex> lock(ctx->mtx);
      ctx->visited_goals.push_back(current_goal_);

      geometry_msgs::msg::PointStamped ps;
      ps.header.frame_id = "map";
      ps.header.stamp = ctx->node->now();
      ps.point = current_goal_;
      ctx->mark_pub->publish(ps);
    }

    if (nav_success_) {
      RCLCPP_INFO(ctx->node->get_logger(), "NavigateToTarget: arrived");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(ctx->node->get_logger(), "NavigateToTarget: failed");
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void NavigateToTarget::onHalted()
{
  auto ctx = getContext();
  if (ctx) {
    ctx->nav_client->async_cancel_all_goals();
  }
}

// ── Action: SpinCollect ──

SpinCollect::SpinCollect(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config) {}

BT::PortsList SpinCollect::providedPorts()
{
  return {
    BT::InputPort<double>("spin_angle", 6.283, "Spin angle in rad"),
    BT::InputPort<double>("duration", 15.0, "Collection duration in seconds"),
  };
}

std::shared_ptr<MissionContext> SpinCollect::getContext()
{
  return ctxFromBB(this);
}

BT::NodeStatus SpinCollect::onStart()
{
  auto ctx = getContext();
  if (!ctx) return BT::NodeStatus::FAILURE;

  double spin_angle = 2.0 * M_PI;
  getInput("spin_angle", spin_angle);

  if (ctx->spin_client->wait_for_action_server(500ms)) {
    nav2_msgs::action::Spin::Goal goal;
    goal.target_yaw = spin_angle;
    goal.time_allowance = rclcpp::Duration::from_seconds(15.0);

    spin_done_ = false;
    spin_success_ = true;
    spin_start_ = ctx->node->now();

    auto opts = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
    opts.result_callback = [this](auto) {
      spin_done_ = true;
    };
    opts.goal_response_callback = [this](auto future) {
      if (!future.get()) {
        spin_done_ = true;
        spin_success_ = false;
      }
    };

    ctx->spin_client->async_send_goal(goal, opts);
    RCLCPP_INFO(ctx->node->get_logger(), "SpinCollect: spinning %.2f rad", spin_angle);
  } else {
    RCLCPP_DEBUG(ctx->node->get_logger(), "SpinCollect: spin server unavailable, waiting");
    spin_done_ = false;
    spin_start_ = ctx->node->now();
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SpinCollect::onRunning()
{
  auto ctx = getContext();
  if (!ctx) return BT::NodeStatus::FAILURE;

  double duration = 15.0;
  getInput("duration", duration);

  double elapsed = (ctx->node->now() - spin_start_).seconds();

  if (spin_done_ || elapsed >= duration) {
    RCLCPP_INFO(ctx->node->get_logger(),
      "SpinCollect: done (elapsed=%.1fs)", elapsed);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void SpinCollect::onHalted()
{
  auto ctx = getContext();
  if (ctx) {
    ctx->spin_client->async_cancel_all_goals();
  }
}

// ── Action: Explore ──

ExploreAction::ExploreAction(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config) {}

BT::PortsList ExploreAction::providedPorts()
{
  return {
    BT::InputPort<double>("explore_duration", 120.0, "Max exploration duration per step"),
  };
}

std::shared_ptr<MissionContext> ExploreAction::getContext()
{
  return ctxFromBB(this);
}

void ExploreAction::findFrontiers(const nav_msgs::msg::OccupancyGrid& map,
  std::vector<std::vector<std::pair<int,int>>>& clusters)
{
  int w = map.info.width;
  int h = map.info.height;

  const int dx[] = {1, -1, 0, 0};
  const int dy[] = {0, 0, 1, -1};

  std::set<int> frontier_set;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      int idx = y * w + x;
      if (map.data[idx] != 0) continue;  // only FREE cells
      for (int n = 0; n < 4; ++n) {
        int nx = x + dx[n], ny = y + dy[n];
        if (nx >= 0 && nx < w && ny >= 0 && ny < h && map.data[ny * w + nx] == -1) {
          frontier_set.insert(idx);
          break;
        }
      }
    }
  }

  // Cluster frontiers via BFS
  std::set<int> visited;
  for (int seed : frontier_set) {
    if (visited.count(seed)) continue;
    std::vector<std::pair<int,int>> cluster;
    std::queue<int> q;
    q.push(seed);
    visited.insert(seed);
    while (!q.empty()) {
      int idx = q.front(); q.pop();
      int x = idx % w, y = idx / w;
      cluster.push_back({x, y});
      for (int n = 0; n < 4; ++n) {
        int nx = x + dx[n], ny = y + dy[n];
        if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
          int nidx = ny * w + nx;
          if (frontier_set.count(nidx) && !visited.count(nidx)) {
            visited.insert(nidx);
            q.push(nidx);
          }
        }
      }
    }
    clusters.push_back(std::move(cluster));
  }
}

BT::NodeStatus ExploreAction::onStart()
{
  auto ctx = getContext();
  if (!ctx) return BT::NodeStatus::FAILURE;

  explore_done_ = false;
  explore_start_ = ctx->node->now();

  {
    std::lock_guard<std::mutex> lock(ctx->mtx);

    // Get robot position
    double rx = 0, ry = 0;
    try {
      auto t = ctx->tf_buffer->lookupTransform("map", "base_link",
        tf2::TimePointZero, tf2::durationFromSec(0.5));
      rx = t.transform.translation.x;
      ry = t.transform.translation.y;
    } catch (...) {}

    // Try frontier exploration first
    if (ctx->latest_map) {
      std::vector<std::vector<std::pair<int,int>>> clusters;
      findFrontiers(*ctx->latest_map, clusters);

      if (!clusters.empty()) {
        // Pick nearest frontier cluster
        int best_idx = -1;
        double best_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < clusters.size(); ++i) {
          double cx = 0, cy = 0;
          for (auto& c : clusters[i]) {
            cx += ctx->latest_map->info.origin.position.x
                + (c.first + 0.5) * ctx->latest_map->info.resolution;
            cy += ctx->latest_map->info.origin.position.y
                + (c.second + 0.5) * ctx->latest_map->info.resolution;
          }
          cx /= clusters[i].size();
          cy /= clusters[i].size();
          double d = std::hypot(cx - rx, cy - ry);

          if (d < 0.5) continue; // Ignore frontiers under the robot

          // Slight bias toward density direction
          if (ctx->latest_density) {
            int mx = static_cast<int>((cx - ctx->latest_density->info.origin.position.x)
                       / ctx->latest_density->info.resolution);
            int my = static_cast<int>((cy - ctx->latest_density->info.origin.position.y)
                       / ctx->latest_density->info.resolution);
            if (mx >= 0 && my >= 0 &&
                mx < static_cast<int>(ctx->latest_density->info.width) &&
                my < static_cast<int>(ctx->latest_density->info.height)) {
              int idx = my * ctx->latest_density->info.width + mx;
              double dens = ctx->latest_density->data[idx] / 100.0;
              d -= dens * 2.0;  // density bonus
            }
          }

          if (d < best_dist) {
            best_dist = d;
            best_idx = static_cast<int>(i);
          }
        }

        if (best_idx >= 0) {
          double cx = 0, cy = 0;
          for (auto& c : clusters[best_idx]) {
            cx += ctx->latest_map->info.origin.position.x
                + (c.first + 0.5) * ctx->latest_map->info.resolution;
            cy += ctx->latest_map->info.origin.position.y
                + (c.second + 0.5) * ctx->latest_map->info.resolution;
          }
          cx /= clusters[best_idx].size();
          cy /= clusters[best_idx].size();

          geometry_msgs::msg::PoseStamped target;
          target.header.frame_id = "map";
          target.pose.position.x = cx;
          target.pose.position.y = cy;
          target.pose.position.z = 0;
          target.pose.orientation.w = 1.0;
          config().blackboard->set("target_pose", target);

          RCLCPP_INFO(ctx->node->get_logger(),
            "Explore: frontier at (%.2f,%.2f) cluster_size=%zu",
            cx, cy, clusters[best_idx].size());
          return BT::NodeStatus::SUCCESS;
        }
      }
    }

    // No frontiers — set target to current position so NavigateToTarget
    // skips (within 0.3 m) and the BT loops back to re-check HasDetections.
    RCLCPP_INFO(ctx->node->get_logger(), "Explore: no frontiers, idling");

    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.frame_id = "map";
    current_pose.pose.position.x = rx;
    current_pose.pose.position.y = ry;
    current_pose.pose.position.z = 0;
    current_pose.pose.orientation.w = 1.0;
    config().blackboard->set("target_pose", current_pose);
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus ExploreAction::onRunning()
{
  auto ctx = getContext();
  if (!ctx) return BT::NodeStatus::FAILURE;

  if (explore_done_) {
    RCLCPP_INFO(ctx->node->get_logger(), "Explore: spin done");
    return BT::NodeStatus::SUCCESS;
  }

  double explore_duration = 120.0;
  getInput("explore_duration", explore_duration);
  if ((ctx->node->now() - explore_start_).seconds() > explore_duration) {
    RCLCPP_WARN(ctx->node->get_logger(), "Explore: timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void ExploreAction::onHalted()
{
  auto ctx = getContext();
  if (ctx) {
    ctx->spin_client->async_cancel_all_goals();
  }
}

// ── main ──

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
