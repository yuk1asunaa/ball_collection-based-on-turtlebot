#ifndef TURTLEBOT3_BALL_COLLECTION__MISSION_CONTROLLER_HPP_
#define TURTLEBOT3_BALL_COLLECTION__MISSION_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <mutex>
#include <vector>

struct MissionContext
{
  rclcpp::Node* node{nullptr};
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client;
  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr density_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr peaks_sub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr mark_pub;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_density;
  geometry_msgs::msg::PoseArray latest_peaks;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map;
  std::vector<geometry_msgs::msg::Point> visited_goals;
  rclcpp::Time last_detection_time;
  bool has_seen_detection{false};
  bool mission_complete{false};

  double density_weight{1.0};
  double distance_weight{0.5};
  double visit_penalty_weight{0.3};
  double visit_radius{1.0};
  double spin_angle{2.0 * M_PI};
  double collect_duration_sec{15.0};
  double explore_duration_sec{120.0};
  double mission_timeout_sec{60.0};
  double goal_min_separation{0.5};

  std::mutex mtx;
};

class MissionControllerNode : public rclcpp::Node
{
public:
  MissionControllerNode();

private:
  void density_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void peaks_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void tick_tree();

  std::shared_ptr<MissionContext> ctx_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr bt_timer_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  bool bt_initialized_{false};
};

// ── BT Condition Nodes ──

class WaitForMap : public BT::ConditionNode
{
public:
  WaitForMap(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  bool started_{false};
};

class WaitForNav2 : public BT::ConditionNode
{
public:
  WaitForNav2(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  bool started_{false};
  bool nav_exists_{false};
  bool spin_exists_{false};
  bool probe_sent_{false};
  bool probe_done_{false};
  bool spin_ready_{false};
  bool nav_probe_sent_{false};
  bool nav_probe_done_{false};
  bool nav_ready_{false};
};

class CheckBallsRemaining : public BT::ConditionNode
{
public:
  CheckBallsRemaining(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class HasDetections : public BT::ConditionNode
{
public:
  HasDetections(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// ── BT Action Nodes ──

class InitialSpin : public BT::StatefulActionNode
{
public:
  InitialSpin(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> getContext();
  bool spin_done_{false};
  bool spin_success_{false};
  rclcpp::Time spin_start_;
  double spin_angle_{2.0 * M_PI};
  int retry_count_{0};
  static constexpr int kMaxRetries = 5;
};

class SelectBestTarget : public BT::SyncActionNode
{
public:
  SelectBestTarget(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  double density_at_world(const geometry_msgs::msg::Point& p,
                          const nav_msgs::msg::OccupancyGrid& map) const;
  double visit_penalty(const geometry_msgs::msg::Point& p) const;
  bool is_far_enough(const geometry_msgs::msg::Point& p) const;
  std::shared_ptr<MissionContext> getContext() const;
};

class NavigateToTarget : public BT::StatefulActionNode
{
public:
  NavigateToTarget(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> getContext();
  void sendGoal();
  bool nav_done_{false};
  bool nav_success_{false};
  bool goal_sent_{false};
  bool goal_rejected_{false};
  int retry_count_{0};
  rclcpp::Time last_reject_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Point current_goal_;
  static constexpr int kMaxRetries = 5;
};

class SpinCollect : public BT::StatefulActionNode
{
public:
  SpinCollect(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> getContext();
  bool spin_done_{false};
  bool spin_success_{false};
  rclcpp::Time spin_start_;
};

class ExploreAction : public BT::StatefulActionNode
{
public:
  ExploreAction(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> getContext();
  void findFrontiers(const nav_msgs::msg::OccupancyGrid& map,
    std::vector<std::vector<std::pair<int,int>>>& clusters);
  bool explore_done_{false};
  rclcpp::Time explore_start_;
};

#endif
