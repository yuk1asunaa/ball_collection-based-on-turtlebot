#ifndef TURTLEBOT3_BALL_COLLECTION__DENSITY_MAP_BUILDER_NODE_HPP_
#define TURTLEBOT3_BALL_COLLECTION__DENSITY_MAP_BUILDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class DensityMapBuilderNode : public rclcpp::Node
{
public:
  DensityMapBuilderNode();

private:
  void target_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void build_density_map(const std::vector<geometry_msgs::msg::Point>& points);
  void publish_density_map();
  void send_peak_navigation_goal();
  void trigger_spin(const std::string & reason);
  void on_spin_goal_response(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr & goal_handle);
  void on_spin_result(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult & result);
  void monitor_lost_targets();
  void maybe_startup_spin();
  bool set_decay_paused(bool paused, const char * source);
  void handle_pause_decay(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_resume_decay(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_poses_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr density_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_decay_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_decay_srv_;
  rclcpp::TimerBase::SharedPtr lost_target_timer_;
  rclcpp::TimerBase::SharedPtr startup_spin_timer_;

  nav_msgs::msg::OccupancyGrid density_grid_;
  double resolution_;
  int width_, height_;
  double origin_x_, origin_y_;
  int density_increment_;
  int max_density_;
  double gaussian_sigma_;
  double time_decay_factor_;
  bool decay_paused_{false};
  bool navigate_on_peak_;
  double goal_republish_distance_;
  std::string spin_action_name_;
  bool enable_startup_spin_;
  double startup_spin_delay_sec_;
  double lost_target_timeout_sec_;
  double spin_angle_rad_;
  double spin_cooldown_sec_;
  bool enable_lost_target_spin_{false};
  bool startup_spin_triggered_{false};
  bool spin_in_progress_{false};
  bool nav_in_progress_{false};
  bool has_seen_detection_{false};
  rclcpp::Time last_detection_time_;
  rclcpp::Time last_spin_time_;
  geometry_msgs::msg::Point last_goal_point_;
  bool has_last_goal_{false};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif