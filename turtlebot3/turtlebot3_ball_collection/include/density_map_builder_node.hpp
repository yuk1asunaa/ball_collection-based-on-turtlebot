#ifndef TURTLEBOT3_BALL_COLLECTION__DENSITY_MAP_BUILDER_NODE_HPP_
#define TURTLEBOT3_BALL_COLLECTION__DENSITY_MAP_BUILDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class DensityMapBuilderNode : public rclcpp::Node
{
public:
  DensityMapBuilderNode();

private:
  void target_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void ball_collected_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void build_density_map(const std::vector<geometry_msgs::msg::Point>& points);
  void publish_density_map();
  void apply_decay();
  void remove_gaussian_at(const geometry_msgs::msg::Point & p);

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_poses_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ball_collected_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr density_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr peaks_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr known_map_pub_;
  rclcpp::TimerBase::SharedPtr decay_timer_;

  nav_msgs::msg::OccupancyGrid density_grid_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  double resolution_;
  int width_, height_;
  double origin_x_, origin_y_;
  int density_increment_;
  int max_density_;
  double gaussian_sigma_;
  double time_decay_factor_;
  int peak_count_;
  double decay_period_sec_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif
