#ifndef TURTLEBOT3_BALL_COLLECTION__DENSITY_AWARE_ASTAR_PLANNER_HPP_
#define TURTLEBOT3_BALL_COLLECTION__DENSITY_AWARE_ASTAR_PLANNER_HPP_

#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace turtlebot3_ball_collection
{

class DensityAwareAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  DensityAwareAStarPlanner() = default;
  ~DensityAwareAStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct GridNode
  {
    unsigned int index{0};
    double f{0.0};
  };

  struct GridNodeCompare
  {
    bool operator()(const GridNode & lhs, const GridNode & rhs) const
    {
      return lhs.f > rhs.f;
    }
  };

  struct Neighbor
  {
    int dx;
    int dy;
    double base_distance;
  };

  void densityCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  double getDensityValueAtMapCell(unsigned int mx, unsigned int my) const;
  double heuristic(unsigned int mx, unsigned int my, unsigned int gx, unsigned int gy) const;
  nav_msgs::msg::Path reconstructPath(
    const std::vector<int> & parent,
    unsigned int start_index,
    unsigned int goal_index,
    const std::string & frame_id) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  rclcpp::Logger logger_{rclcpp::get_logger("DensityAwareAStarPlanner")};
  std::string name_;
  std::string global_frame_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr density_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_density_map_;
  mutable std::mutex density_mutex_;

  std::vector<Neighbor> neighbors_;

  double obstacle_weight_{1.0};
  double density_weight_{0.6};
  double min_step_cost_{0.01};
  double unknown_cost_penalty_{1.0};
  std::string density_topic_{"/density_map"};
};

}  // namespace turtlebot3_ball_collection

#endif  // TURTLEBOT3_BALL_COLLECTION__DENSITY_AWARE_ASTAR_PLANNER_HPP_
