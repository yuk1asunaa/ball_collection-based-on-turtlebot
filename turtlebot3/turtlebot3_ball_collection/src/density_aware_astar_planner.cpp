#include "density_aware_astar_planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>



#include "pluginlib/class_list_macros.hpp"



namespace turtlebot3_ball_collection

{
  


void DensityAwareAStarPlanner::configure(

  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,

  std::string name,

  std::shared_ptr<tf2_ros::Buffer> tf,

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)

{

  node_ = parent.lock();

  if (!node_) {

    throw std::runtime_error("Failed to lock lifecycle node in planner configure");

  }



  name_ = std::move(name);

  logger_ = node_->get_logger();

  tf_ = std::move(tf);

  costmap_ros_ = std::move(costmap_ros);

  costmap_ = costmap_ros_->getCostmap();

  global_frame_ = costmap_ros_->getGlobalFrameID();



  node_->declare_parameter(name_ + ".obstacle_weight", obstacle_weight_);

  node_->declare_parameter(name_ + ".density_weight", density_weight_);

  node_->declare_parameter(name_ + ".min_step_cost", min_step_cost_);

  node_->declare_parameter(name_ + ".unknown_cost_penalty", unknown_cost_penalty_);

  node_->declare_parameter(name_ + ".density_topic", density_topic_);



  node_->get_parameter(name_ + ".obstacle_weight", obstacle_weight_);

  node_->get_parameter(name_ + ".density_weight", density_weight_);

  node_->get_parameter(name_ + ".min_step_cost", min_step_cost_);

  node_->get_parameter(name_ + ".unknown_cost_penalty", unknown_cost_penalty_);

  node_->get_parameter(name_ + ".density_topic", density_topic_);



  neighbors_ = {

    {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},

    {1, 1, std::sqrt(2.0)}, {1, -1, std::sqrt(2.0)}, {-1, 1, std::sqrt(2.0)}, {-1, -1, std::sqrt(2.0)}

  };



  density_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(

    density_topic_,

    rclcpp::QoS(10),

    std::bind(&DensityAwareAStarPlanner::densityCallback, this, std::placeholders::_1));



  RCLCPP_INFO(

    logger_,

    "Configured DensityAwareAStarPlanner: topic=%s obstacle_weight=%.3f density_weight=%.3f",

    density_topic_.c_str(), obstacle_weight_, density_weight_);

}



void DensityAwareAStarPlanner::cleanup()

{

  std::scoped_lock<std::mutex> lock(density_mutex_);

  latest_density_map_.reset();

  density_sub_.reset();

}



void DensityAwareAStarPlanner::activate()

{

}



void DensityAwareAStarPlanner::deactivate()

{

}



void DensityAwareAStarPlanner::densityCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)

{

  std::scoped_lock<std::mutex> lock(density_mutex_);

  latest_density_map_ = msg;

}



double DensityAwareAStarPlanner::getDensityValueAtMapCell(unsigned int mx, unsigned int my) const

{

  std::scoped_lock<std::mutex> lock(density_mutex_);

  if (!latest_density_map_) {

    return 0.0;

  }



  const auto & map = *latest_density_map_;

  const double wx = costmap_->getOriginX() + (static_cast<double>(mx) + 0.5) * costmap_->getResolution();

  const double wy = costmap_->getOriginY() + (static_cast<double>(my) + 0.5) * costmap_->getResolution();



  const double rx = (wx - map.info.origin.position.x) / map.info.resolution;

  const double ry = (wy - map.info.origin.position.y) / map.info.resolution;



  if (rx < 0.0 || ry < 0.0) {

    return 0.0;

  }



  const auto dx = static_cast<unsigned int>(rx);

  const auto dy = static_cast<unsigned int>(ry);

  if (dx >= map.info.width || dy >= map.info.height) {

    return 0.0;

  }



  const auto idx = static_cast<size_t>(dy) * map.info.width + dx;

  const int raw = map.data[idx];

  if (raw < 0) {

    return 0.0;

  }



  return std::clamp(static_cast<double>(raw) / 100.0, 0.0, 1.0);

}



double DensityAwareAStarPlanner::heuristic(unsigned int mx, unsigned int my, unsigned int gx, unsigned int gy) const

{

  const double dx = static_cast<double>(gx) - static_cast<double>(mx);

  const double dy = static_cast<double>(gy) - static_cast<double>(my);

  return std::sqrt(dx * dx + dy * dy) * min_step_cost_;

}



nav_msgs::msg::Path DensityAwareAStarPlanner::reconstructPath(

  const std::vector<int> & parent,

  unsigned int start_index,

  unsigned int goal_index,

  const std::string & frame_id) const

{

  nav_msgs::msg::Path path;

  path.header.stamp = node_->now();

  path.header.frame_id = frame_id;



  std::vector<unsigned int> indices;

  unsigned int current = goal_index;

  indices.push_back(current);



  while (current != start_index) {

    const int p = parent[current];

    if (p < 0) {

      break;

    }

    current = static_cast<unsigned int>(p);

    indices.push_back(current);

  }



  std::reverse(indices.begin(), indices.end());

  path.poses.reserve(indices.size());



  for (const unsigned int idx : indices) {

    const unsigned int mx = idx % costmap_->getSizeInCellsX();

    const unsigned int my = idx / costmap_->getSizeInCellsX();



    double wx = 0.0;

    double wy = 0.0;

    costmap_->mapToWorld(mx, my, wx, wy);



    geometry_msgs::msg::PoseStamped pose;

    pose.header = path.header;

    pose.pose.position.x = wx;

    pose.pose.position.y = wy;

    pose.pose.position.z = 0.0;

    pose.pose.orientation.w = 1.0;

    path.poses.push_back(pose);

  }

  return path;

}



nav_msgs::msg::Path DensityAwareAStarPlanner::createPlan(

  const geometry_msgs::msg::PoseStamped & start,

  const geometry_msgs::msg::PoseStamped & goal)

{

  nav_msgs::msg::Path empty_path;

  empty_path.header.stamp = node_->now();

  empty_path.header.frame_id = global_frame_;



  unsigned int start_mx = 0;

  unsigned int start_my = 0;

  unsigned int goal_mx = 0;

  unsigned int goal_my = 0;

  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {

    RCLCPP_WARN(logger_, "Start pose out of costmap bounds");

    return empty_path;

  }

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {

    RCLCPP_WARN(logger_, "Goal pose out of costmap bounds");

    return empty_path;

  }



  const unsigned int width = costmap_->getSizeInCellsX();

  const unsigned int height = costmap_->getSizeInCellsY();

  const size_t cell_count = static_cast<size_t>(width) * height;



  const unsigned int start_idx = start_my * width + start_mx;

  const unsigned int goal_idx = goal_my * width + goal_mx;



  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());

  std::vector<int> parent(cell_count, -1);

  std::vector<uint8_t> closed(cell_count, 0);

  std::priority_queue<GridNode, std::vector<GridNode>, GridNodeCompare> open_set;



  g_score[start_idx] = 0.0;

  open_set.push({start_idx, heuristic(start_mx, start_my, goal_mx, goal_my)});



  while (!open_set.empty()) {

    const GridNode current_node = open_set.top();

    open_set.pop();



    const unsigned int current = current_node.index;

    if (closed[current] != 0U) {

      continue;

    }

    closed[current] = 1;



    const unsigned int cx = current % width;

    const unsigned int cy = current / width;



    const double dist_to_goal_m = std::hypot(

      (static_cast<double>(cx) - static_cast<double>(goal_mx)) * costmap_->getResolution(),

      (static_cast<double>(cy) - static_cast<double>(goal_my)) * costmap_->getResolution());



    if (current == goal_idx || dist_to_goal_m <= 0.35) {

      return reconstructPath(parent, start_idx, current, global_frame_);

    }



    for (const auto & n : neighbors_) {

      const int nx = static_cast<int>(cx) + n.dx;

      const int ny = static_cast<int>(cy) + n.dy;

      if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {

        continue;

      }



      const unsigned int ux = static_cast<unsigned int>(nx);

      const unsigned int uy = static_cast<unsigned int>(ny);

      const unsigned int neighbor = uy * width + ux;



      if (closed[neighbor] != 0U) {

        continue;

      }



      const unsigned char raw_cost = costmap_->getCost(ux, uy);

      if (raw_cost != nav2_costmap_2d::NO_INFORMATION &&

          raw_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {

        continue;

      }



      double normalized_obstacle_cost = 0.0;

      if (raw_cost == nav2_costmap_2d::NO_INFORMATION) {

        normalized_obstacle_cost = unknown_cost_penalty_;

      } else {

        normalized_obstacle_cost = static_cast<double>(raw_cost) / 252.0;

      }



      const double density_reward = getDensityValueAtMapCell(ux, uy);

      double step_cost = n.base_distance

        + obstacle_weight_ * normalized_obstacle_cost

        - density_weight_ * density_reward;



      // Keep edge costs positive; A* assumptions break with negative edges.

      step_cost = std::max(step_cost, min_step_cost_ * n.base_distance);



      const double tentative_g = g_score[current] + step_cost;

      if (tentative_g < g_score[neighbor]) {

        g_score[neighbor] = tentative_g;

        parent[neighbor] = static_cast<int>(current);

        const double f = tentative_g + heuristic(ux, uy, goal_mx, goal_my);

        open_set.push({neighbor, f});

      }

    }

  }



  RCLCPP_WARN(logger_, "DensityAwareAStarPlanner failed to find path");

  return empty_path;

}



}



PLUGINLIB_EXPORT_CLASS(

  turtlebot3_ball_collection::DensityAwareAStarPlanner,

  nav2_core::GlobalPlanner)

