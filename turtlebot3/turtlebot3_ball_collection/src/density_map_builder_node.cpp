#include "density_map_builder_node.hpp"
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

DensityMapBuilderNode::DensityMapBuilderNode() : Node("density_map_builder_node")
{
  target_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/vision/target_poses", 10,
    std::bind(&DensityMapBuilderNode::target_poses_callback, this, std::placeholders::_1));

  density_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("density_map", 10);
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(), std::bind(&DensityMapBuilderNode::map_callback, this, std::placeholders::_1));
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/density_markers", 10);
  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "/navigate_to_pose");
  spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(this, "/spin");

  this->declare_parameter<double>("resolution", 0.1);
  this->declare_parameter<int>("width", 100);
  this->declare_parameter<int>("height", 100);
  this->declare_parameter<double>("origin_x", -5.0);
  this->declare_parameter<double>("origin_y", -5.0);
  this->declare_parameter<int>("density_increment", 10);
  this->declare_parameter<int>("max_density", 100);
  this->declare_parameter<double>("gaussian_sigma", 0.5);
  this->declare_parameter<double>("time_decay_factor", 0.5);
  this->declare_parameter<bool>("navigate_on_peak", true);
  this->declare_parameter<double>("goal_republish_distance", 0.2);
  this->declare_parameter<std::string>("spin_action_name", "/spin");
  this->declare_parameter<bool>("enable_startup_spin", true);
  this->declare_parameter<double>("startup_spin_delay_sec", 1.0);
  this->declare_parameter<double>("lost_target_timeout_sec", 3.0);
  this->declare_parameter<double>("spin_angle_rad", 2.0 * M_PI);
  this->declare_parameter<double>("spin_cooldown_sec", 1.0);
  this->declare_parameter<bool>("enable_lost_target_spin", true);

  this->get_parameter("resolution", resolution_);
  this->get_parameter("width", width_);
  this->get_parameter("height", height_);
  this->get_parameter("origin_x", origin_x_);
  this->get_parameter("origin_y", origin_y_);
  this->get_parameter("density_increment", density_increment_);
  this->get_parameter("max_density", max_density_);
  this->get_parameter("gaussian_sigma", gaussian_sigma_);
  this->get_parameter("time_decay_factor", time_decay_factor_);
  this->get_parameter("navigate_on_peak", navigate_on_peak_);
  this->get_parameter("goal_republish_distance", goal_republish_distance_);
  this->get_parameter("spin_action_name", spin_action_name_);
  this->get_parameter("enable_startup_spin", enable_startup_spin_);
  this->get_parameter("startup_spin_delay_sec", startup_spin_delay_sec_);
  this->get_parameter("lost_target_timeout_sec", lost_target_timeout_sec_);
  this->get_parameter("spin_angle_rad", spin_angle_rad_);
  this->get_parameter("spin_cooldown_sec", spin_cooldown_sec_);
  this->get_parameter("enable_lost_target_spin", enable_lost_target_spin_);

  spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(this, spin_action_name_);

  pause_decay_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/pause_decay",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handle_pause_decay(request, response);
    });
  resume_decay_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/resume_decay",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handle_resume_decay(request, response);
    });

  lost_target_timer_ = this->create_wall_timer(
    200ms, std::bind(&DensityMapBuilderNode::monitor_lost_targets, this));
  startup_spin_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(startup_spin_delay_sec_)),
    [this]() { maybe_startup_spin(); });

  last_detection_time_ = this->now();
  has_seen_detection_ = false;
  last_spin_time_ = this->now() - rclcpp::Duration::from_seconds(spin_cooldown_sec_ + 1.0);

  // Initialize TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  density_grid_.header.frame_id = "map";
  density_grid_.info.resolution = resolution_;
  density_grid_.info.width = width_;
  density_grid_.info.height = height_;
  density_grid_.info.origin.position.x = origin_x_;
  density_grid_.info.origin.position.y = origin_y_;
  density_grid_.info.origin.position.z = 0.0;
  density_grid_.info.origin.orientation.w = 1.0;
  density_grid_.data.assign(width_ * height_, 0);

  RCLCPP_INFO(
    this->get_logger(),
    "Density map builder ready. startup_spin=%s, lost_target_timeout=%.2fs, spin_action=%s",
    enable_startup_spin_ ? "true" : "false", lost_target_timeout_sec_, spin_action_name_.c_str());
}

void DensityMapBuilderNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = msg;
}

void DensityMapBuilderNode::target_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (nav_in_progress_) {
    return;
  }

  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(msg->poses.size());
  for (const auto &pose : msg->poses) {
    points.push_back(pose.position);
  }

  if (points.empty()) {
    build_density_map(points);
    publish_density_map();
    send_peak_navigation_goal();
    return;
  }

  has_seen_detection_ = true;
  last_detection_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Received %zu balls from %s frame", points.size(), msg->header.frame_id.c_str());

  // Transform points to map frame using TF2
  std::vector<geometry_msgs::msg::Point> transformed_points;
  try {
    // Get transform from point cloud frame to map frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        "map", msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF2 transform error: %s", ex.what());
      return; 
    }

    // Transform each point to map frame
    for (const auto& p : points) {
      geometry_msgs::msg::PointStamped point_in, point_out;
      point_in.header.frame_id = msg->header.frame_id;
      point_in.header.stamp = msg->header.stamp;
      point_in.point = p;

      tf2::doTransform(point_in, point_out, transform_stamped);
      transformed_points.push_back(point_out.point);
    }
  } catch (std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error transforming points: %s", ex.what());
    transformed_points = points;
  }

  build_density_map(transformed_points);
  publish_density_map();
  send_peak_navigation_goal();
}

void DensityMapBuilderNode::build_density_map(const std::vector<geometry_msgs::msg::Point>& points)
{
  // Keep peaks persistent while tactical scan is running.
  if (!decay_paused_) {
    for (size_t i = 0; i < density_grid_.data.size(); ++i) {
      density_grid_.data[i] = static_cast<int>(density_grid_.data[i] * time_decay_factor_);
    }
  }

  // Apply Gaussian distribution for each point
  for (const auto& p : points) {
    // Calculate the radius for Gaussian spread (3 sigma rule)
    int spread_radius = static_cast<int>(gaussian_sigma_ / resolution_ * 3);
    
    // Calculate center cell
    int center_x = static_cast<int>((p.x - origin_x_) / resolution_);
    int center_y = static_cast<int>((p.y - origin_y_) / resolution_);
    
    // Spread density using 2D Gaussian distribution
    for (int dx = -spread_radius; dx <= spread_radius; ++dx) {
      for (int dy = -spread_radius; dy <= spread_radius; ++dy) {
        int x = center_x + dx;
        int y = center_y + dy;
        
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
          // Calculate distance from center
          double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
          
          // Calculate Gaussian weight
          double weight = std::exp(-(distance * distance) / (2 * gaussian_sigma_ * gaussian_sigma_));
          
          // Apply density increment with Gaussian weight
          int index = y * width_ + x;
          int new_density = static_cast<int>(density_grid_.data[index] + density_increment_ * weight);
          density_grid_.data[index] = std::min(max_density_, new_density);
        }
      }
    }
  }
}

void DensityMapBuilderNode::publish_density_map()
{
  density_grid_.header.stamp = this->now();
  density_map_pub_->publish(density_grid_);

  // Calculate and log density map statistics
  int max_density = 0;
  int high_density_cells = 0;
  int total_density = 0;
  for (int density : density_grid_.data) {
    if (density > max_density) {
      max_density = density;
    }
    if (density > 50) { // High density threshold
      high_density_cells++;
    }
    total_density += density;
  }
  double avg_density = density_grid_.data.empty() ? 0 : static_cast<double>(total_density) / density_grid_.data.size();

  RCLCPP_DEBUG(this->get_logger(), "Density map stats: max=%d, high_density_cells=%d, avg=%.2f",
               max_density, high_density_cells, avg_density);

  // Publish markers for visualization
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "density";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = marker.scale.y = resolution_;
  marker.scale.z = 0.01;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  
  for (int i = 0; i < width_; ++i) {
    for (int j = 0; j < height_; ++j) {
      int index = j * width_ + i;
      if (density_grid_.data[index] > 0) {
        geometry_msgs::msg::Point p;
        p.x = origin_x_ + i * resolution_;
        p.y = origin_y_ + j * resolution_;
        p.z = 0.0;
        marker.points.push_back(p);
        
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = density_grid_.data[index] / 100.0;
        marker.colors.push_back(color);
      }
    }
  }
  if (!marker.points.empty()) {
    markers.markers.push_back(marker);
  }
  marker_pub_->publish(markers);
}

void DensityMapBuilderNode::send_peak_navigation_goal()
{
  this->get_parameter("navigate_on_peak", navigate_on_peak_);
  if (!navigate_on_peak_) {
    return;
  }

  if (spin_in_progress_) {
    return;
  }

  int max_val = 0;
  auto max_iter = density_grid_.data.end();
  for (auto it = density_grid_.data.begin(); it != density_grid_.data.end(); ++it) {
    if (*it > max_val) {
      max_val = *it;
      max_iter = it;
    }
  }
  if (max_iter == density_grid_.data.end() || *max_iter <= 0) {
    return;
  }

  const int max_index = static_cast<int>(std::distance(density_grid_.data.begin(), max_iter));
  const int x = max_index % width_;
  const int y = max_index / width_;

  geometry_msgs::msg::Point goal_point;
  goal_point.x = origin_x_ + (static_cast<double>(x) + 0.5) * resolution_;
  goal_point.y = origin_y_ + (static_cast<double>(y) + 0.5) * resolution_;
  goal_point.z = 0.0;

  if (has_last_goal_ && nav_in_progress_) {
    const double dx = goal_point.x - last_goal_point_.x;
    const double dy = goal_point.y - last_goal_point_.y;
    if (std::hypot(dx, dy) < goal_republish_distance_) {
      return;
    }
  }

  if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "navigate_to_pose action server not available");
    return;
  }

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type()); // Use TimePointZero to avoid extrapolation errors
  goal.pose.pose.position = goal_point;
  goal.pose.pose.orientation.w = 1.0;

  nav_in_progress_ = true;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions
    options;
  options.result_callback =
    [this, goal_point](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
      // Only set to false if this was the latest goal, otherwise we overwrite the new goal's state
      if (std::hypot(last_goal_point_.x - goal_point.x, last_goal_point_.y - goal_point.y) < 0.01) {
          this->nav_in_progress_ = false;
      }
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        geometry_msgs::msg::TransformStamped transform;
        try {
          transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
          double rx = transform.transform.translation.x;
          double ry = transform.transform.translation.y;
          double dist = std::hypot(goal_point.x - rx, goal_point.y - ry);
          if (dist > 0.2) {
            RCLCPP_WARN(this->get_logger(), "Fake success detected (dist %.2f m). Clearing unreachable density peak.", dist);
            int center_x = static_cast<int>((goal_point.x - origin_x_) / resolution_);
            int center_y = static_cast<int>((goal_point.y - origin_y_) / resolution_);
            for (int dx = -15; dx <= 15; ++dx) {
              for (int dy = -15; dy <= 15; ++dy) {
                int nx = center_x + dx;
                int ny = center_y + dy;
                if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                  density_grid_.data[ny * width_ + nx] = 0;
                }
              }
            }
          }
        } catch (tf2::TransformException &ex) {
          RCLCPP_DEBUG(this->get_logger(), "TF Error: %s", ex.what());
        }
      }
    };
  nav_client_->async_send_goal(goal, options);
  last_goal_point_ = goal_point;
  has_last_goal_ = true;

  RCLCPP_INFO(this->get_logger(), "Sent navigation goal to density peak (%.2f, %.2f)",
    goal_point.x, goal_point.y);
}

void DensityMapBuilderNode::trigger_spin(const std::string & reason)
{
  if (spin_in_progress_) {
    return;
  }

  const rclcpp::Time now = this->now();
  if ((now - last_spin_time_).seconds() < spin_cooldown_sec_) {
    return;
  }

  if (!spin_client_->wait_for_action_server(500ms)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Spin action server '%s' not available", spin_action_name_.c_str());
    return;
  }

  set_decay_paused(true, reason.c_str());
  spin_in_progress_ = true;
  last_spin_time_ = now;

  nav2_msgs::action::Spin::Goal goal;
  goal.target_yaw = spin_angle_rad_;
  goal.time_allowance = rclcpp::Duration::from_seconds(30.0);

  rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions options;
  options.goal_response_callback =
    std::bind(&DensityMapBuilderNode::on_spin_goal_response, this, std::placeholders::_1);
  options.result_callback =
    std::bind(&DensityMapBuilderNode::on_spin_result, this, std::placeholders::_1);

  spin_client_->async_send_goal(goal, options);
  RCLCPP_INFO(this->get_logger(), "Triggered spin (reason=%s, angle=%.2f rad)", reason.c_str(), spin_angle_rad_);
}

void DensityMapBuilderNode::on_spin_goal_response(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    spin_in_progress_ = false;
    set_decay_paused(false, "spin_goal_rejected");
    RCLCPP_WARN(this->get_logger(), "Spin goal was rejected by Nav2. Retrying in 5 seconds...");
    if (enable_startup_spin_) {
      startup_spin_triggered_ = false;
      startup_spin_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(5.0)),
        [this]() { maybe_startup_spin(); });
    }
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Spin goal accepted");
}

void DensityMapBuilderNode::on_spin_result(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult & result)
{
  spin_in_progress_ = false;
  // Reset lost-target watchdog after each scan attempt to avoid immediate retrigger loops.
  last_detection_time_ = this->now();
  set_decay_paused(false, "spin_result");

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Spin completed successfully");
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_WARN(this->get_logger(), "Spin aborted");
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "Spin canceled");
  } else {
    RCLCPP_WARN(this->get_logger(), "Spin ended with unknown result code");
  }

  int max_val = 0;
  auto max_iter = density_grid_.data.end();
  for (auto it = density_grid_.data.begin(); it != density_grid_.data.end(); ++it) {
    if (*it > max_val) {
      max_val = *it;
      max_iter = it;
    }
  }
  if (max_iter == density_grid_.data.end() || *max_iter <= 0) {
    static int s_idx = 0;
    std::vector<std::pair<double, double>> s_path;
    double min_x = origin_x_ + 1.0;
    double max_x = origin_x_ + (width_ * resolution_) - 1.0;
    double min_y = origin_y_ + 1.0;
    double max_y = origin_y_ + (height_ * resolution_) - 1.0;
    int num_lines = 5;
    double y_step = (max_y - min_y) / (num_lines - 1);
    for (int i = 0; i < num_lines; i++) {
        double y = min_y + i * y_step;
        if (i % 2 == 0) {
            s_path.push_back({min_x, y});
            s_path.push_back({max_x, y});
        } else {
            s_path.push_back({max_x, y});
            s_path.push_back({min_x, y});
        }
    }
    
    geometry_msgs::msg::Point random_goal;
    random_goal.x = s_path[s_idx % s_path.size()].first;
    random_goal.y = s_path[s_idx % s_path.size()].second;
    random_goal.z = 0.0;
    s_idx++;
    
    nav2_msgs::action::NavigateToPose::Goal nav_goal;
    nav_goal.pose.header.frame_id = "map";
    nav_goal.pose.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    nav_goal.pose.pose.position = random_goal;
    nav_goal.pose.pose.orientation.w = 1.0;
    
    if (nav_client_) {
      nav_in_progress_ = true;
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions
        nav_options;
      nav_options.result_callback =
        [this, random_goal](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
          if (std::hypot(last_goal_point_.x - random_goal.x, last_goal_point_.y - random_goal.y) < 0.01) {
              this->nav_in_progress_ = false;
          }
        };
      nav_client_->async_send_goal(nav_goal, nav_options);
      last_goal_point_ = random_goal;
      has_last_goal_ = true;
      RCLCPP_INFO(this->get_logger(), "Spin found no targets. Exploring random point (%.2f, %.2f)", random_goal.x, random_goal.y);
    }
  } else {
    // Immediately transition from scan phase back to peak-driven navigation.
    send_peak_navigation_goal();
  }
}

void DensityMapBuilderNode::monitor_lost_targets()
{
  if (spin_in_progress_ || nav_in_progress_) {
    return;
  }

  if (!enable_lost_target_spin_) {
    return;
  }

  const double seconds_without_target = (this->now() - last_detection_time_).seconds();
  if (seconds_without_target >= lost_target_timeout_sec_) {
    trigger_spin("lost_target");
    // Prevent immediate retrigger while still no detections.
    last_detection_time_ = this->now();
  }
}

void DensityMapBuilderNode::maybe_startup_spin()
{
  if (startup_spin_triggered_) {
    return;
  }

  if (enable_startup_spin_) {
    if (!spin_client_->action_server_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Waiting for spin action server '%s' to become available...", spin_action_name_.c_str());
      return;
    }
    
    startup_spin_triggered_ = true;
    startup_spin_timer_->cancel();
    trigger_spin("startup");
  } else {
    startup_spin_triggered_ = true;
    startup_spin_timer_->cancel();
  }
}

bool DensityMapBuilderNode::set_decay_paused(bool paused, const char * source)
{
  if (decay_paused_ == paused) {
    return false;
  }

  decay_paused_ = paused;
  RCLCPP_INFO(
    this->get_logger(),
    "Decay %s by %s",
    decay_paused_ ? "paused" : "resumed",
    source == nullptr ? "unknown" : source);
  return true;
}

void DensityMapBuilderNode::handle_pause_decay(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  set_decay_paused(true, "service_pause_decay");
  response->success = true;
  response->message = "Decay paused";
}

void DensityMapBuilderNode::handle_resume_decay(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  set_decay_paused(false, "service_resume_decay");
  response->success = true;
  response->message = "Decay resumed";
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DensityMapBuilderNode>());
  rclcpp::shutdown();
  return 0;
}