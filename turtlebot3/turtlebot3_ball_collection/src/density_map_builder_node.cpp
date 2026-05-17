#include "density_map_builder_node.hpp"
#include <cmath>
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

DensityMapBuilderNode::DensityMapBuilderNode() : Node("density_map_builder_node")
{
  target_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/vision/target_poses", 10,
    std::bind(&DensityMapBuilderNode::target_poses_callback, this, std::placeholders::_1));

  ball_collected_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/ball_collected", 10,
    std::bind(&DensityMapBuilderNode::ball_collected_callback, this, std::placeholders::_1));

  density_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "density_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
    std::bind(&DensityMapBuilderNode::map_callback, this, std::placeholders::_1));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/visualization/density_markers", 10);

  peaks_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/density/peaks", rclcpp::QoS(5));

  this->declare_parameter<double>("resolution", 0.1);
  this->declare_parameter<int>("width", 100);
  this->declare_parameter<int>("height", 100);
  this->declare_parameter<double>("origin_x", -5.0);
  this->declare_parameter<double>("origin_y", -5.0);
  this->declare_parameter<int>("density_increment", 10);
  this->declare_parameter<int>("max_density", 100);
  this->declare_parameter<double>("gaussian_sigma", 0.5);
  this->declare_parameter<double>("time_decay_factor", 0.99);
  this->declare_parameter<int>("peak_count", 5);
  this->declare_parameter<double>("decay_period_sec", 1.0);

  this->get_parameter("resolution", resolution_);
  this->get_parameter("width", width_);
  this->get_parameter("height", height_);
  this->get_parameter("origin_x", origin_x_);
  this->get_parameter("origin_y", origin_y_);
  this->get_parameter("density_increment", density_increment_);
  this->get_parameter("max_density", max_density_);
  this->get_parameter("gaussian_sigma", gaussian_sigma_);
  this->get_parameter("time_decay_factor", time_decay_factor_);
  this->get_parameter("peak_count", peak_count_);
  this->get_parameter("decay_period_sec", decay_period_sec_);

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

  // Periodic decay timer
  decay_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(decay_period_sec_)),
    [this]() {
      apply_decay();
      publish_density_map();
    });

  RCLCPP_INFO(this->get_logger(),
    "DensityMapBuilder ready (perception-only): grid=%dx%d, sigma=%.2f, decay=%.3f",
    width_, height_, gaussian_sigma_, time_decay_factor_);
}

void DensityMapBuilderNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = msg;
}

void DensityMapBuilderNode::ball_collected_callback(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  remove_gaussian_at(msg->point);
  publish_density_map();
  RCLCPP_DEBUG(this->get_logger(), "Removed Gaussian at (%.2f, %.2f)",
    msg->point.x, msg->point.y);
}

void DensityMapBuilderNode::target_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(msg->poses.size());
  for (const auto &pose : msg->poses) {
    points.push_back(pose.position);
  }

  if (points.empty()) {
    // Still apply decay and republish even when no new detections
    return;
  }

  // Transform points to map frame
  std::vector<geometry_msgs::msg::Point> transformed_points;
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        "map", msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF2 transform error: %s", ex.what());
      return;
    }

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
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Received %zu balls from %s frame",
    transformed_points.size(), msg->header.frame_id.c_str());

  build_density_map(transformed_points);
  publish_density_map();
}

void DensityMapBuilderNode::apply_decay()
{
  for (auto & cell : density_grid_.data) {
    cell = static_cast<int>(cell * time_decay_factor_);
  }
}

void DensityMapBuilderNode::build_density_map(const std::vector<geometry_msgs::msg::Point>& points)
{
  const int spread_radius = static_cast<int>(gaussian_sigma_ / resolution_ * 3);

  for (const auto& p : points) {
    int center_x = static_cast<int>((p.x - origin_x_) / resolution_);
    int center_y = static_cast<int>((p.y - origin_y_) / resolution_);

    for (int dx = -spread_radius; dx <= spread_radius; ++dx) {
      for (int dy = -spread_radius; dy <= spread_radius; ++dy) {
        int x = center_x + dx;
        int y = center_y + dy;

        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
          double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
          double weight = std::exp(-(distance * distance) / (2 * gaussian_sigma_ * gaussian_sigma_));
          int index = y * width_ + x;
          int new_density = static_cast<int>(density_grid_.data[index] + density_increment_ * weight);
          density_grid_.data[index] = std::min(max_density_, new_density);
        }
      }
    }
  }
}

void DensityMapBuilderNode::remove_gaussian_at(const geometry_msgs::msg::Point & p)
{
  int spread_radius = static_cast<int>(gaussian_sigma_ / resolution_ * 3);
  int center_x = static_cast<int>((p.x - origin_x_) / resolution_);
  int center_y = static_cast<int>((p.y - origin_y_) / resolution_);

  for (int dx = -spread_radius; dx <= spread_radius; ++dx) {
    for (int dy = -spread_radius; dy <= spread_radius; ++dy) {
      int x = center_x + dx;
      int y = center_y + dy;
      if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
        double weight = std::exp(-(distance * distance) / (2 * gaussian_sigma_ * gaussian_sigma_));
        int index = y * width_ + x;
        int dec = static_cast<int>(density_increment_ * weight + 0.5);
        density_grid_.data[index] = std::max(0, density_grid_.data[index] - dec);
      }
    }
  }
}

void DensityMapBuilderNode::publish_density_map()
{
  density_grid_.header.stamp = this->now();
  density_map_pub_->publish(density_grid_);

  // RViz markers
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

  for (int j = 0; j < height_; ++j) {
    for (int i = 0; i < width_; ++i) {
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

  // Top N peaks
  geometry_msgs::msg::PoseArray peaks;
  peaks.header.frame_id = "map";
  peaks.header.stamp = this->now();

  struct Peak { int value; int index; };
  std::vector<Peak> top;
  for (int i = 0; i < static_cast<int>(density_grid_.data.size()); ++i) {
    int v = density_grid_.data[i];
    if (v <= 0) continue;
    top.push_back({v, i});
  }
  std::sort(top.begin(), top.end(),
    [](const Peak &a, const Peak &b) { return a.value > b.value; });

  int cnt = 0;
  for (auto &kv : top) {
    if (cnt++ >= peak_count_) break;
    int ix = kv.index % width_;
    int iy = kv.index / width_;
    geometry_msgs::msg::Pose p;
    p.position.x = origin_x_ + (static_cast<double>(ix) + 0.5) * resolution_;
    p.position.y = origin_y_ + (static_cast<double>(iy) + 0.5) * resolution_;
    p.position.z = 0.0;
    p.orientation.w = 1.0;
    peaks.poses.push_back(p);
  }
  if (!peaks.poses.empty()) {
    peaks_pub_->publish(peaks);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DensityMapBuilderNode>());
  rclcpp::shutdown();
  return 0;
}
