#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), last_x(0.0), last_y(0.0), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
              "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    // Initialize publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Initialize timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
    

    nav_msgs::msg::MapMetaData metadata;
    metadata.resolution = 1.0 / scale_factor;
    metadata.width = width * scale_factor;
    metadata.height = height * scale_factor;
    metadata.origin.position.x = -20;
    metadata.origin.position.y = -20;
    metadata.origin.orientation.w = 1.0;
    
    global_map_.info = metadata;
    global_map_.data = std::vector<int8_t>(width * scale_factor * height * scale_factor, 0);
}

// Callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Store the latest costmap
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

 // Callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    yaw = quaternionToYaw(msg->pose.pose.orientation);
    // Compute distance traveled
    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (distance >= distance_threshold) {
        // RCLCPP_INFO(this->get_logger(), "Current position: x = %f, y = %f", x, y);
        // RCLCPP_INFO(this->get_logger(), "Previous position: last_x = %f, last_y = %f", last_x, last_y);
        last_x = x;
        last_y = y;
        should_update_map_ = true;
    }
}

// Timer-based map update
void MapMemoryNode::updateMap() {
    if (should_update_map_ && costmap_updated_) {
        MapMemoryNode::integrateCostmap();
        map_pub_->publish(global_map_);
        RCLCPP_INFO(this->get_logger(), "Updated map memory");
        should_update_map_ = false;
    }
}

double MapMemoryNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
  // Euler angles from quaternion (yaw-pitch-roll sequence)
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  return yaw;
}

// Integrate the latest costmap into the global map
void MapMemoryNode::integrateCostmap() {
  double costmap_origin_x_in_global = x + latest_costmap_.info.origin.position.x;
  double costmap_origin_y_in_global = y + latest_costmap_.info.origin.position.y;

  // Define the center of rotation for the latest costmap.
  double costmap_center_x = width * scale_factor / 2.0;
  double costmap_center_y = height * scale_factor / 2.0;

  // Iterate through the latest costmap and integrate the rotated data into the global map.
  for (unsigned int costmap_y = 0; costmap_y < latest_costmap_.info.height; ++costmap_y) {
      for (unsigned int costmap_x = 0; costmap_x < latest_costmap_.info.width; ++costmap_x) {
          // Calculate the cell coordinates relative to the costmap center (before rotation).
          double cell_x_rel = (costmap_x * latest_costmap_.info.resolution) - costmap_center_x * latest_costmap_.info.resolution;
          double cell_y_rel = (costmap_y * latest_costmap_.info.resolution) - costmap_center_y * latest_costmap_.info.resolution;

          // Rotate the cell coordinates around the center.
          double rotated_x_rel = cos(yaw) * cell_x_rel - sin(yaw) * cell_y_rel;
          double rotated_y_rel = sin(yaw) * cell_x_rel + cos(yaw) * cell_y_rel;

          // Calculate the global coordinates of the rotated cell.
          double rotated_x_global_meters = rotated_x_rel + costmap_origin_x_in_global + costmap_center_x* latest_costmap_.info.resolution; //add back center
          double rotated_y_global_meters = rotated_y_rel + costmap_origin_y_in_global + costmap_center_y* latest_costmap_.info.resolution; //add back center

          // Convert the global coordinates from meters to grid coordinates.
          int global_x = (int)((rotated_x_global_meters - global_map_.info.origin.position.x) / global_map_.info.resolution);
          int global_y = (int)((rotated_y_global_meters - global_map_.info.origin.position.y) / global_map_.info.resolution);

          // Check if the global coordinates are within the bounds of the global map.
          if (global_x >= 0 && global_x < global_map_.info.width &&
              global_y >= 0 && global_y < global_map_.info.height) {
              // Only update if greater than 0
              if (latest_costmap_.data[costmap_y * latest_costmap_.info.width + costmap_x] > 0) {
                  global_map_.data[global_y * global_map_.info.width + global_x] =
                      latest_costmap_.data[costmap_y * latest_costmap_.info.width + costmap_x];
              }
          }
      }
  }
  global_map_.header = latest_costmap_.header;
  global_map_.header.frame_id = "sim_world"; 
  costmap_updated_ = false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
