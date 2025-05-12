#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <algorithm>
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void mark_free_space(int x0, int y0, int x1, int y1, std::vector<std::vector<int>>& grid) {
  if (x0 == x1) { // vertical line
        int start_y = std::min(y0, y1);
        int end_y = std::max(y0, y1);
        for (int y = start_y; y <= end_y; ++y) {
            grid[y][x0] = 0;
        }
        return;
    }
  double m = (double)(y1 - y0) / (x1 - x0);

  if(x0 < x1) {
    for (size_t i = 0; i < x1-x0; i++) { 
        int y = round(m*i + y0);
        int x = x0 + i;

        grid[y][x] = 0;
    } 
  }
  else {
    for (size_t i = 0; i > x1-x0; i--) { 
        int y = round(m*i + y0);
        int x = x0 + i;

        grid[y][x] = 0;
    }
  }
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    const int scale_factor = 10;
    const int width = 40, height = 40;
    const int max_cost = 100;
    std::vector<std::vector<int>> occupancy_grid(height * scale_factor, std::vector<int>(width * scale_factor, 0));
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            x_grid = (int)(range * cos(angle) * scale_factor + width * scale_factor / 2.0);
            y_grid = (int)(range * sin(angle) * scale_factor + height * scale_factor / 2.0);

            if(x_grid >= 0 && x_grid < width*scale_factor && y_grid >= 0 && y_grid < height*scale_factor){ 
              
              // Mark obstacle
              occupancy_grid[y_grid][x_grid] = max_cost;
            }
            
        }
    }
 
    // Step 3: Inflate obstacles
    const int inflation_radius = 1.5 * scale_factor;
    for(int i = 0; i < height*scale_factor; i++) {
      for(int j = 0; j < width*scale_factor; j++) {
        if(occupancy_grid[i][j] == max_cost) {
          for(int a = i-inflation_radius; a <= i+inflation_radius; a++) {
            for(int b = j-inflation_radius; b <= j+inflation_radius; b++) {
              double cost_factor = (double)sqrt((i-a)*(i-a) + (j-b)*(j-b))/inflation_radius;
              if(a > 0 && b > 0 && a < height*scale_factor && b < width*scale_factor && cost_factor < 1.0) {
                double cost = max_cost * (1.0 - cost_factor);
                if(cost > occupancy_grid[a][b]){
                  occupancy_grid[a][b] = (int)cost;
                }
              }
            }
          }
        }
      }
    }
 
    // Step 4: Publish costmap
    nav_msgs::msg::MapMetaData metadata;
    metadata.resolution = 1.0 / scale_factor;
    metadata.width = width * scale_factor;
    metadata.height = height * scale_factor;
    metadata.origin.position.x = -20;
    metadata.origin.position.y = -20;
    metadata.origin.orientation.w = 1.0;
    std::vector<int8_t> flattened_occupancy_grid;
    for(int i = 0; i < height*scale_factor; i++) {
      for(int j = 0; j < width*scale_factor; j++) {
        flattened_occupancy_grid.push_back(occupancy_grid[i][j]);
      }
    }
    nav_msgs::msg::OccupancyGrid og;
    og.header = scan->header;
    og.header.frame_id = "sim_world"; 
    og.info = metadata;
    og.data = flattened_occupancy_grid;
    // RCLCPP_INFO(this->get_logger(), "Generated and publishing OccupancyGrid with width: %d, height: %d, data size: %zu",
    //             og.info.width, og.info.height, og.data.size());
    occupancy_grid_pub_->publish(og);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}