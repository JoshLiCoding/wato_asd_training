#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

  private:
    robot::MapMemoryCore map_memory_;
    void integrateCostmap();
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    double x, y, yaw;
    double last_x, last_y;
    const double distance_threshold = 1.5;
    const int scale_factor = 10;
    const int width = 40, height = 40;

    // Flags
    bool costmap_updated_ = false;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_ = false;
};

#endif 
