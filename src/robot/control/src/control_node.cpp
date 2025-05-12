#include "control_node.hpp"
#include <cmath>
#include <optional>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Initialize parameters
  lookahead_distance_ = 1.0;  // Lookahead distance
  goal_tolerance_ = 0.5;     // Distance to consider the goal reached
  linear_speed_ = 0.5;       // Constant forward speed

  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { ControlNode::controlLoop(); });
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if(current_path_->poses.empty()) {
    return std::nullopt;
  }
  geometry_msgs::msg::PoseStamped ps;
  for(auto &pose:current_path_->poses) {
    if(abs(computeDistance(pose.pose, robot_odom_->pose.pose) - lookahead_distance_) <= 0.1) {
      ps.pose = pose.pose;
      return ps;
    }
  }
  ps.pose = current_path_->poses.back().pose;
  // RCLCPP_INFO(this->get_logger(), "Dist: %f", abs(computeDistance(ps.pose, robot_odom_->pose.pose)));
  
  if(abs(computeDistance(ps.pose, robot_odom_->pose.pose)) <= goal_tolerance_) {
    return std::nullopt;
  }
  return ps;
}
geometry_msgs::msg::Point ControlNode::transformToRobotFrame(const geometry_msgs::msg::Point& global_point, const geometry_msgs::msg::Pose& robot_pose) {
  double robot_yaw = extractYaw(robot_pose.orientation);
  double cos_yaw = cos(robot_yaw);
  double sin_yaw = sin(robot_yaw);
  geometry_msgs::msg::Point robot_point;
  robot_point.x = cos_yaw * (global_point.x - robot_pose.position.x) + sin_yaw * (global_point.y - robot_pose.position.y);
  robot_point.y = -sin_yaw * (global_point.x - robot_pose.position.x) + cos_yaw * (global_point.y - robot_pose.position.y);
  robot_point.z = 0.0;
  return robot_point;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    // TODO: Implement logic to compute velocity commands
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_speed_;
    geometry_msgs::msg::Point robot_frame_point = ControlNode::transformToRobotFrame(target.pose.position, robot_odom_->pose.pose);

    double x_l = robot_frame_point.x;
    double y_l = robot_frame_point.y;

    // 2. Calculate the angle to the lookahead point in the robot frame
    double alpha = std::atan2(y_l, x_l);

    // 3. Calculate the angular velocity using the pure pursuit formula
    double lD = std::sqrt(x_l * x_l + y_l * y_l);
    double k = 1.0; // Tuning gain
    cmd_vel.angular.z = k * (2.0 * linear_speed_ * std::sin(alpha)) / lD;

    return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b) {
    // TODO: Implement distance calculation between two points
    return sqrt((a.position.x-b.position.x)*(a.position.x-b.position.x)+(a.position.y-b.position.y)*(a.position.y-b.position.y));
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}

void ControlNode::controlLoop() {
        // Skip control if no path or odometry data is available
        if (!current_path_ || !robot_odom_) {
            return;
        }
 
        // Find the lookahead point
        auto lookahead_point = findLookaheadPoint();
        if (!lookahead_point) {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            cmd_vel_pub_->publish(cmd_vel);
            return;  // No valid lookahead point found
        }
 
        // Compute velocity command
        auto cmd_vel = computeVelocity(*lookahead_point);
 
        // Publish the velocity command
        cmd_vel_pub_->publish(cmd_vel);
    }


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
