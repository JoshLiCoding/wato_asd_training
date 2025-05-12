#include "planner_node.hpp"
#include <cmath>
#include <unordered_map>
#include <vector>
#include <queue>

PlannerNode::PlannerNode() : Node("planner"), state_(State::WAITING_FOR_GOAL), planner_(robot::PlannerCore(this->get_logger())) {
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}

bool PlannerNode::goalReached() {
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

void PlannerNode::timerCallback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (PlannerNode::goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            planPath();
        }
    }
}

// ------------------- Supporting Structures -------------------
 
// 2D grid index
struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double g_score;
  double h_score;
  double f_score;  // f = g + h
  CellIndex parent;
  AStarNode(): index(CellIndex(-1, -1)), g_score(-1), h_score(-1), f_score(-1), parent{CellIndex(-1, -1)} {}
  AStarNode(CellIndex idx, double g, double h, double f, CellIndex parent) : index(idx), g_score(g), h_score(h), f_score(f), parent{parent} {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

double get_dist(double x1, double y1, double x2, double y2) {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

void PlannerNode::planPath() {
    if (!goal_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }

    // A* Implementation (pseudo-code)
    std::unordered_map<CellIndex, AStarNode, CellIndexHash> open_set;
    std::unordered_map<CellIndex, AStarNode, CellIndexHash> closed_set;

    // Init start node
    CellIndex start_index = CellIndex(round((robot_pose_.position.x+20)*scale_factor), round((robot_pose_.position.y+20)*scale_factor));
    double start_h_score = get_dist((robot_pose_.position.x+20)*scale_factor, (robot_pose_.position.y+20)*scale_factor, (goal_.point.x+20)*scale_factor, (goal_.point.y+20)*scale_factor);
    AStarNode start_node = AStarNode(start_index, 0, start_h_score, 0+start_h_score+current_map_.data[start_index.y*current_map_.info.width+start_index.x]*100, CellIndex(-1, -1));
    open_set.emplace(start_index, start_node);

    while(!open_set.empty()) {
      AStarNode current; // don't use PQ here to avoid manually implementing delete operation. Time complexity is O(n) each time
      for(const auto& [key, value] : open_set) {
        if(current.f_score == -1 || current.f_score > value.f_score) {
          current = value;
        }
      }
      open_set.erase(current.index);
      closed_set.emplace(current.index, current);

      if(current.index.x == round((goal_.point.x+20)*scale_factor) && current.index.y == round((goal_.point.y+20)*scale_factor)) {
        // RCLCPP_INFO(this->get_logger(), "goal: x: %f, y: %f", (goal_.point.x+20)*scale_factor, (goal_.point.y+20)*scale_factor);
        // RCLCPP_INFO(this->get_logger(), "cur: x: %d, y: %d, f_score: %d", current.index.x, current.index.y, current.f_score);
        break;
      }
      
      int delta_2d_coords[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
      for(auto& i: delta_2d_coords){
        CellIndex neighbour = CellIndex(current.index.x+i[0], current.index.y+i[1]);
        if(neighbour.x >= 0 && neighbour.x < current_map_.info.width && neighbour.y >= 0 && neighbour.y < current_map_.info.height){
          if(closed_set.find(neighbour) != closed_set.end()) {
            continue;
          }
          double new_cost = current_map_.data[neighbour.y*current_map_.info.width+neighbour.x]*100; //try to avoid if possible
          double new_g_score = current.g_score + sqrt(i[0]*i[0]+i[1]*i[1]);
          double new_h_score = get_dist(neighbour.x, neighbour.y, (goal_.point.x+20)*scale_factor, (goal_.point.y+20)*scale_factor);
          AStarNode neighbour_node = AStarNode(neighbour, new_g_score, new_h_score, new_g_score+new_h_score+new_cost, current.index);
          if (open_set.find(neighbour) == open_set.end() || open_set.find(neighbour)->second.f_score > new_g_score+new_h_score) {
            open_set.emplace(neighbour, neighbour_node);
          }
        }
      }
    }


    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "sim_world";
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    CellIndex current_index = CellIndex(round((goal_.point.x+20)*scale_factor), round((goal_.point.y+20)*scale_factor));

    if(closed_set.find(current_index) == closed_set.end()) {
      path_pub_->publish(path);
      return;
    }
    while(current_index != CellIndex(-1, -1)) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = (double)current_index.x/scale_factor-20;
      pose.pose.position.y = (double)current_index.y/scale_factor-20;
      pose.pose.orientation.w = 1.0;

      pose.header.frame_id = "sim_world";
      waypoints.insert(waypoints.begin(), pose);
      current_index = closed_set[current_index].parent;
    }
    path.poses = waypoints;

    path_pub_->publish(path);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
