#ifndef A_STAR_PLANNER_HPP_
#define A_STAR_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>
#include <unordered_map>
#include <memory>
#include <vector>

namespace path_planning_robot
{

struct GridNode {
    int x, y;
    double g_cost;  // Cost from start to this node
    double h_cost;  // Heuristic cost from this node to goal
    double f_cost;  // Total cost (g_cost + h_cost)
    std::shared_ptr<GridNode> parent;

    GridNode(int _x = 0, int _y = 0) : x(_x), y(_y), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}

    bool operator==(const GridNode& other) const {
        return x == other.x && y == other.y;
    }
};

// Custom hash function for GridNode
struct GridNodeHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

// Custom comparator for priority queue
struct CompareNode {
    bool operator()(const std::shared_ptr<GridNode>& a, const std::shared_ptr<GridNode>& b) {
        return a->f_cost > b->f_cost;  // Min heap
    }
};

class AStarPlanner : public rclcpp::Node
{
public:
    AStarPlanner();
    virtual ~AStarPlanner() = default;

private:
    // ROS 2 subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // Map data
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    int map_width_;
    int map_height_;
    
    // Path planning
    std::vector<std::pair<int, int>> directions_;  // Possible movement directions
    
    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    // A* algorithm functions
    bool findPath(const GridNode& start, const GridNode& goal);
    double calculateHeuristic(const GridNode& current, const GridNode& goal);
    bool isValid(int x, int y) const;
    void publishPath(const std::vector<GridNode>& path);
    std::vector<GridNode> reconstructPath(std::shared_ptr<GridNode> goal_node);
};

} // namespace path_planning_robot

#endif // A_STAR_PLANNER_HPP_ 