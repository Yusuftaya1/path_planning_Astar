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

/**
 * @brief Represents a node in the A* search grid
 * Contains position, cost values, and parent pointer for path reconstruction
 */
struct GridNode {
    int x, y;                          // Grid coordinates
    double g_cost;                     // Cost from start to this node
    double h_cost;                     // Heuristic cost from this node to goal
    double f_cost;                     // Total cost (g_cost + h_cost)
    std::shared_ptr<GridNode> parent;  // Parent node for path reconstruction

    /**
     * @brief Constructs a grid node with given coordinates
     * @param _x X coordinate in grid
     * @param _y Y coordinate in grid
     */
    GridNode(int _x = 0, int _y = 0) : x(_x), y(_y), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}

    /**
     * @brief Equality comparison operator
     * @param other Node to compare with
     * @return true if coordinates match
     */
    bool operator==(const GridNode& other) const {
        return x == other.x && y == other.y;
    }
};

/**
 * @brief Hash function for grid coordinates
 * Enables use of coordinate pairs in unordered_map
 */
struct GridNodeHash {
    /**
     * @brief Computes hash value for coordinate pair
     * @param p Coordinate pair to hash
     * @return Hash value combining x and y coordinates
     */
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

/**
 * @brief Comparison function for priority queue
 * Orders nodes by f_cost for A* search
 */
struct CompareNode {
    /**
     * @brief Compares two nodes by their f_cost
     * @param a First node
     * @param b Second node
     * @return true if a's f_cost is greater (min-heap)
     */
    bool operator()(const std::shared_ptr<GridNode>& a, const std::shared_ptr<GridNode>& b) {
        return a->f_cost > b->f_cost;  // Min heap
    }
};

/**
 * @brief ROS 2 node that implements A* path planning
 * Subscribes to occupancy grid and publishes planned path
 */
class AStarPlanner : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the A* planner node
     * Initializes subscribers, publishers, and movement directions
     */
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
    
    /**
     * @brief Callback for map updates
     * Triggers path planning when new map is received
     * @param msg Received occupancy grid message
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    /**
     * @brief Implements A* path finding algorithm
     * @param start Start node in grid
     * @param goal Goal node in grid
     * @return true if path found, false otherwise
     */
    bool findPath(const GridNode& start, const GridNode& goal);
    
    /**
     * @brief Calculates Manhattan distance heuristic
     * @param current Current node
     * @param goal Goal node
     * @return Heuristic cost estimate
     */
    double calculateHeuristic(const GridNode& current, const GridNode& goal);
    
    /**
     * @brief Checks if given coordinates are valid and free
     * @param x X coordinate in grid
     * @param y Y coordinate in grid
     * @return true if position is valid and traversable
     */
    bool isValid(int x, int y) const;
    
    /**
     * @brief Publishes found path as ROS message
     * @param path Vector of nodes forming the path
     */
    void publishPath(const std::vector<GridNode>& path);
    
    /**
     * @brief Reconstructs path from goal to start
     * @param goal_node Goal node with parent pointers
     * @return Vector of nodes forming the path
     */
    std::vector<GridNode> reconstructPath(std::shared_ptr<GridNode> goal_node);
};

} // namespace path_planning_robot

#endif // A_STAR_PLANNER_HPP_ 