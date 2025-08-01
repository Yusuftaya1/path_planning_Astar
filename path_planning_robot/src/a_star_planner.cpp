#include "path_planning_robot/a_star_planner.hpp"
#include <cmath>

namespace path_planning_robot
{

/**
 * @brief Constructor for A* planner node
 * 
 * Initializes:
 * - Map subscriber with transient local QoS
 * - Path publisher
 * - Movement directions (4-connected grid)
 */
AStarPlanner::AStarPlanner()
: Node("a_star_planner")
{
    // Initialize subscribers and publishers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1)
    );
    
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "planned_path",
        rclcpp::QoS(rclcpp::KeepLast(1))
    );
    
    // Initialize possible movement directions (4-connected grid)
    directions_ = {
        {-1, 0},  // up
        {0, 1},   // right
        {1, 0},   // down
        {0, -1}   // left
    };
    
    RCLCPP_INFO(this->get_logger(), "A* Planner initialized");
}

/**
 * @brief Callback for map updates
 * 
 * When new map is received:
 * 1. Updates map data
 * 2. Attempts to find path from start to goal
 * 3. Publishes found path or reports failure
 * 
 * @param msg Received occupancy grid message
 */
void AStarPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = msg;
    map_width_ = msg->info.width;
    map_height_ = msg->info.height;
    
    RCLCPP_INFO(this->get_logger(), "Received map %dx%d", map_width_, map_height_);
    
    // For testing, try to find a path from start to goal
    GridNode start(1, 1);  // Same as in occupancy_map_publisher
    GridNode goal(map_height_ - 2, map_width_ - 2);
    
    RCLCPP_INFO(this->get_logger(), "Planning path from (%d,%d) to (%d,%d)",
                start.x, start.y, goal.x, goal.y);
    
    // Check if start and goal are valid
    if (!isValid(start.x, start.y)) {
        RCLCPP_ERROR(this->get_logger(), "Start position (%d,%d) is invalid!", start.x, start.y);
        return;
    }
    if (!isValid(goal.x, goal.y)) {
        RCLCPP_ERROR(this->get_logger(), "Goal position (%d,%d) is invalid!", goal.x, goal.y);
        return;
    }
    
    if (findPath(start, goal)) {
        RCLCPP_INFO(this->get_logger(), "Path found!");
    } else {
        RCLCPP_WARN(this->get_logger(), "No path found!");
    }
}

/**
 * @brief Implements A* path finding algorithm
 * 
 * Algorithm steps:
 * 1. Maintains open set (priority queue) and closed set
 * 2. Explores nodes based on f_cost (g_cost + h_cost)
 * 3. Uses Manhattan distance heuristic
 * 4. Reconstructs and publishes path when goal is reached
 * 
 * @param start Start node in grid
 * @param goal Goal node in grid
 * @return true if path found
 * @return false if no path exists
 */
bool AStarPlanner::findPath(const GridNode& start, const GridNode& goal)
{
    // Priority queue for open set
    std::priority_queue<std::shared_ptr<GridNode>, 
                       std::vector<std::shared_ptr<GridNode>>, 
                       CompareNode> open_set;
    
    // Hash map for closed set using coordinate pairs
    std::unordered_map<std::pair<int, int>, double, GridNodeHash> best_costs;
    
    // Initialize start node
    auto start_node = std::make_shared<GridNode>(start);
    start_node->h_cost = calculateHeuristic(*start_node, goal);
    start_node->f_cost = start_node->h_cost;
    
    open_set.push(start_node);
    
    int iterations = 0;
    const int max_iterations = map_width_ * map_height_ * 2;
    
    RCLCPP_INFO(this->get_logger(), "Starting A* search from (%d,%d) to (%d,%d)",
                start.x, start.y, goal.x, goal.y);
    
    while (!open_set.empty() && iterations < max_iterations) {
        iterations++;
        auto current = open_set.top();
        open_set.pop();
        
        // Check if we reached the goal
        if (current->x == goal.x && current->y == goal.y) {
            auto path = reconstructPath(current);
            publishPath(path);
            RCLCPP_INFO(this->get_logger(), "Path found in %d iterations with %zu steps and total cost %.2f",
                       iterations, path.size(), current->g_cost);
            return true;
        }
        
        // Add to closed set using coordinate pair
        auto current_pair = std::make_pair(current->x, current->y);
        
        // Skip if we already found a better path to this node
        if (best_costs.find(current_pair) != best_costs.end() && 
            best_costs[current_pair] <= current->g_cost) {
            continue;
        }
        
        // Update best cost to this node
        best_costs[current_pair] = current->g_cost;
        
        // Progress update every 1000 iterations
        if (iterations % 1000 == 0) {
            RCLCPP_INFO(this->get_logger(), "A* search in progress: %d iterations, current position (%d,%d), cost %.2f",
                       iterations, current->x, current->y, current->g_cost);
        }
        
        // Check all possible directions
        for (const auto& dir : directions_) {
            int new_x = current->x + dir.first;
            int new_y = current->y + dir.second;
            
            // Skip if not valid
            if (!isValid(new_x, new_y)) {
                continue;
            }
            
            // Calculate new costs
            double new_g_cost = current->g_cost + 1.0;  // Cost of 1 for orthogonal movements
            
            // Check if we already found a better path to this neighbor
            auto neighbor_pair = std::make_pair(new_x, new_y);
            if (best_costs.find(neighbor_pair) != best_costs.end() && 
                best_costs[neighbor_pair] <= new_g_cost) {
                continue;
            }
            
            // Create new neighbor node
            auto neighbor_node = std::make_shared<GridNode>(new_x, new_y);
            neighbor_node->g_cost = new_g_cost;
            neighbor_node->h_cost = calculateHeuristic(*neighbor_node, goal);
            neighbor_node->f_cost = neighbor_node->g_cost + neighbor_node->h_cost;
            neighbor_node->parent = current;
            
            open_set.push(neighbor_node);
        }
    }
    
    RCLCPP_WARN(this->get_logger(), "Path not found after %d iterations", iterations);
    return false;
}

/**
 * @brief Calculates Manhattan distance heuristic
 * 
 * Uses Manhattan distance (L1 norm) because:
 * 1. Grid-based movement
 * 2. Only horizontal/vertical movement allowed
 * 3. Admissible heuristic for A*
 * 
 * @param current Current node
 * @param goal Goal node
 * @return double Heuristic estimate of remaining cost
 */
double AStarPlanner::calculateHeuristic(const GridNode& current, const GridNode& goal)
{
    // Using Manhattan distance as heuristic
    return std::abs(current.x - goal.x) + std::abs(current.y - goal.y);
}

/**
 * @brief Checks if position is valid and traversable
 * 
 * Validates:
 * 1. Coordinates are within map bounds
 * 2. Cell is free (0) or special point (50, 25)
 * 
 * @param x X coordinate in grid
 * @param y Y coordinate in grid
 * @return true if position is valid and traversable
 * @return false if position is invalid or blocked
 */
bool AStarPlanner::isValid(int x, int y) const
{
    // Check bounds
    if (x < 0 || x >= map_height_ || y < 0 || y >= map_width_) {
        return false;
    }
    
    // Check if cell is free (not an obstacle)
    int index = x * map_width_ + y;
    if (index >= map_->data.size()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid map index %d (max: %zu) for position (%d,%d)",
                    index, map_->data.size(), x, y);
        return false;
    }
    
    // Debug output for specific positions
    if ((x == 1 && y == 1) || (x == map_height_-2 && y == map_width_-2)) {
        RCLCPP_INFO(this->get_logger(), "Checking position (%d,%d): value=%d", 
                   x, y, static_cast<int>(map_->data[index]));
    }
    
    // Accept free space (0), start point (50), and goal point (25)
    int cell_value = static_cast<int>(map_->data[index]);
    return cell_value == 0 || cell_value == 50 || cell_value == 25;
}

/**
 * @brief Reconstructs path from goal to start
 * 
 * Follows parent pointers to:
 * 1. Build complete path
 * 2. Reverse path to get start-to-goal order
 * 
 * @param goal_node Goal node with parent pointers
 * @return std::vector<GridNode> Complete path from start to goal
 */
std::vector<GridNode> AStarPlanner::reconstructPath(std::shared_ptr<GridNode> goal_node)
{
    std::vector<GridNode> path;
    auto current = goal_node;
    
    while (current != nullptr) {
        path.push_back(*current);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
    RCLCPP_INFO(this->get_logger(), "Path reconstructed with %zu steps", path.size());
    return path;
}

/**
 * @brief Publishes found path as ROS message
 * 
 * Converts path to ROS message by:
 * 1. Converting grid coordinates to world coordinates
 * 2. Adding map origin offset
 * 3. Setting proper orientation
 * 
 * @param path Vector of nodes forming the path
 */
void AStarPlanner::publishPath(const std::vector<GridNode>& path)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();
    
    // Get map origin
    double origin_x = map_->info.origin.position.x;
    double origin_y = map_->info.origin.position.y;
    
    for (const auto& node : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        
        // Convert grid coordinates to world coordinates
        // Add map origin and center the position within the cell
        pose.pose.position.x = origin_x + (node.y + 0.5) * map_->info.resolution;
        pose.pose.position.y = origin_y + (node.x + 0.5) * map_->info.resolution;
        pose.pose.position.z = 0.0;
        
        // Set orientation to default (looking forward)
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        
        path_msg.poses.push_back(pose);
    }
    
    RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", path_msg.poses.size());
    RCLCPP_INFO(this->get_logger(), "First pose: (%.2f, %.2f), Last pose: (%.2f, %.2f)",
                path_msg.poses.front().pose.position.x,
                path_msg.poses.front().pose.position.y,
                path_msg.poses.back().pose.position.x,
                path_msg.poses.back().pose.position.y);
    
    path_pub_->publish(path_msg);
}

} // namespace path_planning_robot

/**
 * @brief Main function for the A* planner node
 * 
 * 1. Initializes ROS 2
 * 2. Creates and spins the node
 * 3. Handles shutdown
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int Exit status
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planning_robot::AStarPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 