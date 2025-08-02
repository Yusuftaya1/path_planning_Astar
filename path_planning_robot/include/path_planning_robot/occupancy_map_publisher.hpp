#ifndef OCCUPANCY_MAP_PUBLISHER_HPP_
#define OCCUPANCY_MAP_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>
#include <vector>
#include <random>
#include <set>
#include <array>

namespace path_planning_robot
{

/**
 * @brief Represents a cell in the maze with row and column coordinates
 */
struct Cell {
    int r, c;
    Cell(int row = 0, int col = 0) : r(row), c(col) {}
};

/**
 * @brief Represents a point in the maze with x and y coordinates
 * Provides comparison operators for use in containers
 */
struct Point {
    int x, y;
    Point(int _x = 0, int _y = 0) : x(_x), y(_y) {}
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
    bool operator<(const Point& other) const {
        return x < other.x || (x == other.x && y < other.y);
    }
};

/**
 * @brief Node that generates and publishes a maze as an occupancy grid
 * Uses Prim's algorithm to generate a maze with guaranteed solution paths
 */
class OccupancyMapPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the node and initializes maze generation
     * Sets up publishers, timers, and generates initial maze
     */
    OccupancyMapPublisher();
    virtual ~OccupancyMapPublisher() = default;

private:
    // Maze parameters
    static constexpr int ROWS = 50;  // Satır sayısı
    static constexpr int COLS = 50;  // Sütun sayısı
    static constexpr double RESOLUTION = 0.1;  // metre/hücre - daha detaylı grid için küçülttük
    
    // Random number generator
    std::random_device rd_;
    std::mt19937 gen_;
    
    // ...
    
    // Maze representation
    std::vector<std::vector<int>> maze_;
    std::vector<std::vector<bool>> visited_;
    Point start_;
    Point goal_;
    
    // ROS 2 publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    
    // Timer for periodic publishing
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // Map message
    nav_msgs::msg::OccupancyGrid map_msg_;
    
    /**
     * @brief Publishes the current maze state as an occupancy grid
     * Called periodically by the timer to update visualization
     */
    void publishMap();
    
    /**
     * @brief Initializes maze data structures and map message
     * Sets up dimensions, resolution, and coordinate frames
     */
    void initializeMaze();
    
    /**
     * @brief Creates the maze pattern and marks start/goal points
     * Calls generateMazePattern() and sets special cell values
     */
    void createMaze();
    
    // ...
    
    void generateMazePattern();
    
    bool inBounds(int r, int c) const;
};

} // namespace path_planning_robot

#endif // OCCUPANCY_MAP_PUBLISHER_HPP_ 