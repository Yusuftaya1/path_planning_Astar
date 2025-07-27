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
    
    // Path counting
    int total_paths_;
    std::set<std::vector<Point>> unique_paths_;
    static constexpr int MAX_PATH_LENGTH = 2000;  // Arttırıldı çünkü harita büyüdü
    
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
    
    /**
     * @brief Checks if given coordinates are within maze bounds
     * @param x Row coordinate to check
     * @param y Column coordinate to check
     * @return true if coordinates are valid, false otherwise
     */
    bool isValid(int x, int y) const;
    
    /**
     * @brief Prints the current maze state to ROS logs
     * Shows walls, paths, start and goal points
     */
    void printMazeToLog() const;
    
    /**
     * @brief Prints a section of the maze around given coordinates
     * @param center_x Center row coordinate
     * @param center_y Center column coordinate
     * @param label Description of the section (e.g., "Start", "Goal")
     */
    void printMapSection(int center_x, int center_y, const std::string& label);
    
    /**
     * @brief Generates maze pattern using Prim's algorithm
     * Creates a perfect maze with exactly one path between any two points
     */
    void generateMazePattern();
    
    /**
     * @brief Checks if given coordinates are within maze bounds
     * @param r Row coordinate to check
     * @param c Column coordinate to check
     * @return true if coordinates are valid, false otherwise
     */
    bool inBounds(int r, int c) const;
    
    /**
     * @brief Recursive DFS helper for finding all possible paths
     * @param r Current row coordinate
     * @param c Current column coordinate
     */
    void dfs(int r, int c);
    
    /**
     * @brief Finds all possible paths from start to goal
     * Uses DFS to explore and store unique paths
     */
    void findAllPaths();
    
    /**
     * @brief DFS implementation for path finding
     * @param current Current point in the maze
     * @param path Current path being explored
     * @param visited Track of visited cells
     */
    void findPathsDFS(Point current, std::vector<Point>& path, 
                     std::vector<std::vector<bool>>& visited);
    
    /**
     * @brief Prints statistics about found paths
     * Shows total paths, unique paths, shortest and longest paths
     */
    void printPathInfo() const;
};

} // namespace path_planning_robot

#endif // OCCUPANCY_MAP_PUBLISHER_HPP_ 