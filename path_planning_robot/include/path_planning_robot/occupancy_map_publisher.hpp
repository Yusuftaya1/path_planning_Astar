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

struct Cell {
    int r, c;
    Cell(int row = 0, int col = 0) : r(row), c(col) {}
};

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

class OccupancyMapPublisher : public rclcpp::Node
{
public:
    OccupancyMapPublisher();
    virtual ~OccupancyMapPublisher() = default;

private:
    // Maze parameters
    static constexpr int ROWS = 40;  // Satır sayısı
    static constexpr int COLS = 40;  // Sütun sayısı
    static constexpr double RESOLUTION = 0.2;  // metre/hücre
    
    // Random number generator
    std::random_device rd_;
    std::mt19937 gen_;
    
    // Path counting
    int total_paths_;
    std::set<std::vector<Point>> unique_paths_;
    static constexpr int MAX_PATH_LENGTH = 1000;
    
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
    
    // Timer callback for publishing map
    void publishMap();
    
    // Initialize maze with default values
    void initializeMaze();
    
    // Create maze pattern
    void createMaze();
    
    // Helper functions
    bool isValid(int x, int y) const;
    void printMazeToLog() const;
    
    // DFS Maze Generation
    void generateMazePattern();
    bool inBounds(int r, int c) const;
    void dfs(int r, int c);
    
    // Path finding functions
    void findAllPaths();
    void findPathsDFS(Point current, std::vector<Point>& path, 
                     std::vector<std::vector<bool>>& visited);
    void printPathInfo() const;
};

} // namespace path_planning_robot

#endif // OCCUPANCY_MAP_PUBLISHER_HPP_ 