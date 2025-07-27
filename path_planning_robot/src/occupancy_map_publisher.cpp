#include "path_planning_robot/occupancy_map_publisher.hpp"
#include <cmath>
#include <stack>
#include <algorithm>

namespace path_planning_robot
{

OccupancyMapPublisher::OccupancyMapPublisher()
: Node("occupancy_map_publisher"), gen_(rd_()), total_paths_(0)
{
    // Initialize publisher
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()
    );

    // Initialize maze
    initializeMaze();
    
    // Create maze pattern
    createMaze();
    
    // Find all possible paths
    findAllPaths();
    
    // Print maze and path info
    printMazeToLog();
    printPathInfo();
    
    // Create timer for periodic publishing (2Hz)
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&OccupancyMapPublisher::publishMap, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Occupancy Map Publisher initialized");
}

void OccupancyMapPublisher::initializeMaze()
{
    // Initialize maze with all walls
    maze_.resize(ROWS, std::vector<int>(COLS, 1));
    visited_.resize(ROWS, std::vector<bool>(COLS, false));
    
    // Set start and goal points
    start_ = Point(1, 1);  // Başlangıç noktası
    goal_ = Point(ROWS-2, COLS-2);  // Hedef noktası
    
    // Initialize map message
    map_msg_.header.frame_id = "map";
    map_msg_.info.resolution = RESOLUTION;
    map_msg_.info.width = COLS;
    map_msg_.info.height = ROWS;
    map_msg_.info.origin.position.x = -5.0;
    map_msg_.info.origin.position.y = -5.0;
    map_msg_.info.origin.position.z = 0.0;
    map_msg_.info.origin.orientation.w = 1.0;
    
    // Initialize map data
    map_msg_.data.resize(ROWS * COLS, 0);
}

void OccupancyMapPublisher::createMaze()
{
    // Generate maze using DFS
    generateMazePattern();

    // Ensure start and goal points are accessible
    maze_[start_.x][start_.y] = 0;
    maze_[goal_.x][goal_.y] = 0;

    // Copy maze to occupancy grid
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            map_msg_.data[i * COLS + j] = maze_[i][j] * 100;
        }
    }

    // Mark start and goal points
    map_msg_.data[start_.x * COLS + start_.y] = 50;  // Start point (gri)
    map_msg_.data[goal_.x * COLS + goal_.y] = 25;   // Goal point (açık gri)

    RCLCPP_INFO(this->get_logger(), "Start point: (%d, %d)", start_.x, start_.y);
    RCLCPP_INFO(this->get_logger(), "Goal point: (%d, %d)", goal_.x, goal_.y);
}

bool OccupancyMapPublisher::inBounds(int r, int c) const
{
    return r >= 0 && r < ROWS && c >= 0 && c < COLS;
}

void OccupancyMapPublisher::generateMazePattern()
{
    // Initialize all cells as walls
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            maze_[i][j] = 1;
            visited_[i][j] = false;
        }
    }

    // Create a guaranteed path from start to goal first
    int current_x = start_.x;
    int current_y = start_.y;
    maze_[current_x][current_y] = 0;  // Start point

    // Create a snake-like path to the goal
    while (current_x < goal_.x) {
        current_x += 2;
        maze_[current_x][current_y] = 0;
        maze_[current_x-1][current_y] = 0;
    }
    while (current_y < goal_.y) {
        current_y += 2;
        maze_[current_x][current_y] = 0;
        maze_[current_x][current_y-1] = 0;
    }

    // Now use DFS to create additional paths
    for (int i = 1; i < ROWS-1; i += 2) {
        for (int j = 1; j < COLS-1; j += 2) {
            if (!visited_[i][j] && maze_[i][j] == 1) {
                dfs(i, j);
            }
        }
    }

    // Add some random connections to create alternative paths
    int num_extra_paths = ROWS / 4;
    for (int i = 0; i < num_extra_paths; ++i) {
        int x = std::uniform_int_distribution<>(2, ROWS-3)(gen_);
        int y = std::uniform_int_distribution<>(2, COLS-3)(gen_);
        
        if (x % 2 == 0) x--;
        if (y % 2 == 0) y--;
        
        // Create a small opening
        if (maze_[x][y] == 1) {
            // Check if this creates a reasonable path
            int walls_around = 0;
            walls_around += (maze_[x-1][y] == 1);
            walls_around += (maze_[x+1][y] == 1);
            walls_around += (maze_[x][y-1] == 1);
            walls_around += (maze_[x][y+1] == 1);
            
            if (walls_around >= 2) {  // Only create opening if it doesn't make the path too easy
                maze_[x][y] = 0;
            }
        }
    }

    // Ensure the borders are walls
    for (int i = 0; i < ROWS; ++i) {
        maze_[i][0] = maze_[i][COLS-1] = 1;
    }
    for (int j = 0; j < COLS; ++j) {
        maze_[0][j] = maze_[ROWS-1][j] = 1;
    }

    // Ensure start and goal areas are clear
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (inBounds(start_.x + dx, start_.y + dy)) {
                maze_[start_.x + dx][start_.y + dy] = 0;
            }
            if (inBounds(goal_.x + dx, goal_.y + dy)) {
                maze_[goal_.x + dx][goal_.y + dy] = 0;
            }
        }
    }
}

void OccupancyMapPublisher::dfs(int r, int c)
{
    visited_[r][c] = true;
    maze_[r][c] = 0;  // Mark current cell as path

    // Define possible directions (up, right, down, left)
    std::array<Cell, 4> dirs = {
        Cell(r-2, c), // up
        Cell(r+2, c), // down
        Cell(r, c-2), // left
        Cell(r, c+2)  // right
    };

    // Shuffle directions randomly
    std::shuffle(dirs.begin(), dirs.end(), gen_);

    // Try each direction
    for (const auto& d : dirs) {
        if (inBounds(d.r, d.c) && !visited_[d.r][d.c]) {
            // Only create path if it won't interfere with the main solution path
            if (maze_[d.r][d.c] == 1) {  // If the target cell is still a wall
                // Create the path
                maze_[(r + d.r)/2][(c + d.c)/2] = 0;
                maze_[d.r][d.c] = 0;
                dfs(d.r, d.c);
            }
        }
    }
}

void OccupancyMapPublisher::findAllPaths()
{
    std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS, false));
    std::vector<Point> current_path;
    
    // Start DFS from start point
    findPathsDFS(start_, current_path, visited);
    
    RCLCPP_INFO(this->get_logger(), "Found %d unique paths", static_cast<int>(unique_paths_.size()));
}

void OccupancyMapPublisher::findPathsDFS(Point current, std::vector<Point>& path,
                                       std::vector<std::vector<bool>>& visited)
{
    // Check if path is too long
    if (path.size() > MAX_PATH_LENGTH) {
        return;
    }

    // Mark current cell as visited
    visited[current.x][current.y] = true;
    path.push_back(current);

    // If we reached the goal, save the path
    if (current.x == goal_.x && current.y == goal_.y) {
        unique_paths_.insert(path);
        total_paths_++;
    } else {
        // Direction vectors (up, right, down, left)
        const int dx[] = {-1, 0, 1, 0};
        const int dy[] = {0, 1, 0, -1};

        // Try all possible directions
        for (int i = 0; i < 4; i++) {
            int new_x = current.x + dx[i];
            int new_y = current.y + dy[i];

            // Check if the new position is valid and not visited
            if (isValid(new_x, new_y) && !visited[new_x][new_y]) {
                findPathsDFS(Point(new_x, new_y), path, visited);
            }
        }
    }

    // Backtrack
    visited[current.x][current.y] = false;
    path.pop_back();
}

void OccupancyMapPublisher::printPathInfo() const
{
    RCLCPP_INFO(this->get_logger(), "Path Analysis:");
    RCLCPP_INFO(this->get_logger(), "Total paths attempted: %d", total_paths_);
    RCLCPP_INFO(this->get_logger(), "Unique paths found: %zu", unique_paths_.size());
    
    if (!unique_paths_.empty()) {
        // Find shortest and longest paths
        auto shortest = std::min_element(unique_paths_.begin(), unique_paths_.end(),
            [](const std::vector<Point>& a, const std::vector<Point>& b) {
                return a.size() < b.size();
            });
        auto longest = std::max_element(unique_paths_.begin(), unique_paths_.end(),
            [](const std::vector<Point>& a, const std::vector<Point>& b) {
                return a.size() < b.size();
            });
        
        RCLCPP_INFO(this->get_logger(), "Shortest path length: %zu steps", shortest->size());
        RCLCPP_INFO(this->get_logger(), "Longest path length: %zu steps", longest->size());
    }
}

void OccupancyMapPublisher::publishMap()
{
    map_msg_.header.stamp = this->now();
    map_publisher_->publish(map_msg_);
}

bool OccupancyMapPublisher::isValid(int x, int y) const
{
    return (x >= 0 && x < ROWS &&
            y >= 0 && y < COLS &&
            maze_[x][y] == 0);
}

void OccupancyMapPublisher::printMazeToLog() const
{
    std::string maze_str = "\nMaze Pattern:\n";
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            if (i == start_.x && j == start_.y)
                maze_str += "S ";
            else if (i == goal_.x && j == goal_.y)
                maze_str += "G ";
            else if (maze_[i][j] == 1)
                maze_str += "# ";
            else
                maze_str += ". ";
        }
        maze_str += "\n";
    }
    RCLCPP_INFO(this->get_logger(), "%s", maze_str.c_str());
}

} // namespace path_planning_robot

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planning_robot::OccupancyMapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 