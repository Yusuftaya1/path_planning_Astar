#include "path_planning_robot/occupancy_map_publisher.hpp"
#include <cmath>
#include <stack>
#include <algorithm>

namespace path_planning_robot
{

/**
 * @brief Constructor for OccupancyMapPublisher node
 * 
 * Initializes the node with:
 * - Map publisher with transient local QoS
 * - Timer for periodic publishing
 * - Random number generator for maze generation
 * - Creates and publishes initial maze
 */
OccupancyMapPublisher::OccupancyMapPublisher()
: Node("occupancy_map_publisher"), gen_(rd_()){
    // Initialize publisher
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()
    );

    // Initialize maze
    initializeMaze();
    
    // Create maze pattern
    createMaze();
    
    // Sadece maze oluşturulacak ve publish edilecek
    
    // Create timer for periodic publishing (2Hz)
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&OccupancyMapPublisher::publishMap, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Occupancy Map Publisher initialized");
}

/**
 * @brief Initializes maze data structures and map message
 * 
 * Sets up:
 * - Maze dimensions and grid
 * - Start and goal positions
 * - Map message parameters (resolution, origin, etc.)
 * - Initializes occupancy grid data
 */
void OccupancyMapPublisher::initializeMaze(){
    // Initialize maze with all walls
    maze_.resize(ROWS, std::vector<int>(COLS, 1));
    visited_.resize(ROWS, std::vector<bool>(COLS, false));
    
    // Set start and goal points
    start_ = Point(1, 1);  // Başlangıç noktası
    goal_ = Point(ROWS-2, COLS-2);  // Hedef noktası
    
    // Initialize map message
    map_msg_ = nav_msgs::msg::OccupancyGrid();  // Clear all fields
    
    // Set header
    map_msg_.header.frame_id = "map";
    map_msg_.header.stamp = this->now();
    
    // Set map metadata
    map_msg_.info.map_load_time = this->now();
    map_msg_.info.resolution = RESOLUTION;
    map_msg_.info.width = COLS;
    map_msg_.info.height = ROWS;
    
    // Set origin - center the map
    map_msg_.info.origin.position.x = -(COLS * RESOLUTION) / 2.0;
    map_msg_.info.origin.position.y = -(ROWS * RESOLUTION) / 2.0;
    map_msg_.info.origin.position.z = 0.0;
    map_msg_.info.origin.orientation.x = 0.0;
    map_msg_.info.origin.orientation.y = 0.0;
    map_msg_.info.origin.orientation.z = 0.0;
    map_msg_.info.origin.orientation.w = 1.0;
    
    // Initialize map data with the correct size
    map_msg_.data.resize(ROWS * COLS, -1);  // -1 for unknown
    
    RCLCPP_INFO(this->get_logger(), "Map initialized with size %dx%d", ROWS, COLS);
    RCLCPP_INFO(this->get_logger(), "Resolution: %.2f meters/cell", RESOLUTION);
    RCLCPP_INFO(this->get_logger(), "Origin: (%.2f, %.2f)", 
                map_msg_.info.origin.position.x,
                map_msg_.info.origin.position.y);
    RCLCPP_INFO(this->get_logger(), "Start grid position: (%d, %d)", start_.x, start_.y);
    RCLCPP_INFO(this->get_logger(), "Start world position: (%.2f, %.2f)",
                map_msg_.info.origin.position.x + (start_.y + 0.5) * RESOLUTION,
                map_msg_.info.origin.position.y + (start_.x + 0.5) * RESOLUTION);
}

/**
 * @brief Creates the complete maze pattern
 * 
 * Process:
 * 1. Generates base maze using Prim's algorithm
 * 2. Ensures start and goal points are accessible
 * 3. Copies maze to occupancy grid
 * 4. Marks special points (start/goal)
 * 5. Prints debug information
 */
void OccupancyMapPublisher::createMaze() {
    // Maze oluştur
    generateMazePattern();

    // Start ve Goal açık
    maze_[start_.x][start_.y] = 0;
    maze_[goal_.x][goal_.y] = 0;

    // Ekstra yollar ekle (loop)
    int extra_paths = (ROWS * COLS) / 10;
    for (int i = 0; i < extra_paths; i++) {
        int r = std::uniform_int_distribution<>(2, ROWS-3)(gen_);
        int c = std::uniform_int_distribution<>(2, COLS-3)(gen_);
        if (maze_[r][c] == 1) {
            int walls = 0;
            bool up = maze_[r-1][c] == 1;
            bool down = maze_[r+1][c] == 1;
            bool left = maze_[r][c-1] == 1;
            bool right = maze_[r][c+1] == 1;
            walls += up + down + left + right;

            if (walls >= 3) {
                // Mevcut hücreyi aç
                maze_[r][c] = 0;

                // Komşu engelleri listele
                std::vector<std::pair<int,int>> neighbors;
                if (up) neighbors.emplace_back(r-1, c);
                if (down) neighbors.emplace_back(r+1, c);
                if (left) neighbors.emplace_back(r, c-1);
                if (right) neighbors.emplace_back(r, c+1);

                // Rastgele bir komşu engeli daha kaldır
                if (!neighbors.empty()) {
                    auto extra = neighbors[std::uniform_int_distribution<>(0, neighbors.size()-1)(gen_)];
                    maze_[extra.first][extra.second] = 0;
                }
            }
        }
    }

    // OccupancyGrid verisini doldur
    map_msg_.data.assign(ROWS * COLS, -1);
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            std::size_t idx = i * COLS + j;
            map_msg_.data[idx] = (maze_[i][j] == 1) ? 100 : 0;
        }
    }

    // Start ve Goal işaretle
    map_msg_.data[start_.x * COLS + start_.y] = 50;
    map_msg_.data[goal_.x * COLS + goal_.y] = 25;}



/**
 * @brief Publishes current maze state as occupancy grid
 * 
 * Called periodically to:
 * 1. Update timestamp
 * 2. Publish current maze state
 * Used by RViz and other nodes for visualization
 */
void OccupancyMapPublisher::publishMap(){
    // Update timestamp
    map_msg_.header.stamp = this->now();
    
    // Ensure map data is properly set
    if (map_msg_.data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Map data is empty!");
        return;
    }

    if (map_msg_.data.size() != ROWS * COLS) {
        RCLCPP_ERROR(this->get_logger(), "Map data size mismatch! Expected: %d, Got: %zu", 
                    ROWS * COLS, map_msg_.data.size());
        return;
    }

    // Debug output
    RCLCPP_DEBUG(this->get_logger(), "Publishing map with frame_id: %s", 
                 map_msg_.header.frame_id.c_str());
    RCLCPP_DEBUG(this->get_logger(), "Map dimensions: %dx%d", 
                 map_msg_.info.width, map_msg_.info.height);
    
    map_publisher_->publish(map_msg_);
}

/**
 * @brief Checks if given coordinates are within maze bounds
 * 
 * @param r Row coordinate to check
 * @param c Column coordinate to check
 * @return true if coordinates are within bounds
 * @return false if coordinates are outside bounds
 */
bool OccupancyMapPublisher::inBounds(int r, int c) const{
    return r >= 0 && r < ROWS && c >= 0 && c < COLS;
}

/**
 * @brief Generates maze pattern using Prim's algorithm
 * 
 * Algorithm steps:
 * 1. Initializes all cells as walls
 * 2. Starts from (1,1) and expands using frontiers
 * 3. Randomly connects paths to create maze structure
 * 4. Ensures borders are walls
 * 5. Clears areas around start and goal
 * 6. Adds some random paths for alternatives
 */
void OccupancyMapPublisher::generateMazePattern(){
    // Initialize all cells as walls
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            maze_[i][j] = 1;  // 1 represents wall
            visited_[i][j] = false;
        }
    }

    // Frontiers for Prim's algorithm
    std::vector<Cell> frontiers;
    
    // Start from cell (1,1)
    int start_r = 1;
    int start_c = 1;
    
    // Mark the start cell as path
    maze_[start_r][start_c] = 0;
    visited_[start_r][start_c] = true;
    
    // Add initial frontiers
    if (start_r + 2 < ROWS-1) frontiers.push_back(Cell{start_r + 2, start_c});
    if (start_c + 2 < COLS-1) frontiers.push_back(Cell{start_r, start_c + 2});
    
    // Prim's algorithm
    while (!frontiers.empty()) {
        // Pick a random frontier cell
        int idx = std::uniform_int_distribution<>(0, frontiers.size() - 1)(gen_);
        Cell current = frontiers[idx];
        
        // Remove it from frontiers
        frontiers[idx] = frontiers.back();
        frontiers.pop_back();
        
        // Skip if already visited
        if (visited_[current.r][current.c]) continue;
        
        // Find neighbors that are paths (visited)
        std::vector<Cell> neighbors;
        if (current.r >= 2 && visited_[current.r-2][current.c])
            neighbors.push_back(Cell{current.r-2, current.c});
        if (current.r + 2 < ROWS-1 && visited_[current.r+2][current.c])
            neighbors.push_back(Cell{current.r+2, current.c});
        if (current.c >= 2 && visited_[current.r][current.c-2])
            neighbors.push_back(Cell{current.r, current.c-2});
        if (current.c + 2 < COLS-1 && visited_[current.r][current.c+2])
            neighbors.push_back(Cell{current.r, current.c+2});
            
        if (!neighbors.empty()) {
            // Pick a random neighbor
            Cell neighbor = neighbors[std::uniform_int_distribution<>(0, neighbors.size() - 1)(gen_)];
            
            // Connect current cell to the neighbor
            maze_[current.r][current.c] = 0;
            maze_[(current.r + neighbor.r)/2][(current.c + neighbor.c)/2] = 0;
            visited_[current.r][current.c] = true;
            
            // Add new frontiers
            if (current.r >= 2 && !visited_[current.r-2][current.c])
                frontiers.push_back(Cell{current.r-2, current.c});
            if (current.r + 2 < ROWS-1 && !visited_[current.r+2][current.c])
                frontiers.push_back(Cell{current.r+2, current.c});
            if (current.c >= 2 && !visited_[current.r][current.c-2])
                frontiers.push_back(Cell{current.r, current.c-2});
            if (current.c + 2 < COLS-1 && !visited_[current.r][current.c+2])
                frontiers.push_back(Cell{current.r, current.c+2});
        }
    }

    // Ensure the borders are walls
    for (int i = 0; i < ROWS; ++i) {
        maze_[i][0] = maze_[i][COLS-1] = 1;
    }
    for (int j = 0; j < COLS; ++j) {
        maze_[0][j] = maze_[ROWS-1][j] = 1;
    }

    // Set start and goal points
    maze_[start_.x][start_.y] = 0;
    maze_[goal_.x][goal_.y] = 0;

    // Clear small area around start and goal
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

    // Add a few random paths to create loops (optional)
    int extra_paths = (ROWS + COLS) / 4;
    for (int i = 0; i < extra_paths; i++) {
        int r = std::uniform_int_distribution<>(2, ROWS-3)(gen_);
        int c = std::uniform_int_distribution<>(2, COLS-3)(gen_);
        if (r % 2 == 0) r--;
        if (c % 2 == 0) c--;
        maze_[r][c] = 0;
    }
}
} // namespace path_planning_robot

/**
 * @brief Main function for the occupancy map publisher node
 * 
 * 1. Initializes ROS 2
 * 2. Creates and spins the node
 * 3. Handles shutdown
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int Exit status
 */
int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planning_robot::OccupancyMapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}