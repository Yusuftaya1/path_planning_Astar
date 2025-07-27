cmake_minimum_required(VERSION 3.8)
project(path_planning_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executables
add_executable(occupancy_map_publisher src/occupancy_map_publisher.cpp)
target_include_directories(occupancy_map_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(occupancy_map_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  occupancy_map_publisher
  "rclcpp"
  "nav_msgs"
)

add_executable(a_star_planner src/a_star_planner.cpp)
target_include_directories(a_star_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(a_star_planner PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  a_star_planner
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
)

install(TARGETS
  occupancy_map_publisher
  a_star_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
