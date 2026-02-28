cmake_minimum_required(VERSION 3.8)
project(ompl_planner_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(mav_trajectory_generation REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

# Find OMPL using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(OMPL REQUIRED ompl)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OMPL_INCLUDE_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})

# ============================================================================
# Exploration Manager Node
# ============================================================================
add_executable(exploration_manager_node src/exploration_manager_node.cpp)
ament_target_dependencies(exploration_manager_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  mav_msgs
  mav_planning_msgs
  mav_trajectory_generation
  Eigen3
  PCL
  pcl_conversions
  tf2
)
target_link_libraries(exploration_manager_node
  ${PCL_LIBRARIES}
)
target_include_directories(exploration_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ============================================================================
# RRT Path Planner Node
# ============================================================================
add_executable(rrt_path_planner_node src/rrt_path_planner_node.cpp)
ament_target_dependencies(rrt_path_planner_node
  rclcpp
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  mav_planning_msgs
  mav_trajectory_generation
  octomap_msgs
  Eigen3
  PCL
)
target_link_libraries(rrt_path_planner_node
  ${PCL_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
target_include_directories(rrt_path_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ============================================================================
# Install executables
# ============================================================================
install(TARGETS
  exploration_manager_node
  rrt_path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
