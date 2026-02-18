cmake_minimum_required(VERSION 3.16)
project(navigation_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(frontier_exploration src/frontier_exploration.cpp)
ament_target_dependencies(frontier_exploration rclcpp nav_msgs sensor_msgs geometry_msgs octomap_msgs)
ament_target_dependencies(frontier_exploration visualization_msgs)
target_link_libraries(frontier_exploration ${PCL_LIBRARIES} octomap)

install(TARGETS
  frontier_exploration
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
