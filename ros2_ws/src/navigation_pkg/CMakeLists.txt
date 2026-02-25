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
find_package(fcl REQUIRED)
if(NOT DEFINED OMPL_DIR)
  # Common system location for OMPL CMake config on Debian/Ubuntu
  set(OMPL_DIR "/usr/share/ompl/cmake" CACHE PATH "Path to OMPLConfig.cmake")
  list(APPEND CMAKE_PREFIX_PATH "/usr/share/ompl")
endif()
find_package(OMPL CONFIG PATHS "/usr/share/ompl/cmake" NO_DEFAULT_PATH)

# Fallback if CMake package config isn't available at build time
if(NOT TARGET OMPL::ompl)
  find_library(OMPL_LIB NAMES ompl PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib NO_DEFAULT_PATH)
  if(OMPL_LIB)
    message(STATUS "Found OMPL library: ${OMPL_LIB}")
    # Add include directories where OMPL headers are typically installed on Debian/Ubuntu
    include_directories(
      "/usr/include/ompl"
      "/usr/include/ompl-1.5"
    )
  else()
    message(FATAL_ERROR "OMPL not found. Please install libompl-dev or set OMPL_DIR.")
  endif()
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(frontier_exploration src/frontier_exploration.cpp)
ament_target_dependencies(frontier_exploration rclcpp nav_msgs sensor_msgs geometry_msgs octomap_msgs)
ament_target_dependencies(frontier_exploration visualization_msgs)
target_link_libraries(frontier_exploration ${PCL_LIBRARIES} octomap)

add_executable(path_planner src/path_planner.cpp)
ament_target_dependencies(path_planner rclcpp nav_msgs geometry_msgs octomap_msgs)
if(TARGET OMPL::ompl)
  target_link_libraries(path_planner OMPL::ompl)
else()
  target_link_libraries(path_planner ${OMPL_LIB})
endif()
target_link_libraries(path_planner octomap)
if(TARGET fcl::fcl)
  target_link_libraries(path_planner fcl::fcl)
else()
  find_library(FCL_LIB NAMES fcl fclc)
  if(FCL_LIB)
    target_link_libraries(path_planner ${FCL_LIB})
  endif()
endif()
# Some FCL builds require linking against libccd
find_library(CCD_LIB NAMES ccd)
if(CCD_LIB)
  target_link_libraries(path_planner ${CCD_LIB})
endif()

add_executable(navigation_test src/navigation_test.cpp)
ament_target_dependencies(navigation_test rclcpp nav_msgs geometry_msgs)

install(TARGETS
  frontier_exploration
  path_planner
  navigation_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
