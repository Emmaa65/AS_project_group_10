cmake_minimum_required(VERSION 3.8)
project(object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_opencv REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(ObjectMappingToWorld 
                src/ObjectMappingToWorld.cpp
                src/ObjectManager.cpp
                )
ament_target_dependencies(ObjectMappingToWorld 
  rclcpp 
  geometry_msgs
  sensor_msgs
  image_geometry
  vision_opencv
  cv_bridge 
  message_filters
  tf2_ros
  tf2_geometry_msgs
  tf2
)#only for ros packages!

target_link_libraries(ObjectMappingToWorld
  ${OpenCV_LIBS}
) #system package

install(TARGETS ObjectMappingToWorld
  DESTINATION lib/${PROJECT_NAME})

  #for launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}) 



ament_package()
