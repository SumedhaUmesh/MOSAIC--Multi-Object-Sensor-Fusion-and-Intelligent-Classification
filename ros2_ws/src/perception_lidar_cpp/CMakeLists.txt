cmake_minimum_required(VERSION 3.8)
project(perception_lidar_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mosaic_msgs REQUIRED)

add_executable(lidar_perception_node src/lidar_perception_node.cpp)
ament_target_dependencies(lidar_perception_node rclcpp sensor_msgs mosaic_msgs)

install(TARGETS lidar_perception_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lidar test/test_lidar.cpp)
  target_include_directories(test_lidar PRIVATE include)
endif()

ament_package()
