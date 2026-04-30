cmake_minimum_required(VERSION 3.8)
project(perception_lidar_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mosaic_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)

add_executable(lidar_perception_node src/lidar_perception_node.cpp)
target_include_directories(lidar_perception_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar_perception_node ${PCL_LIBRARIES})
ament_target_dependencies(lidar_perception_node rclcpp sensor_msgs mosaic_msgs pcl_conversions)

install(TARGETS lidar_perception_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lidar test/test_lidar.cpp)
  target_include_directories(test_lidar PRIVATE include)
endif()

ament_package()
