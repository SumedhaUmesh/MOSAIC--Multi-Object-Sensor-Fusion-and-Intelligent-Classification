cmake_minimum_required(VERSION 3.8)
project(adas_app_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mosaic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(adas_app_node src/adas_app_node.cpp)
ament_target_dependencies(adas_app_node rclcpp mosaic_msgs std_msgs)

install(TARGETS adas_app_node DESTINATION lib/${PROJECT_NAME})

ament_package()
