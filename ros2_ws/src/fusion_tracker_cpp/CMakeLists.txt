cmake_minimum_required(VERSION 3.8)
project(fusion_tracker_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mosaic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(fusion_core
  src/hungarian_assigner.cpp
  src/ekf_tracker.cpp
)
target_link_libraries(fusion_core Eigen3::Eigen)

add_executable(fusion_tracker_node src/fusion_tracker_node.cpp)
target_link_libraries(fusion_tracker_node fusion_core)
ament_target_dependencies(fusion_tracker_node rclcpp mosaic_msgs)

install(TARGETS fusion_core fusion_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fusion test/test_fusion.cpp)
  target_link_libraries(test_fusion fusion_core Eigen3::Eigen)
endif()

ament_package()
