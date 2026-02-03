cmake_minimum_required(VERSION 3.8)
project(inverse_mapping)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(custom_msgs REQUIRED)

add_executable(inverse_mapping_node src/inverse_mapping.cpp)

ament_target_dependencies(inverse_mapping_node
rclcpp
sensor_msgs
OpenCV
cv_bridge
custom_msgs
)

target_include_directories(inverse_mapping_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  inverse_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
