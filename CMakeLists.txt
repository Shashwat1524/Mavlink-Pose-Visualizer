cmake_minimum_required(VERSION 3.8)
project(px4_visualize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)



add_executable(sensor_combined_node src/sensor_combined.cpp)
ament_target_dependencies(sensor_combined_node rclcpp px4_msgs sensor_msgs)

add_executable(visualize_node src/visualize.cpp)
ament_target_dependencies(visualize_node rclcpp px4_msgs geometry_msgs )

install(TARGETS 
  sensor_combined_node
  visualize_node
  DESTINATION lib/${PROJECT_NAME}    
)
ament_package()
