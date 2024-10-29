cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_2dmap_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Link directories (if needed)
link_directories(${PCL_LIBRARY_DIRS})

# Define executable
add_executable(pointcloud_to_2dmap_node src/pointcloud_to_2dmap_node.cpp)

# Link libraries and set dependencies
target_link_libraries(pointcloud_to_2dmap_node ${PCL_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(pointcloud_to_2dmap_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install target
install(TARGETS pointcloud_to_2dmap_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories
install(DIRECTORY
  launch
  map
  DESTINATION share/${PROJECT_NAME}/
)

# ament package macros
ament_package()
