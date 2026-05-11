cmake_minimum_required(VERSION 3.5)
project(multiple_object_tracking_lidar)

# Default to C++17 (Better for ROS 2 Humble)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED) # OpenCV found here

# 2. Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)

# 3. Define the Library
add_library(multiple_object_tracking_lidar SHARED
  src/multiple_object_tracking_lidar.cpp)

# Link dependencies to the library (Crucial for KalmanFilter)
target_link_libraries(multiple_object_tracking_lidar
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(multiple_object_tracking_lidar
    "rclcpp"
    "pcl_ros"
    "sensor_msgs"
    "cv_bridge"
    "visualization_msgs" 
)

# 4. Define the Executable (Node)
add_executable(multiple_object_tracking_lidar_node
    src/multiple_object_tracking_lidar_node.cpp)

# Link the internal library AND OpenCV to the node
target_link_libraries(multiple_object_tracking_lidar_node
    multiple_object_tracking_lidar
    ${OpenCV_LIBRARIES} 
)

# Required for components
rclcpp_components_register_nodes(multiple_object_tracking_lidar "multiple_object_tracking_lidar::MultipleObjectTrackingLidar")

# 5. Install logic
install(DIRECTORY include/ DESTINATION include)

install(TARGETS
    multiple_object_tracking_lidar
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    multiple_object_tracking_lidar_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()