cmake_minimum_required(VERSION 3.8)
project(lidar_object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# DEPENDENCIES
# ========================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# ========================
# EXECUTABLES
# ========================

add_executable(simple_node src/simple_node.cpp)
ament_target_dependencies(simple_node
  rclcpp
)

add_executable(lidar_subscriber src/lidar_subscriber.cpp)
ament_target_dependencies(lidar_subscriber
  rclcpp
  sensor_msgs
)

add_executable(lidar_detector src/lidar_person_detector_node.cpp)
ament_target_dependencies(lidar_detector
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
)

add_executable(object_tracker src/object_tracker.cpp)
ament_target_dependencies(object_tracker
  rclcpp
  std_msgs
  visualization_msgs
)

add_executable(safety_supervisor src/safety_supervisor.cpp)
ament_target_dependencies(safety_supervisor
  rclcpp
  std_msgs
)

add_executable(ego_speed_constant_node src/ego_speed_constant_node.cpp)
ament_target_dependencies(ego_speed_constant_node
  rclcpp
  std_msgs
)

add_executable(ego_speed_from_odom_node src/ego_speed_from_odom_node.cpp)
ament_target_dependencies(ego_speed_from_odom_node
  rclcpp
  std_msgs
  nav_msgs
)

add_executable(ego_yaw_from_imu_node src/ego_yaw_from_imu_node.cpp)
ament_target_dependencies(ego_yaw_from_imu_node
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(safety_monitor_node src/safety_monitor_node.cpp)
ament_target_dependencies(safety_monitor_node
  rclcpp
  std_msgs
)

# ========================
# INSTALL EXECUTABLES
# ========================

install(TARGETS
  simple_node
  lidar_subscriber
  lidar_detector
  object_tracker
  safety_supervisor
  ego_speed_constant_node
  ego_speed_from_odom_node
  ego_yaw_from_imu_node
  control_node
  safety_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# ========================
# INSTALL LAUNCH FILES
# ========================

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ========================
# TESTING
# ========================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
