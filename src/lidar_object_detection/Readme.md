# LiDAR Object Detection

This ROS 2 package contains the custom LiDAR object detection, object tracking, safety supervision and monitoring nodes used for the scaled automated truck safety-layer project. The package receives Velodyne VLP-16 pointcloud data, extracts obstacle candidates, tracks these objects over time and converts the tracked objects into a discrete safety signal.

The package is part of the repository:

```text
EC_Safety_Layer_Lidar
└── src
    └── lidar_object_detection
```

The implementation is intended as a lightweight and transparent safety layer for low-speed prototype testing. It is not a full autonomous-driving stack and it does not perform semantic object classification.

## Pipeline overview

```text
/velodyne_points
      |
      v
lidar_detector
      |
      v
/detected_objects
      |
      v
object_tracker
      |
      v
/tracked_objects
      |
      v
safety_supervisor
      |
      v
/safety_signal
      |
      v
control_node or truck_gateway
```

The detector processes the raw pointcloud. The tracker associates detections over time and estimates object motion. The safety supervisor evaluates the tracked objects relative to the configured emergency and hazard zones. The result is published as a simple integer safety signal.

## Safety signal

| Signal | Meaning | Intended behaviour |
|---:|---|---|
| 0 | Emergency | Stop or block unsafe motion |
| 1 | Hazard | Restrict motion or reduce speed |
| 2 | Free | Normal motion allowed |

The signal is deliberately simple so that it can be used by different control layers, such as the manual truck gateway or a future MPC-based controller.

## Main nodes

### `lidar_detector`

Source file:

```text
src/lidar_person_detector_node.cpp
```

Subscribes to:

```text
/velodyne_points        sensor_msgs/msg/PointCloud2
```

Publishes:

```text
/detected_objects       std_msgs/msg/Float32MultiArray
/detected_object_markers visualization_msgs/msg/MarkerArray
```

Main function:

- filters the raw pointcloud with a region of interest;
- downsamples the pointcloud with a voxel grid;
- removes the ground plane using RANSAC-based plane segmentation;
- clusters remaining points using Euclidean clustering;
- calculates bounding boxes for accepted clusters;
- publishes detected objects for the tracker;
- optionally publishes RViz markers.

The detector publishes 11 fields per detected object:

```text
[id, x, y, z, dx, dy, dz, num_points, source_time_sec, detector_processing_ms, detector_frame]
```

### `object_tracker`

Source file:

```text
src/object_tracker.cpp
```

Subscribes to:

```text
/detected_objects       std_msgs/msg/Float32MultiArray
```

Publishes:

```text
/tracked_objects        std_msgs/msg/Float32MultiArray
/tracked_object_markers visualization_msgs/msg/MarkerArray
```

Main function:

- parses detected objects from the detector output;
- associates detections with existing tracks;
- assigns stable track IDs;
- estimates object velocity;
- keeps tracks alive for a configurable number of missed frames;
- filters tracks using a minimum hit count;
- calculates horizontal distance and closing speed;
- publishes tracked objects for the safety supervisor.

The tracker publishes 19 fields per tracked object:

```text
[id, x, y, z,
 vx, vy, vz,
 closing_speed,
 dx, dy, dz,
 distance_xy,
 num_points,
 hits, misses, age,
 track_timestamp_sec,
 tracker_processing_ms,
 detector_processing_ms]
```

### `safety_supervisor`

Source file:

```text
src/safety_supervisor.cpp
```

Subscribes to:

```text
/tracked_objects
/ego_speed
```

Publishes:

```text
/safety_signal          std_msgs/msg/Int16
/safety_zones_array     visualization_msgs/msg/MarkerArray
```

Main function:

- converts tracked-object positions from the LiDAR frame to the vehicle safety frame;
- evaluates objects inside front emergency, front hazard and side hazard zones;
- publishes safety signal `0`, `1` or `2`;
- applies stale-data timeout behaviour;
- applies emergency and hazard hold times;
- publishes RViz markers for the safety zones.

### `ego_speed_constant_node`

Publishes a fixed ego speed to `/ego_speed`. This is useful when odometry is not available.

### `ego_speed_from_odom_node`

Reads `/odom` and publishes estimated vehicle speed to `/ego_speed`.

### `ego_yaw_from_imu_node`

Reads `/imu/data` and publishes yaw rate to `/ego_yaw_rate`. This node is available for future extensions.

### `control_node`

Subscribes to `/safety_signal` and publishes `/cmd_vel`. It converts the safety state into a simple velocity command:

- free: normal speed;
- hazard: reduced speed;
- emergency: zero speed.

For the physical truck, the `truck_gateway` package can be used to combine the safety signal with manual controller input.

### `safety_monitor_node`

Prints runtime information from tracked objects, ego speed and safety signal.

### `safety_performance_monitor`

Records pipeline performance metrics such as topic frequency, processing time, safety-state counts, transitions, command behaviour and CPU usage. This node is not included in the default launch files and can be started manually when performance logging is needed.

## Package structure

```text
lidar_object_detection
├── CMakeLists.txt
├── package.xml
├── Readme.md
├── config
│   └── lidar_pipeline_tuning.yaml
├── launch
│   ├── full_lidar_safety.launch.py
│   └── minimal_lidar_safety.launch.py
└── src
    ├── control_node.cpp
    ├── ego_speed_constant_node.cpp
    ├── ego_speed_from_odom_node.cpp
    ├── ego_yaw_from_imu_node.cpp
    ├── lidar_person_detector_node.cpp
    ├── lidar_subscriber.cpp
    ├── object_tracker.cpp
    ├── safety_monitor_node.cpp
    ├── safety_performance_monitor.cpp
    ├── safety_supervisor.cpp
    └── simple_node.cpp
```

## Dependencies

This package was developed for:

```text
Ubuntu 22.04
ROS 2 Humble
C++
PCL
RViz2
Velodyne VLP-16 pointcloud input
```

ROS 2 package dependencies:

```text
rclcpp
sensor_msgs
std_msgs
visualization_msgs
geometry_msgs
nav_msgs
pcl_conversions
pcl_ros
```

## Build instructions

Place the repository in a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone or copy the repository into `~/ros2_ws/src`. Then build:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_object_detection --symlink-install
source install/setup.bash
```

To build the complete repository workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Running the object-detection pipeline

The package expects pointcloud data on:

```text
/velodyne_points
```

This can come from the Velodyne driver or from a rosbag.

### Full package launch

```bash
ros2 launch lidar_object_detection full_lidar_safety.launch.py
```

Optional launch arguments:

```bash
ros2 launch lidar_object_detection full_lidar_safety.launch.py use_odom:=true
ros2 launch lidar_object_detection full_lidar_safety.launch.py use_imu:=true
ros2 launch lidar_object_detection full_lidar_safety.launch.py use_lidar_subscriber:=true
```

When `use_odom:=false`, the package uses `ego_speed_constant_node`. When `use_odom:=true`, it expects an `/odom` topic.

### Launch from `lidar_launch`

The repository also contains the `lidar_launch` package. This launch file starts the Velodyne driver if available, starts the object-detection pipeline and opens RViz2:

```bash
ros2 launch lidar_launch lidar_safety_layer_launch.py
```

## Important note about the minimal launch file

In the current repository, `minimal_lidar_safety.launch.py` refers to:

```text
config/safety_supervisor.yaml
```

However, the available configuration file in this package is:

```text
config/lidar_pipeline_tuning.yaml
```

Before using `minimal_lidar_safety.launch.py`, either rename/copy the configuration file or update the launch file to use `lidar_pipeline_tuning.yaml`.

## Running with rosbag data

Terminal 1:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 bag play -l <bag_folder>
```

Terminal 2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch lidar_object_detection full_lidar_safety.launch.py
```

Useful checks:

```bash
ros2 topic list
ros2 topic hz /velodyne_points
ros2 topic hz /detected_objects
ros2 topic hz /tracked_objects
ros2 topic echo /safety_signal
```

## RViz2 visualisation

The following marker topics can be visualised in RViz2:

```text
/detected_object_markers
/tracked_object_markers
/safety_zones_array
```

These markers are used to check whether the pointcloud, detected objects, tracked objects and safety zones are aligned correctly.

## Coordinate frames

The detector and tracker operate in the LiDAR frame. The safety supervisor converts the object position to a simplified vehicle safety frame.

In the vehicle safety frame:

```text
x = 0 is located at the front of the truck
x points forward
y points sideways
z = 0 is located at ground level
```

The LiDAR is mounted behind the front bumper and above the ground. The current supervisor defaults are:

```text
sensor_offset_x = -0.57 m
sensor_offset_y =  0.00 m
sensor_offset_z =  1.80 m
```

This offset allows the supervisor to evaluate safety distances relative to the front of the truck instead of relative to the LiDAR sensor.

## Important parameters

### Detector parameters

| Parameter | Meaning |
|---|---|
| `roi_x_min`, `roi_x_max` | Region-of-interest limits in x-direction |
| `roi_y_min`, `roi_y_max` | Region-of-interest limits in y-direction |
| `roi_z_min`, `roi_z_max` | Region-of-interest limits in z-direction |
| `voxel_size` | Voxel-grid leaf size |
| `use_ground_removal` | Enables or disables ground removal |
| `ground_max_distance` | RANSAC ground-plane distance threshold |
| `ground_max_angle_deg` | Maximum ground-plane angle relative to z-axis |
| `cluster_tolerance` | Maximum point distance inside one Euclidean cluster |
| `min_cluster_size`, `max_cluster_size` | Minimum and maximum number of points per cluster |
| `min_x_size`, `min_y_size`, `min_z_size` | Minimum accepted bounding-box dimensions |
| `max_x_size`, `max_y_size`, `max_z_size` | Maximum accepted bounding-box dimensions |

### Tracker parameters

| Parameter | Meaning |
|---|---|
| `max_match_distance` | Maximum xy-distance for detection-to-track association |
| `max_z_match_distance` | Maximum z-distance for association |
| `max_size_change_ratio` | Maximum accepted ratio between previous and new object dimensions |
| `max_missed_frames` | Number of missed frames before deleting a track |
| `min_hits_to_publish` | Minimum number of hits before a track is published |
| `velocity_alpha` | Velocity smoothing factor |
| `max_velocity` | Maximum allowed estimated velocity |
| `predict_missed_tracks` | Enables track prediction during missed detections |

### Safety-supervisor parameters

| Parameter | Meaning |
|---|---|
| `publish_rate_hz` | Safety-signal publication rate |
| `stale_timeout_sec` | Timeout for stale tracked-object data |
| `stale_signal` | Safety signal used when object data is stale |
| `sensor_offset_x`, `sensor_offset_y`, `sensor_offset_z` | LiDAR position relative to the vehicle safety frame |
| `emergency_distance` | Radius of the front emergency zone |
| `hazard_distance` | Radius of the front hazard zone |
| `enable_side_zones` | Enables side hazard zones |
| `side_zone_length` | Side-zone length |
| `side_zone_width` | Side-zone width |
| `side_zone_offset_y` | Lateral position of side zones |
| `emergency_hold_sec` | Minimum emergency hold time |
| `hazard_hold_sec` | Minimum hazard hold time |
| `lidar_down_angle_deg`, `lidar_up_angle_deg` | Vertical visibility angles for the VLP-16 model |

## Fail-safe behaviour

The safety supervisor contains stale-data handling. If no recent `/tracked_objects` data is received within the configured timeout, it publishes the configured stale signal. For safety testing this is normally:

```text
stale_signal = 0
```

This means that missing or outdated perception data results in an emergency signal.

The truck gateway also contains command-timeout behaviour. If recent command messages are no longer received, the outgoing command is set to a safe value.

## Limitations

This package is a prototype for low-speed testing. Important limitations are:

- the system detects obstacle candidates but does not semantically classify object type;
- the focus of the tests was mainly pedestrian-related safety behaviour;
- the VLP-16 has limited vertical resolution compared with higher-channel LiDAR sensors;
- very close objects may not be detected reliably due to the roof-mounted LiDAR position and vertical field-of-view limitations;
- classical pointcloud clustering is sensitive to sparse points, occlusion, object shape and ground-removal settings;
- the full physical braking response is not measured by this package alone;
- trailer swept-path behaviour requires additional validation.

## License

See the repository license file.
