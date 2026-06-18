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


## YAML configuration

The package contains the following configuration file:

```text
src/lidar_object_detection/config/lidar_pipeline_tuning.yaml
```

In the current repository this YAML file mainly configures the `safety_supervisor` node. The detector and tracker parameters are declared in their C++ nodes and can also be overridden from a launch file if needed. The full launch file currently passes several parameters inline, so make sure that the launch file and this YAML file use the same values when tuning the system.

Current YAML configuration:

```yaml
safety_supervisor:
  ros__parameters:

    # =========================================================
    # Topics
    # =========================================================
    input_topic: /tracked_objects
    output_topic: /safety_signal
    ego_speed_topic: /ego_speed

    # =========================================================
    # QoS / timing
    # =========================================================
    qos_depth: 20
    publish_rate_hz: 20.0

    # =========================================================
    # Fail-safe
    # =========================================================
    stale_timeout_sec: 0.5
    stale_signal: 0
    ego_speed_stale_timeout_sec: 1.0

    # =========================================================
    # Sensor offset: LiDAR t.o.v. safety_base
    # safety_base: x=0 voorkant truck, y=0 middenlijn, z=0 grond
    # =========================================================
    sensor_offset_x: -0.57
    sensor_offset_y: 0.0
    sensor_offset_z: 1.8

    # =========================================================
    # Front/cab zones
    # Emergency: 0.0 -> 1.5 m
    # Hazard:    1.5 -> 2.5 m
    # =========================================================
    emergency_distance: 1.5
    hazard_distance: 2.5

    # =========================================================
    # Side/trailer zones
    # Side-zone hysteresis. Side zones are HAZARD only, never emergency
    side_zone_hysteresis_x: 0.20
    side_zone_hysteresis_y: 0.10
    min_side_approach_speed: 0.10

    # =========================================================
    # Debug
    # =========================================================
    debug: true
    debug_every_n_frames: 1
    debug_timer: true
    # =========================================================
    enable_side_zones: true
    side_zone_length: 6.0
    side_zone_width: 0.6
    side_zone_offset_y: 0.5

    # =========================================================
    # Track filtering
    # =========================================================
    min_track_hits: 1
    max_track_misses: 2

    # =========================================================
    # Hold times
    # =========================================================
    emergency_hold_sec: 0.30
    hazard_hold_sec: 0.50

    # =========================================================
    # VLP-16 vertical FOV
    # =========================================================
    zone_height: 3.0
    lidar_down_angle_deg: 15.0
    lidar_up_angle_deg: 15.0

    # =========================================================
    # RViz markers
    # =========================================================
    marker_topic: /safety_zones_array
    marker_frame_id: safety_base
    sector_azimuth_steps: 120

    # =========================================================
    # Compatibility parameters
    # These remain here so older launch/config does not break.
    # They are not used as extra triggers in this version.
    # =========================================================
    corridor_half_width: 1.0
    min_x_consider: 0.0
    max_x_consider: 20.0

    enable_ttc: true
    emergency_ttc: 0.6
    hazard_ttc: 1.5
    min_closing_speed: 0.10

    enable_cut_in_prediction: true
    hazard_prediction_horizon: 1.5
    emergency_prediction_horizon: 0.5
    min_lateral_speed: 0.10

    enable_brake_model: true
    ego_speed_mps: 1.3889
    max_decel_mps2: 1.5
    system_delay_sec: 0.30
    emergency_margin_m: 0.10
    hazard_margin_m: 0.25

    emergency_distance_hysteresis: 0.30
    hazard_distance_hysteresis: 0.30
    emergency_ttc_hysteresis: 0.20
```

### Parameter choices and tuning rationale

The parameters were chosen for low-speed testing of the 1:3 scaled truck with a roof-mounted Velodyne VLP-16. The main goal was to obtain stable pedestrian/obstacle detection and a conservative safety signal, rather than maximum object-classification accuracy.

#### Detector tuning choices

| Parameter | Current value | Reason for this choice | Effect if changed |
|---|---:|---|---|
| `roi_x_min` / `roi_x_max` | -4.0 / 4.0 m | Limits processing to the relevant area around the truck | A larger ROI increases processing load; a smaller ROI may remove relevant obstacles |
| `roi_y_min` / `roi_y_max` | -3.0 / 3.0 m | Covers the area in front and beside the truck during low-speed tests | Too narrow may miss side objects; too wide may include irrelevant objects |
| `roi_z_min` / `roi_z_max` | -3.0 / 3.0 m | Keeps enough vertical range for ground and pedestrian points | Too narrow may remove useful points before clustering |
| `voxel_size` | 0.05 m | Reduces point count while preserving pedestrian shape | Larger is faster but less detailed; smaller is more detailed but slower |
| `use_ground_removal` | true | Prevents ground points from becoming obstacle clusters | Disabling it can create false clusters from the floor |
| `ground_max_distance` | 0.10 m | Allows points close to the fitted plane to be removed as ground | Too high may remove low obstacle points; too low may leave ground points |
| `ground_max_angle_deg` | 15.0 deg | Keeps the ground plane close to the expected floor orientation | Higher values allow tilted planes; lower values are stricter |
| `cluster_tolerance` | 0.25 m | Groups nearby LiDAR points into one object candidate | Too small splits one person; too large merges nearby objects |
| `min_cluster_size` / `max_cluster_size` | 8 / 2000 points | Removes noise clusters and unrealistically large clusters | Lower minimum increases noise; lower maximum may remove large objects |
| `min_z_size` / `max_z_size` | 0.30 / 2.00 m | Accepts pedestrian-sized vertical objects | Too strict may reject valid people; too loose accepts more false objects |

#### Tracker tuning choices

| Parameter | Current value | Reason for this choice | Effect if changed |
|---|---:|---|---|
| `max_match_distance` | 1.0 m | Allows detections to be associated between frames at low LiDAR frequency | Smaller is stricter but may lose tracks; larger may cause wrong matches |
| `max_z_match_distance` | 1.0 m | Allows vertical variation in sparse VLP-16 detections | Smaller may lose tracks due to height variation |
| `max_size_change_ratio` | 2.5 | Prevents very different clusters from being matched to the same track | Lower is stricter; higher can associate unrelated objects |
| `max_missed_frames` | 5 | Keeps a track alive during short missed detections | Higher improves continuity but may keep stale tracks too long |
| `min_hits_to_publish` | 1 | Publishes tracks quickly for reactive safety behaviour | Higher reduces noise but delays first detection |
| `velocity_alpha` | 0.35 | Smooths velocity without making response too slow | Higher responds faster but noisier; lower is smoother but slower |
| `predict_missed_tracks` | true | Predicts object position when one frame is missed | Helps continuity but can drift if detections disappear for too long |

#### Safety-supervisor tuning choices

| Parameter | Current value | Reason for this choice | Effect if changed |
|---|---:|---|---|
| `publish_rate_hz` | 20.0 Hz | Provides a stable control-relevant safety output | Higher rate gives faster repeated output but more CPU load |
| `stale_timeout_sec` | 0.5 s | Treats missing tracked-object data as unsafe after a short timeout | Shorter reacts faster to data loss; longer avoids false emergency on small gaps |
| `stale_signal` | 0 | Fail-safe behaviour: stale perception data results in emergency | Setting this to 2 would be unsafe for safety-critical tests |
| `sensor_offset_x` | -0.57 m | LiDAR is approximately 0.57 m behind the front bumper | Wrong value shifts all safety distances |
| `sensor_offset_z` | 1.8 m | LiDAR height above the ground | Used by the VLP-16 visibility model |
| `emergency_distance` | 1.5 m | Defines the front emergency region | Larger is more conservative; smaller allows closer approach |
| `hazard_distance` | 2.5 m | Defines the front hazard region | Larger gives earlier warning/restriction |
| `enable_side_zones` | true | Enables hazard zones beside the truck/trailer area | Useful for near-side awareness, but not an emergency trigger |
| `side_zone_length` | 6.0 m | Covers the side area of the truck/trailer setup | Should match the vehicle geometry used in testing |
| `side_zone_width` | 0.6 m | Defines the lateral width of each side hazard zone | Wider detects more side objects but may trigger more hazards |
| `side_zone_offset_y` | 0.5 m | Places the side zones away from the vehicle centreline | Should be tuned to truck width and RViz alignment |
| `min_track_hits` | 1 | Allows immediate safety reaction to a new valid track | Higher values reduce false positives but delay reaction |
| `max_track_misses` | 2 | Rejects tracks that have recently disappeared | Higher values are more tolerant but can keep stale objects |
| `emergency_hold_sec` | 0.30 s | Prevents emergency signal from flickering | Longer holds are more stable but return to free more slowly |
| `hazard_hold_sec` | 0.50 s | Prevents hazard signal from flickering | Longer holds reduce switching near zone boundaries |
| `lidar_down_angle_deg` / `lidar_up_angle_deg` | 15.0 / 15.0 deg | Approximates the vertical visibility of the VLP-16 | Used to model near-field visibility limitations |

### Recommended tuning workflow

1. Start with RViz2 and verify that `/velodyne_points` is correctly aligned with the LiDAR frame.
2. Tune the ROI so only the relevant area around the truck is processed.
3. Tune `voxel_size`, `ground_max_distance` and `cluster_tolerance` until pedestrians form stable clusters.
4. Tune tracker parameters only after detection is stable.
5. Tune safety-zone distances and side-zone dimensions using the RViz2 safety-zone markers.
6. Verify that `/safety_signal` changes correctly for known object positions.
7. Test stale-data behaviour by stopping `/tracked_objects` and checking that the supervisor publishes the configured fail-safe signal.

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
