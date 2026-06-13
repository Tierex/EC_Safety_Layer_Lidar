# LiDAR Safety System (ROS2)

## 1. Overview

The LiDAR Safety System is a ROS2-based safety pipeline designed to evaluate collision risk for a low-speed autonomous vehicle using LiDAR-based detected and tracked objects.

### System goals

- 1:3 scale truck
- Maximum speed ≈ 5 km/h (≈ 1.39 m/s)
- LiDAR mounted on the roof
- LiDAR position: approx. 0.5 m behind the front bumper
- LiDAR height: approx. 1.8 m

### Safety output

- `0` = Emergency
- `1` = Hazard
- `2` = Free

### Pipeline features

- LiDAR point cloud ROI filtering
- Ground removal and voxel downsampling
- Euclidean clustering for object detection
- Object tracking with velocity estimation
- Corridor and side-zone safety filtering
- TTC-based emergency/hazard logic
- Vehicle-frame sensor offset correction
- Static or odometry-based ego speed input
- Optional control output and runtime monitoring

## 2. Current launch pipelines

### Full pipeline

Defined in `src/lidar_object_detection/launch/full_lidar_safety.launch.py`

Includes:

- `lidar_detector`
- `object_tracker`
- `ego_speed_constant_node` or `ego_speed_from_odom_node`
- `ego_yaw_from_imu_node` (optional)
- `safety_supervisor`
- `control_node`
- `safety_monitor_node`
- `lidar_subscriber` (optional debug node)

Launch arguments:

- `use_odom`: if true, run `ego_speed_from_odom_node`
- `use_imu`: if true, run `ego_yaw_from_imu_node`
- `use_lidar_subscriber`: if true, run `lidar_subscriber`

### Minimal pipeline

Defined in `src/lidar_object_detection/launch/minimal_lidar_safety.launch.py`

Includes:

- `lidar_detector`
- `object_tracker`
- `ego_speed_constant_node`
- `safety_supervisor`
- `control_node`

## 3. System node hierarchy

### Main pipeline

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
control_node
    |
    v
/cmd_vel
```

### Ego speed input

Option A — Constant speed:

```text
ego_speed_constant_node
    |
    v
/ego_speed
    |
    v
safety_supervisor
```

Option B — Odometry-based speed:

```text
/odom
    |
    v
ego_speed_from_odom_node
    |
    v
/ego_speed
    |
    v
safety_supervisor
```

### Optional yaw-rate input

```text
/imu/data
    |
    v
ego_yaw_from_imu_node
    |
    v
/ego_yaw_rate
```

### Optional monitoring

```text
/tracked_objects
/safety_signal
/ego_speed
    |
    v
safety_monitor_node
```

### Optional debug subscriber

```text
/velodyne_points
    |
    v
lidar_subscriber
```

## 4. Node overview

### 4.1 `lidar_detector`

- Executable: `lidar_detector`
- Source file: `lidar_person_detector_node.cpp`
- Input: `/velodyne_points` (`sensor_msgs/PointCloud2`)
- Output: `/detected_objects` (`std_msgs/Float32MultiArray`)
- Optional output: `/detected_object_markers` (`visualization_msgs/MarkerArray`)

Main tasks:

- ROI filtering
- Voxel downsampling
- Ground removal
- Euclidean clustering
- Bounding box extraction
- Publishes detected objects

Output format:

- 11 fields per detected object:

```text
[id, x, y, z, dx, dy, dz, num_points, source_time, detector_processing_ms, detector_frame]
```

### 4.2 `object_tracker`

- Executable: `object_tracker`
- Source file: `object_tracker.cpp`
- Input: `/detected_objects` (`std_msgs/Float32MultiArray`)
- Output: `/tracked_objects` (`std_msgs/Float32MultiArray`)
- Optional output: `/tracked_object_markers` (`visualization_msgs/MarkerArray`)

Main tasks:

- Matches detections across frames
- Assigns stable track IDs
- Estimates object velocity
- Predicts missed tracks
- Calculates closing speed
- Publishes tracked objects

Output format:

- 19 fields per tracked object:

```text
[id,
 x, y, z,
 vx, vy, vz,
 closing_speed,
 dx, dy, dz,
 distance_xy,
 num_points,
 hits,
 misses,
 age,
 track_timestamp_sec,
 tracker_processing_ms,
 detector_processing_ms]
```

### 4.3 `safety_supervisor`

- Executable: `safety_supervisor`
- Source file: `safety_supervisor.cpp`
- Inputs:
  - `/tracked_objects` (`std_msgs/Float32MultiArray`)
  - `/ego_speed` (`std_msgs/Float32`)
- Output: `/safety_signal` (`std_msgs/Int16`)

Output values:

- `0` = Emergency
- `1` = Hazard
- `2` = Free

Main tasks:

- Parses tracked objects
- Applies sensor offset correction for the vehicle frame
- Filters objects inside front and side safety zones
- Calculates TTC and distance-based risk
- Uses stale-timeout logic for ego speed
- Publishes the safety signal

> Note: the current codebase uses `/ego_speed` plus stale-timeout handling; it does not fall back to a separate `ego_speed_mps` parameter.

### 4.4 `ego_speed_constant_node`

- Executable: `ego_speed_constant_node`
- Source file: `ego_speed_constant_node.cpp`
- Output: `/ego_speed` (`std_msgs/Float32`)

Notes:

- Publishes a fixed speed by default: `1.3889 m/s`
- Default publish rate: `50 Hz`

### 4.5 `ego_speed_from_odom_node`

- Executable: `ego_speed_from_odom_node`
- Source file: `ego_speed_from_odom_node.cpp`
- Input: `/odom` (`nav_msgs/Odometry`)
- Output: `/ego_speed` (`std_msgs/Float32`)

Notes:

- Computes ego speed from odometry
- Can use longitudinal speed or full magnitude
- Applies smoothing and max-speed clamping

### 4.6 `ego_yaw_from_imu_node`

- Executable: `ego_yaw_from_imu_node`
- Source file: `ego_yaw_from_imu_node.cpp`
- Input: `/imu/data` (`sensor_msgs/Imu`)
- Output: `/ego_yaw_rate` (`std_msgs/Float32`)

Notes:

- Computes yaw rate from IMU angular velocity
- Smooths and clamps the output
- Optional in the launch pipeline

### 4.7 `control_node`

- Executable: `control_node`
- Source file: `control_node.cpp`
- Input: `/safety_signal` (`std_msgs/Int16`)
- Output: `/cmd_vel` (`geometry_msgs/Twist`)

Notes:

- Maps safety signals to target forward speeds
- Smooths speed changes using `max_accel_step_mps`

### 4.8 `safety_monitor_node`

- Executable: `safety_monitor_node`
- Source file: `safety_monitor_node.cpp`
- Inputs:
  - `/tracked_objects` (`std_msgs/Float32MultiArray`)
  - `/safety_signal` (`std_msgs/Int16`)
  - `/ego_speed` (`std_msgs/Float32`)

Notes:

- Prints runtime summaries of safety status, ego speed, track count, minimum distance, and TTC.
