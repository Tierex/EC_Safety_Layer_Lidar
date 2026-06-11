# Lidar safety layer readme V1

This document provides the standard operating procedure for starting up the truck lidar safety layer.
Detailed description of the gateway with the truck hardware connection is given in the truck_gateway readme

---
## 0. Build the evirnment
Build and source the ros2 envirment of not already done so;

Open a terminal and navigate to your ROS 2 workspace:
```bash
cd ~/ros2_ws
```

Build the specific package and source the environment:
```bash
colcon build
source install/setup.bash
```

## 1. Connect Lidar

Connect the lidar to the PC on. Lidar config is available on `192.168.201`.
Make sure the wired connection config of the pc is set to `192.168.100` to receive the velodyne points.


## 2. Startup the lidar safety layer
All relevant launch files are located in the lidar_launch package

   ```bash
   cd ros2_ws
   ros2 launch lidar_launch lidar_safety_layer_launch.py
   ```
---
This should oven up RVIZ2 and show the pointcloud data.

## 3. Startup truck_gateway
NOTE: A full readme description on how to start the gateway is available in the gateway package.

Open a seperate terminal and navigate to the ros2 workspace
Run the launch file to start the truck gateway taking into account the safety layer:
```bash
ros2 launch truck_gateway truck_safe_gateway_launch.py
```
Or alternatively the gateway can be launched without the safety layer implementation:
```bash
ros2 launch truck_gateway truck_gateway_launch.py
```
---


## 4. simulation
Simulating poincloud data coming in from the LiDAR.
Open a rosbag;
```bash
ros2 bag play -l bags/rosbag2_2026_04_23-19_14_47
ros2 bag play -l bags/rosbag2_2026_05_21-14_54_18
ros2 bag play -l bags/rosbag2_2026_05_21-14_55_51
ros2 bag play -l bags/rosbag2_2026_05_21-15_02_25

ros2 bag play -l bags/rosbag2_2026_05_28_Angles
ros2 bag play -l bags/rosbag2_2026_05_28_Group_people
ros2 bag play -l bags/rosbag2_2026_05_28_Safety_Zone_Check
ros2 bag play -l bags/rosbag2_2026_05_28_Tim_Angle_R
ros2 bag play -l bags/rosbag2_2026_05_21-Tim_Straight

```
