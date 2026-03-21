MULTI-SENSOR PERCEPTION FOR AUTONOMOUS ROBOTS USING ROS2

------------------------------------------------------------

OVERVIEW

This project implements a multi-sensor perception system using ROS2 (Jazzy) and Gazebo simulation. 
The system integrates Camera, LiDAR, and IMU data to create a unified perception pipeline.

The goal is to simulate a real robotic perception stack including:
- Sensor acquisition
- Synchronization
- Sensor fusion
- Latency measurement
- Modular architecture

------------------------------------------------------------

SYSTEM ARCHITECTURE

Gazebo Sensors (Camera / LiDAR / IMU)
        ↓
ROS2 Topics
        ↓
Sensor Nodes
        ↓
Synchronization (message_filters)
        ↓
Fusion Node
        ↓
Unified Output
        ↓
Latency Measurement

------------------------------------------------------------

NODES DESCRIPTION

Camera Node:
- Input: /camera/image_raw
- Output: /camera/perception
- Function: image processing and detection

LiDAR Node:
- Input: /scan
- Output: /lidar/perception
- Function: obstacle detection and distance calculation

IMU Node:
- Input: /imu
- Output: /imu/perception
- Function: orientation and motion tracking

Fusion Node:
- Input: camera + lidar + imu
- Output: /fusion/output
- Function: combines all sensor data

Latency Node:
- Measures system delays
- Output: /latency

------------------------------------------------------------

TOPICS

/camera/image_raw
/scan
/imu
/camera/perception
/lidar/perception
/imu/perception
/fusion/output
/latency

------------------------------------------------------------

IMPLEMENTATION STEPS

Step 1: Gazebo setup
- Robot with sensors created

Step 2: Camera node
- Image processing initialized

Step 3: LiDAR node
- Distance-based detection implemented

Step 4: IMU node
- Orientation tracking added

Step 5: Synchronization
- message_filters used to align sensor data

Step 6: Fusion
- Combined perception output created

Example:
Fusion: Object detected | Distance: 3.2 m

Step 7: Latency
- Measured delays across pipeline

Example:
Latency: Camera 12 ms | LiDAR 15 ms | Total 22 ms

Step 8: AI (optional)
- YOLO integration for smart detection

------------------------------------------------------------

HOW TO RUN

Build:
colcon build

Source:
source /opt/ros/jazzy/setup.bash
source install/setup.bash

Run simulation:
ros2 launch multi_sensor_perception simulation.launch.py

Run fusion:
ros2 run multi_sensor_perception fusion_node

------------------------------------------------------------

TESTING

Use keyboard control:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Observe:
- distance changes
- detection updates
- latency variation

------------------------------------------------------------

RESULTS

- System runs in real time
- Fusion improves detection reliability
- Synchronization ensures accuracy
- Latency remains acceptable

------------------------------------------------------------

FUTURE WORK

- AI detection (YOLO)
- Autonomous navigation
- Obstacle avoidance
- SLAM and mapping

------------------------------------------------------------

CONCLUSION

This project demonstrates a complete multi-sensor perception system using ROS2. 
The integration of Camera, LiDAR, and IMU with synchronization and fusion provides 
a strong foundation for autonomous robotics systems.

------------------------------------------------------------
