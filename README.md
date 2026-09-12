# Multi-Sensor Perception for Autonomous Robots

A Python perception pipeline that brings camera, LiDAR, and IMU measurements together in ROS 2 and Gazebo. The project explores sensor synchronization, optional visual object detection, and timing diagnostics in a simulated robot environment.

**Focus:** robotics perception · Python · ROS 2 Jazzy · Gazebo Sim · sensor integration · optional YOLO inference

## Problem and approach

Robot sensors produce different kinds of measurements at different times. This project uses a modular ROS 2 architecture to acquire those measurements, align their timestamps, and publish a combined perception summary.

The implementation pairs the nearest valid LiDAR range with IMU orientation and recent camera information. It provides a practical foundation for studying perception pipelines before adding navigation or more advanced fusion.

## Implementation highlights

| Component | What it does | Source |
|---|---|---|
| Simulation | Spawns a sensor-equipped robot and bridges Gazebo messages into ROS 2 | [Launch configuration](ros2_ws/src/multi_sensor_perception/launch/simulation.launch.py) |
| Camera | Publishes camera information; optionally runs YOLO and selects the highest-confidence qualifying detection | [Camera node](ros2_ws/src/multi_sensor_perception/multi_sensor_perception/camera_node.py) |
| LiDAR | Processes laser scans in a dedicated node | [LiDAR node](ros2_ws/src/multi_sensor_perception/multi_sensor_perception/lidar_node.py) |
| IMU | Subscribes to orientation, angular velocity, and acceleration measurements | [IMU node](ros2_ws/src/multi_sensor_perception/multi_sensor_perception/imu_node.py) |
| Fusion | Synchronizes raw sensor messages, filters LiDAR ranges, computes yaw, and adds recent camera information | [Fusion node](ros2_ws/src/multi_sensor_perception/multi_sensor_perception/fusion_node.py) |
| Timing | Measures sensor timestamp-to-callback delays and tracks a rolling timing average | [Latency node](ros2_ws/src/multi_sensor_perception/multi_sensor_perception/latency_node.py) |

## Architecture

Gazebo publishes camera, laser-scan, and IMU messages through `ros_gz_bridge`. Dedicated sensor nodes consume these topics. The fusion and latency nodes independently synchronize the raw sensor streams.

| Topic | Purpose |
|---|---|
| `/camera/image_raw` | Raw camera images |
| `/scan` | LiDAR scans |
| `/imu` | Inertial measurements |
| `/camera/perception` | Camera label, confidence, bounding box, and processing mode |
| `/fusion/output` | Combined distance, orientation, camera information, and timestamp diagnostics |
| `/latency` | Per-sensor timestamp-to-callback timing |

The fusion node uses `ApproximateTimeSynchronizer` with a 150 ms tolerance. It calculates the closest finite LiDAR reading within 7 m and converts IMU orientation from quaternion to yaw. Camera results are included when their timestamp is within 300 ms of the synchronized image.

## Run the simulation

Prerequisites: a ROS 2 Jazzy environment, Gazebo integration through `ros_gz`, `colcon`, and the dependencies declared in [package.xml](ros2_ws/src/multi_sensor_perception/package.xml).

The commands below use the implementation under **`ros2_ws/src/`**. A second, earlier package copy exists under the repository-level `src/`; build from `ros2_ws` to avoid discovering both packages.

```bash
git clone https://github.com/loutchianamarie/robotics.git
cd robotics/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select multi_sensor_perception
source install/setup.bash
ros2 launch multi_sensor_perception simulation.launch.py
```

Start the following nodes in separate terminals. In each terminal, first enter `robotics/ros2_ws` and source both ROS 2 and `install/setup.bash`.

```bash
ros2 run multi_sensor_perception camera_node --ros-args -p use_sim_time:=true -p enable_ai:=false
ros2 run multi_sensor_perception fusion_node --ros-args -p use_sim_time:=true
ros2 run multi_sensor_perception latency_node --ros-args -p use_sim_time:=true
```

The simulation launcher starts Gazebo, the bridge, and robot spawning. It does not start the perception nodes. The separate LiDAR and IMU nodes can also be run for sensor-specific inspection.

## Optional YOLO detection

The camera node supports Ultralytics YOLO through `cv_bridge`, with `yolov8n.pt` as its default model and a confidence threshold of 0.5. Install compatible dependencies and make the weights available in the Python environment used by ROS 2, then enable AI:

```bash
ros2 run multi_sensor_perception camera_node --ros-args -p use_sim_time:=true -p enable_ai:=true
```

When the model or dependencies are unavailable, the node falls back to an `image_present` signal. That signal confirms image availability; it is not an object-detection result.

## Inspect the output

With Gazebo playing and the nodes running:

```bash
ros2 topic hz /scan
ros2 topic echo /fusion/output --once
ros2 topic echo /latency --once
```

The fusion output includes `object_detected`, `distance_m`, `orientation_yaw_rad`, `sync_delay_ms`, and camera label/confidence/mode fields.

## Scope and evaluation

This is a simulation-based perception prototype. It combines sensor information without calibrated camera-to-LiDAR object association or a probabilistic state estimator. The obstacle flag is driven by LiDAR; it does not prove that a camera label corresponds to the closest range.

The latency monitor measures delays at its own synchronized callback. Its total is a diagnostic proxy, not a measurement of the complete camera-inference-to-fusion path. Reproducible benchmark results and a recorded demonstration are still needed before making quantitative performance or accuracy claims.

## Next steps

- Record a simulation walkthrough showing sensor inputs and combined output.
- Compare AI-enabled and fallback runs using recorded timing data.
- Consolidate the two package copies after checking which work must be preserved.
- Add camera–LiDAR calibration and object association.
- Extend evaluation to navigation and obstacle avoidance.

## Skills demonstrated

ROS 2 publish/subscribe architecture, Python node development, asynchronous sensor integration, timestamp synchronization, optional computer-vision inference, and performance instrumentation.
