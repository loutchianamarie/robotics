# multi_sensor_perception

**Multi-Sensor Perception Architecture for Autonomous Robots** — ROS 2 package for integrating camera, LiDAR, and IMU data with optional fusion, timing analysis, and later perception stacks.

## Purpose

This package implements a modular perception pipeline: separate nodes for each sensor modality, a fusion node for combined state or features, and a latency node for evaluating timing. Step 1 establishes only the process architecture; sensor drivers, simulation, synchronization, and AI are added in later steps.

## Step 1 (current)

- Five standalone Python nodes, each inheriting from `rclpy.node.Node`.
- Each node logs a single startup message and spins with no publishers or subscribers.
- Intended as a clean skeleton for Gazebo topics, real sensors, time sync, fusion, and metrics.

## Node roles

| Node | Executable | Role |
|------|------------|------|
| Camera | `camera_node` | Future image / camera topics and preprocessing. |
| LiDAR | `lidar_node` | Future point cloud ingestion and filtering. |
| IMU | `imu_node` | Future inertial measurements and attitude helpers. |
| Fusion | `fusion_node` | Future multi-sensor fusion and unified outputs. |
| Latency | `latency_node` | Future end-to-end or per-stage latency evaluation. |

## Build

From the workspace root (`~/ros2_ws`):

```bash
cd ~/ros2_ws
colcon build --packages-select multi_sensor_perception
```

## Source the workspace

Every new terminal:

```bash
source ~/ros2_ws/install/setup.bash
```

## Run each node

In separate terminals (after sourcing), or stop one with Ctrl+C before starting another if you only need to smoke-test:

```bash
ros2 run multi_sensor_perception camera_node
ros2 run multi_sensor_perception lidar_node
ros2 run multi_sensor_perception imu_node
ros2 run multi_sensor_perception fusion_node
ros2 run multi_sensor_perception latency_node
```

You should see an `[INFO]` line indicating that the corresponding node started (Step 1 placeholder).

## Roadmap

See [PROJECT_STEPS.md](PROJECT_STEPS.md) for planned steps beyond Step 1.

## Step 2 — Gazebo Sim sensors (ros_gz)

Minimal **Gazebo Sim** world (`msp_empty.sdf`) and a box robot described in **`urdf/robot.urdf.xacro`** with three simulated sensors:

| Gazebo (transport) | ROS 2 topic | Message (ROS) |
|--------------------|-------------|----------------|
| `/camera` | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/scan` | `/scan` | `sensor_msgs/msg/LaserScan` |
| `/imu` | `/imu` | `sensor_msgs/msg/Imu` |

Bridging and sim clock use **`config/ros_gz_bridge.yaml`** and `ros_gz_bridge` `parameter_bridge`. Step 1 Python nodes are unchanged; they do not need to run for the simulation to publish these topics.

### Launch simulation

After build and `source ~/ros2_ws/install/setup.bash`:

```bash
ros2 launch multi_sensor_perception simulation.launch.py
```

Gazebo opens with the empty world; the robot is spawned after a short delay. Ensure simulation is **playing** (not paused) so sensor topics publish.

### Expected topics

- `/clock` (sim time, from bridge)
- `/camera/image_raw`
- `/scan`
- `/imu`
- `/robot_description`, `/tf`, `/tf_static` (from `robot_state_publisher` / sim)

### Check topics

```bash
ros2 topic list
ros2 topic echo /imu --once
ros2 topic hz /scan
ros2 topic info /camera/image_raw
```

### Dependencies

Install/build with ROS 2 Jazzy desktop or sim image so **`ros_gz_sim`** and **`ros_gz_bridge`** are available (`sudo apt install ros-jazzy-ros-gz` if needed).

## Step 3.1 — Camera topic subscription

`camera_node` subscribes to **`sensor_msgs/msg/Image`** on **`/camera/image_raw`**. Incoming images are counted; a short summary (frame id, stamp, size, encoding) is logged at most about **once every 2 seconds** so the console stays readable.

### Run simulation, then the camera node

Terminal 1 — Gazebo + bridge (from Step 2):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch multi_sensor_perception simulation.launch.py
```

Terminal 2 — camera node (use simulation time so stamps stay consistent with `/clock`):

```bash
source ~/ros2_ws/install/setup.bash
ros2 run multi_sensor_perception camera_node --ros-args -p use_sim_time:=true
```

You should see periodic log lines while the sim is **playing** and publishing images.

### Quick check without the node

```bash
ros2 topic hz /camera/image_raw
```

## Step 3.2 — LiDAR topic subscription

`lidar_node` subscribes to **`sensor_msgs/msg/LaserScan`** on **`/scan`**. Scans are counted; a short summary (angle limits, number of range readings) is logged at most about **once every 2 seconds**.

### Run simulation, then the LiDAR node

Terminal 1 — Gazebo + bridge (Step 2):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch multi_sensor_perception simulation.launch.py
```

Terminal 2 — LiDAR node (sim time):

```bash
source ~/ros2_ws/install/setup.bash
ros2 run multi_sensor_perception lidar_node --ros-args -p use_sim_time:=true
```

### Quick check without the node

```bash
ros2 topic hz /scan
```

## Step 3.3 — IMU topic subscription

`imu_node` subscribes to **`sensor_msgs/msg/Imu`** on **`/imu`**. Messages are counted; a compact line with orientation (quaternion), angular velocity, and linear acceleration is logged at most about **once every 2 seconds**.

### Run simulation, then the IMU node

Terminal 1 — Gazebo + bridge (Step 2):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch multi_sensor_perception simulation.launch.py
```

Terminal 2 — IMU node (sim time):

```bash
source ~/ros2_ws/install/setup.bash
ros2 run multi_sensor_perception imu_node --ros-args -p use_sim_time:=true
```

### Quick check without the node

```bash
ros2 topic hz /imu
```

## Step 4 — IMU node refinement and validation

Step 4 finalizes the IMU processing node as a stable runtime component:

- `imu_node` subscribes to `/imu` (`sensor_msgs/msg/Imu`).
- Orientation, angular velocity, and linear acceleration are logged in a compact format.
- Logging is throttled to keep terminal output readable during continuous simulation.
- The node is compatible with sim time (`use_sim_time:=true`) and real-time playback.

### Run Step 4 checks

Terminal 1 — simulation:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch multi_sensor_perception simulation.launch.py
```

Terminal 2 — IMU node:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run multi_sensor_perception imu_node --ros-args -p use_sim_time:=true
```

Optional quick validation:

```bash
ros2 topic echo /imu --once
```

## Step 5 — Synchronization, fusion, and latency

Step 5 introduces synchronized multi-sensor fusion with timing diagnostics for presentation and debugging.

### What is included

- **Synchronization (`message_filters`)**
  - `fusion_node` uses `ApproximateTimeSynchronizer` on:
    - `/scan` (`LaserScan`)
    - `/camera/image_raw` (`Image`)
    - `/imu` (`Imu`)
  - This ensures fusion decisions use near-simultaneous measurements.

- **Fusion logic**
  - LiDAR values are filtered to valid finite ranges in `[0.0, 7.0]` meters.
  - The node computes:
    - number of valid detections
    - closest detected distance
  - Console output:
    - `[Fusion] Objects: X | Closest: Xm | Sync delay: XX ms`
  - Decision states:
    - `WARNING: Obstacle too close` when closest `< 2.0 m`
    - `Object confirmed by synchronized sensors` when detections are valid and safe
    - `Clear path` when no valid object is found

- **Latency checks**
  - `fusion_node` logs total callback-side delay from message timestamps.
  - `latency_node` provides standalone monitoring of sensor timestamp-to-callback delay.
  - Console output:
    - `[Latency] Total: XX ms`

- **Basic visualization**
  - `fusion_node` publishes `visualization_msgs/msg/Marker` on `/visualization_marker`.
  - Detections are shown as spheres:
    - green for valid detections (`2.0 m` to `7.0 m`)
    - red for too-close detections (`< 2.0 m`)

### Run Step 5

Terminal 1 — simulation:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch multi_sensor_perception simulation.launch.py
```

Terminal 2 — fusion:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run multi_sensor_perception fusion_node --ros-args -p use_sim_time:=true
```

Terminal 3 — optional latency monitor:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run multi_sensor_perception latency_node --ros-args -p use_sim_time:=true
```

Optional RViz marker visualization:

```bash
rviz2
```

In RViz, add a **Marker** display and set topic to `/visualization_marker`.
