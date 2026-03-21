# multi_sensor_perception

Multi-sensor ROS 2 Jazzy package for camera, LiDAR, and IMU simulation + perception node scaffolding.

## What this project does

This package builds a perception pipeline in clear steps:

1. Create modular ROS 2 nodes (`camera_node`, `lidar_node`, `imu_node`, `fusion_node`, `latency_node`).
2. Launch Gazebo Sim with a robot that publishes camera, LiDAR, and IMU data.
3. Subscribe to the three sensor topics in Python nodes.
4. Add useful processing logic, including LiDAR obstacle filtering between **6.0 m and 7.0 m**.

The package is compatible with **ROS 2 Jazzy** and uses `rclpy` nodes and `ros_gz` bridge tooling.

## Package structure

- `multi_sensor_perception/camera_node.py`: subscribes to `/camera/image_raw`.
- `multi_sensor_perception/lidar_node.py`: subscribes to `/scan` and filters obstacle ranges.
- `multi_sensor_perception/imu_node.py`: subscribes to `/imu`.
- `multi_sensor_perception/fusion_node.py`: placeholder for future sensor fusion logic.
- `multi_sensor_perception/latency_node.py`: placeholder for future latency metrics.
- `launch/simulation.launch.py`: starts Gazebo, bridge, robot state publisher, and robot spawn.
- `config/ros_gz_bridge.yaml`: Gazebo <-> ROS 2 topic mappings.
- `urdf/robot.urdf.xacro`: robot model with simulated sensors.
- `worlds/msp_empty.sdf`: simulation world.

## Completed steps

## Step 1 - Node architecture

- Implemented 5 standalone ROS 2 Python nodes.
- Added console entry points in `setup.py`.
- Each node starts, logs status, and spins.

### Entry points (`setup.py`)

The package exposes:

- `camera_node = multi_sensor_perception.camera_node:main`
- `lidar_node = multi_sensor_perception.lidar_node:main`
- `imu_node = multi_sensor_perception.imu_node:main`
- `fusion_node = multi_sensor_perception.fusion_node:main`
- `latency_node = multi_sensor_perception.latency_node:main`

## Step 2 - Gazebo simulation + sensor bridging

`simulation.launch.py` does the following:

1. Loads world: `worlds/msp_empty.sdf`
2. Builds robot description from `urdf/robot.urdf.xacro`
3. Starts `robot_state_publisher`
4. Launches Gazebo via `ros_gz_sim`
5. Starts `ros_gz_bridge parameter_bridge` using `config/ros_gz_bridge.yaml`
6. Spawns robot into Gazebo after a short delay

Expected ROS topics in simulation:

- `/clock`
- `/camera/image_raw` (`sensor_msgs/msg/Image`)
- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/imu` (`sensor_msgs/msg/Imu`)
- `/tf`, `/tf_static`, `/robot_description`

## Step 3.1 - Camera subscriber

`camera_node.py`:

- Subscribes to `/camera/image_raw` with message type `sensor_msgs/msg/Image`.
- Maintains an image counter.
- Uses throttled logging (about every 2 seconds) to avoid console spam.
- Logs frame metadata: `frame_id`, timestamp, width/height, and encoding.

## Step 3.2 - LiDAR subscriber + obstacle filtering (6.0 to 7.0 m)

`lidar_node.py` now performs real filtering logic:

1. Reads all values from `msg.ranges`.
2. Ignores invalid values:
   - `inf`
   - `nan`
3. Ignores values outside physical sensor limits:
   - `< msg.range_min`
   - `> msg.range_max`
4. Keeps only distances in target band:
   - `6.0 <= distance <= 7.0`
5. Logs results clearly:
   - If matches exist: prints count and a preview list of distances.
   - If no match exists: prints  
     `No obstacle detected between 6 and 7 meters`

This behavior is ROS 2 Jazzy compatible and based on standard `LaserScan` fields.

## Step 3.3 - IMU subscriber

`imu_node.py`:

- Subscribes to `/imu` with message type `sensor_msgs/msg/Imu`.
- Maintains a message counter.
- Uses throttled logging (about every 2 seconds).
- Logs:
  - orientation quaternion `(x, y, z, w)`
  - angular velocity `(x, y, z)`
  - linear acceleration `(x, y, z)`

## Build instructions

From workspace root:

```bash
cd ~/ros2_ws
colcon build --packages-select multi_sensor_perception
```

## Source instructions

Run in every new terminal:

```bash
source ~/ros2_ws/install/setup.bash
```

## Run instructions

Terminal 1 - launch simulation:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch multi_sensor_perception simulation.launch.py
```

Terminal 2 - run LiDAR node with simulation time:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run multi_sensor_perception lidar_node --ros-args -p use_sim_time:=true
```

Optional: run other nodes similarly:

```bash
ros2 run multi_sensor_perception camera_node --ros-args -p use_sim_time:=true
ros2 run multi_sensor_perception imu_node --ros-args -p use_sim_time:=true
ros2 run multi_sensor_perception fusion_node --ros-args -p use_sim_time:=true
ros2 run multi_sensor_perception latency_node --ros-args -p use_sim_time:=true
```

## Verify LiDAR filtering output

Use one command to see only the key LiDAR detection logs:

```bash
cd ~/ros2_ws && source install/setup.bash && ros2 run multi_sensor_perception lidar_node --ros-args -p use_sim_time:=true 2>&1 | rg "Obstacles detected between 6 and 7 meters|No obstacle detected between 6 and 7 meters"
```

## Quick health checks

```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic hz /scan
ros2 topic hz /imu
```

## Notes

- Keep Gazebo in **Play** mode; paused simulation stops sensor updates.
- If `ros_gz` tools are missing, install:
  `sudo apt install ros-jazzy-ros-gz`
- Future work (`fusion_node`, `latency_node`) can add synchronization, state estimation, and performance metrics.
