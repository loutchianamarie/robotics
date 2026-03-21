# Project roadmap

High-level plan for **Multi-Sensor Perception Architecture for Autonomous Robots**. Steps can overlap in time; order reflects typical dependency flow.

## Step 1 — Minimal node architecture

- ROS 2 Python nodes for camera, LiDAR, IMU, fusion, and latency.
- No hardware or simulation yet; nodes spin and log startup for a verifiable skeleton.

## Step 2 — Gazebo simulated sensors

- Launch a robot / world in Gazebo (or Gazebo + ROS bridge as appropriate for the stack).
- Publish representative camera, LiDAR, and IMU topics from simulation.

## Step 3 — Camera / LiDAR / IMU subscriptions

- Wire each sensor node to the correct message types and topic names.
- Validate data flow with `ros2 topic echo` / RViz as needed.

## Step 4 — Synchronization

- Align streams in time (e.g. message filters, approximate/exact sync, timestamps, optional TF).
- Define policy for handling dropped or out-of-order samples.

## Step 5 — Fusion

- Implement fusion in the fusion node (e.g. state estimation, feature-level combination, or map updates depending on research goals).
- Expose stable output topics for downstream consumers.

## Step 6 — Latency evaluation

- Instrument capture-to-consumer paths in the latency node (or shared utilities).
- Report statistics (e.g. mean, variance, percentiles) via logs, topics, or files.

## Step 7 — Optional AI object detection

- Integrate a detector (e.g. YOLO or another model) on camera (and optionally fused) data.
- Keep inference bounded and configurable for real-time experiments.

## Step 8 — Optional autonomy / mapping

- Connect fused perception to navigation, SLAM, or high-level autonomy if the project scope extends that far.
- Treat as optional branch once core multi-sensor pipeline is solid.
