"""Step 7: synchronized latency measurement for camera, LiDAR, and IMU."""

from collections import deque

import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String


class LatencyNode(Node):
    """Measures per-sensor and total pipeline latency from synchronized messages."""

    def __init__(self) -> None:
        super().__init__('latency_node')
        self.get_logger().info(
            'Latency node started; synchronizing /scan, /camera/image_raw, /imu.'
        )

        self._history = deque(maxlen=100)
        self._latency_pub = self.create_publisher(String, '/latency', 10)

        self._scan_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        self._camera_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self._imu_sub = message_filters.Subscriber(self, Imu, '/imu')
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._camera_sub, self._scan_sub, self._imu_sub],
            queue_size=30,
            slop=0.15,
        )
        self._sync.registerCallback(self._sync_callback)

    @staticmethod
    def _stamp_to_ns(stamp: object) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _sync_callback(self, cam_msg: Image, scan_msg: LaserScan, imu_msg: Imu) -> None:
        callback_start_ns = self.get_clock().now().nanoseconds

        cam_ns = self._stamp_to_ns(cam_msg.header.stamp)
        scan_ns = self._stamp_to_ns(scan_msg.header.stamp)
        imu_ns = self._stamp_to_ns(imu_msg.header.stamp)
        if cam_ns <= 0 or scan_ns <= 0 or imu_ns <= 0:
            self.get_logger().warn('Skipping latency frame: invalid sensor timestamp.')
            return

        camera_latency_ms = max(0.0, (callback_start_ns - cam_ns) / 1_000_000.0)
        lidar_latency_ms = max(0.0, (callback_start_ns - scan_ns) / 1_000_000.0)
        imu_latency_ms = max(0.0, (callback_start_ns - imu_ns) / 1_000_000.0)

        # Proxy for "callback -> fusion output": time spent in this callback until publish.
        processing_ms = 0.0
        total_latency_ms = max(camera_latency_ms, lidar_latency_ms, imu_latency_ms)

        latency_msg = String()
        latency_msg.data = (
            f'camera_ms={camera_latency_ms:.1f}; '
            f'lidar_ms={lidar_latency_ms:.1f}; '
            f'imu_ms={imu_latency_ms:.1f}; '
            f'total_ms={total_latency_ms:.1f}'
        )
        self._latency_pub.publish(latency_msg)

        callback_end_ns = self.get_clock().now().nanoseconds
        processing_ms = max(0.0, (callback_end_ns - callback_start_ns) / 1_000_000.0)
        total_pipeline_ms = total_latency_ms + processing_ms

        self._history.append(total_pipeline_ms)
        avg_total_ms = sum(self._history) / len(self._history)

        self.get_logger().info(
            '[Latency]\n'
            f'Camera: {camera_latency_ms:.0f} ms | '
            f'LiDAR: {lidar_latency_ms:.0f} ms | '
            f'IMU: {imu_latency_ms:.0f} ms | '
            f'Total: {total_pipeline_ms:.0f} ms | '
            f'Avg({len(self._history)}): {avg_total_ms:.0f} ms'
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LatencyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
