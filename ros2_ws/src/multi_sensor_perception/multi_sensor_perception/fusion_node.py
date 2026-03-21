"""Step 6: complete synchronized multi-sensor fusion node."""

import math
from typing import Any

import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String


class FusionNode(Node):
    """Synchronize LiDAR/Camera/IMU and publish unified perception output."""

    def __init__(self) -> None:
        super().__init__('fusion_node')
        self.get_logger().info(
            'Fusion node started; syncing /scan, /camera/image_raw, /imu.'
        )

        self._max_detection_m = 7.0
        self._sync_warn_threshold_ms = 120.0
        self._fusion_pub = self.create_publisher(String, '/fusion/output', 10)
        self._latest_camera_ai: dict[str, Any] | None = None

        self._scan_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        self._camera_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self._imu_sub = message_filters.Subscriber(self, Imu, '/imu')
        self._camera_ai_sub = self.create_subscription(
            String, '/camera/perception', self._camera_ai_callback, 10
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._scan_sub, self._camera_sub, self._imu_sub],
            queue_size=20,
            slop=0.15,
        )
        self._sync.registerCallback(self._sync_callback)

    @staticmethod
    def _stamp_to_ns(stamp: object) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _extract_valid_ranges(self, scan_msg: LaserScan) -> list[float]:
        return [
            d
            for d in scan_msg.ranges
            if math.isfinite(d) and 0.0 <= d <= self._max_detection_m
        ]

    def _camera_ai_callback(self, msg: String) -> None:
        parsed = self._parse_perception(msg.data)
        if parsed is not None:
            self._latest_camera_ai = parsed

    @staticmethod
    def _parse_perception(raw: str) -> dict[str, Any] | None:
        try:
            pieces = [part.strip() for part in raw.split(';') if part.strip()]
            data: dict[str, Any] = {}
            for piece in pieces:
                if '=' not in piece:
                    continue
                key, value = piece.split('=', 1)
                data[key.strip()] = value.strip()
            if 'stamp_ns' not in data:
                return None
            return {
                'stamp_ns': int(data.get('stamp_ns', '0')),
                'detected': data.get('detected', 'false').lower() == 'true',
                'label': data.get('label', 'none'),
                'confidence': float(data.get('confidence', '0.0')),
                'mode': data.get('mode', 'simple'),
            }
        except Exception:
            return None

    def _build_output(
        self,
        object_detected: bool,
        closest_distance: float | None,
        yaw_rad: float,
        timestamp_ns: int,
        sync_delay_ms: float,
        camera_detected: bool,
        camera_label: str,
        camera_confidence: float,
        camera_mode: str,
    ) -> String:
        output = String()
        distance_text = 'None' if closest_distance is None else f'{closest_distance:.3f}'
        output.data = (
            f'object_detected={str(object_detected).lower()}; '
            f'distance_m={distance_text}; '
            f'orientation_yaw_rad={yaw_rad:.4f}; '
            f'timestamp_ns={timestamp_ns}; '
            f'sync_delay_ms={sync_delay_ms:.1f}; '
            f'camera_detected={str(camera_detected).lower()}; '
            f'camera_label={camera_label}; '
            f'camera_confidence={camera_confidence:.3f}; '
            f'camera_mode={camera_mode}'
        )
        return output

    def _sync_callback(self, scan_msg: LaserScan, cam_msg: Image, imu_msg: Imu) -> None:
        if not scan_msg.ranges:
            self.get_logger().warn('Skipping frame: empty LiDAR scan.')
            return

        scan_ns = self._stamp_to_ns(scan_msg.header.stamp)
        cam_ns = self._stamp_to_ns(cam_msg.header.stamp)
        imu_ns = self._stamp_to_ns(imu_msg.header.stamp)
        if scan_ns <= 0 or cam_ns <= 0 or imu_ns <= 0:
            self.get_logger().warn('Skipping frame: invalid sensor timestamps.')
            return

        lidar_camera_ms = abs(scan_ns - cam_ns) / 1_000_000.0
        lidar_imu_ms = abs(scan_ns - imu_ns) / 1_000_000.0
        camera_imu_ms = abs(cam_ns - imu_ns) / 1_000_000.0
        sync_delay_ms = max(lidar_camera_ms, lidar_imu_ms, camera_imu_ms)

        valid_ranges = self._extract_valid_ranges(scan_msg)
        object_count = len(valid_ranges)
        closest_distance = min(valid_ranges) if valid_ranges else None
        object_detected = closest_distance is not None

        camera_detected = False
        camera_label = 'none'
        camera_confidence = 0.0
        camera_mode = 'simple'
        if self._latest_camera_ai is not None:
            perception_stamp = int(self._latest_camera_ai['stamp_ns'])
            perception_delta_ms = abs(perception_stamp - cam_ns) / 1_000_000.0
            if perception_delta_ms <= 300.0:
                camera_detected = bool(self._latest_camera_ai['detected'])
                camera_label = str(self._latest_camera_ai['label'])
                camera_confidence = float(self._latest_camera_ai['confidence'])
                camera_mode = str(self._latest_camera_ai['mode'])
            else:
                camera_detected = bool(cam_msg.data)
        else:
            camera_detected = bool(cam_msg.data)

        q = imu_msg.orientation
        yaw_rad = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        output_stamp_ns = max(scan_ns, cam_ns, imu_ns)

        output_msg = self._build_output(
            object_detected=object_detected,
            closest_distance=closest_distance,
            yaw_rad=yaw_rad,
            timestamp_ns=output_stamp_ns,
            sync_delay_ms=sync_delay_ms,
            camera_detected=camera_detected,
            camera_label=camera_label,
            camera_confidence=camera_confidence,
            camera_mode=camera_mode,
        )
        self._fusion_pub.publish(output_msg)

        closest_text = 'N/A' if closest_distance is None else f'{closest_distance:.1f} m'
        self.get_logger().info(
            '[Fusion]\n'
            f'Objects: {object_count} | Closest: {closest_text} | '
            f'Orientation: {yaw_rad:.2f} rad | Sync delay: {sync_delay_ms:.0f} ms'
        )
        if sync_delay_ms > self._sync_warn_threshold_ms:
            self.get_logger().warn('WARNING: High latency')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
