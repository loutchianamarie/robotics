"""Step 3.1: minimal subscriber for /camera/image_raw (sim or hardware)."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):
    """Subscribes to the camera image topic; logs receipts on a light throttle."""

    def __init__(self) -> None:
        super().__init__('camera_node')
        self.get_logger().info(
            'Camera node started; subscribing to /camera/image_raw (Step 3.1).'
        )
        self._image_count = 0
        self._last_log_ns: int | None = None
        self._log_period_ns = 2_000_000_000  # at most one summary line every 2 s

        self._subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            10,
        )

    def _image_callback(self, msg: Image) -> None:
        self._image_count += 1
        now_ns = self.get_clock().now().nanoseconds
        if self._last_log_ns is not None:
            if (now_ns - self._last_log_ns) < self._log_period_ns:
                return
        self._last_log_ns = now_ns

        h = msg.header
        self.get_logger().info(
            f'Received image (count={self._image_count}): '
            f'frame_id={h.frame_id!r} stamp={h.stamp.sec}.{h.stamp.nanosec:09d} '
            f'size={msg.width}x{msg.height} encoding={msg.encoding!r}'
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
