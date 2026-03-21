"""Step 8: camera AI node with optional YOLO and safe fallback."""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - runtime dependency may be unavailable
    CvBridge = None  # type: ignore[assignment]

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - runtime dependency may be unavailable
    YOLO = None  # type: ignore[assignment]


class CameraNode(Node):
    """Runs optional YOLO detection and publishes camera perception output."""

    def __init__(self) -> None:
        super().__init__('camera_node')
        self.declare_parameter('enable_ai', True)
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)

        self._enable_ai = bool(self.get_parameter('enable_ai').value)
        self._yolo_model_path = str(self.get_parameter('yolo_model').value)
        self._confidence_threshold = float(self.get_parameter('confidence_threshold').value)

        self._image_count = 0
        self._last_log_ns: int | None = None
        self._log_period_ns = 2_000_000_000  # one summary line every 2 s
        self._bridge = CvBridge() if CvBridge is not None else None
        self._yolo_model = None

        self._perception_pub = self.create_publisher(String, '/camera/perception', 10)
        self._subscription = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10
        )

        self._init_ai_backend()
        self.get_logger().info('Camera node started; AI detections on /camera/perception.')

    def _init_ai_backend(self) -> None:
        if not self._enable_ai:
            self.get_logger().info('Camera AI disabled (enable_ai:=false).')
            return
        if self._bridge is None:
            self.get_logger().warn('cv_bridge not available; falling back to simple mode.')
            self._enable_ai = False
            return
        if YOLO is None:
            self.get_logger().warn('ultralytics not available; falling back to simple mode.')
            self._enable_ai = False
            return
        try:
            self._yolo_model = YOLO(self._yolo_model_path)
            self.get_logger().info(f'Loaded YOLO model: {self._yolo_model_path}')
        except Exception as exc:
            self.get_logger().warn(f'YOLO init failed ({exc}); falling back to simple mode.')
            self._yolo_model = None
            self._enable_ai = False

    @staticmethod
    def _stamp_to_ns(stamp: object) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _publish_perception(
        self,
        stamp_ns: int,
        label: str,
        confidence: float,
        bbox: tuple[float, float, float, float] | None,
        mode: str,
    ) -> None:
        bbox_text = 'none'
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            bbox_text = f'{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}'

        msg = String()
        msg.data = (
            f'stamp_ns={stamp_ns}; '
            f'detected={str(label != "none").lower()}; '
            f'label={label}; '
            f'confidence={confidence:.3f}; '
            f'bbox={bbox_text}; '
            f'mode={mode}'
        )
        self._perception_pub.publish(msg)

    def _image_callback(self, msg: Image) -> None:
        self._image_count += 1
        callback_start = time.perf_counter()

        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        simple_detected = bool(msg.data)

        detected_label = 'none'
        detected_conf = 0.0
        detected_bbox: tuple[float, float, float, float] | None = None
        mode = 'simple'
        ai_elapsed_ms = 0.0

        if self._enable_ai and self._bridge is not None and self._yolo_model is not None:
            try:
                cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                ai_start = time.perf_counter()
                results = self._yolo_model(cv_image, verbose=False)
                ai_elapsed_ms = (time.perf_counter() - ai_start) * 1000.0

                if results and len(results[0].boxes) > 0:
                    best_box = None
                    best_conf = 0.0
                    names = results[0].names
                    for box in results[0].boxes:
                        conf = float(box.conf.item())
                        if conf < self._confidence_threshold or conf <= best_conf:
                            continue
                        cls_id = int(box.cls.item())
                        xyxy = box.xyxy[0].tolist()
                        best_box = (
                            float(xyxy[0]),
                            float(xyxy[1]),
                            float(xyxy[2]),
                            float(xyxy[3]),
                        )
                        best_conf = conf
                        detected_label = str(names.get(cls_id, f'class_{cls_id}'))

                    if best_box is not None:
                        detected_bbox = best_box
                        detected_conf = best_conf
                mode = 'ai'
            except Exception as exc:
                self.get_logger().warn(f'YOLO inference failed ({exc}); using fallback mode.')
                self._enable_ai = False
                mode = 'simple'

        if mode == 'simple' and simple_detected:
            detected_label = 'image_present'
            detected_conf = 0.10

        self._publish_perception(
            stamp_ns=stamp_ns,
            label=detected_label,
            confidence=detected_conf,
            bbox=detected_bbox,
            mode=mode,
        )

        callback_elapsed_ms = (time.perf_counter() - callback_start) * 1000.0
        now_ns = self.get_clock().now().nanoseconds
        if self._last_log_ns is None or (now_ns - self._last_log_ns) >= self._log_period_ns:
            self._last_log_ns = now_ns
            if detected_label != 'none':
                self.get_logger().info(
                    f'[Camera AI] Detected: {detected_label} '
                    f'({detected_conf:.2f} confidence) | mode={mode}'
                )
            else:
                self.get_logger().info(f'[Camera AI] No object detected | mode={mode}')
            self.get_logger().info(
                f'[Camera AI] Latency compare | simple callback={callback_elapsed_ms:.1f} ms '
                f'| ai inference={ai_elapsed_ms:.1f} ms'
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
