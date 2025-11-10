#!/usr/bin/env python3
import sys
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class HSVInspectorNode(Node):
    def __init__(self):
        super().__init__('hsv_hover_inspector')

        # Parameters
        self.declare_parameter('topic', '/camera/image_raw/compressed')
        topic = self.get_parameter('topic').get_parameter_value().string_value

        # QoS suitable for compressed images
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.bridge = CvBridge()
        self.latest_frame_lock = threading.Lock()
        self.latest_frame: Optional[np.ndarray] = None  # BGR frame
        self.cursor_pos: Optional[Tuple[int, int]] = None  # (x, y)

        self.window_name = 'HSV Hover Inspector (press q to quit)'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self._on_mouse)

        self.sub = self.create_subscription(
            CompressedImage,
            topic,
            self._image_cb,
            qos
        )
        self.get_logger().info(f"Subscribed to: {topic}")

        # Timer to refresh UI ~60 FPS
        self.timer = self.create_timer(1.0 / 60.0, self._ui_tick)

    def _image_cb(self, msg: CompressedImage):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to decode compressed image: {e}")
            return

        with self.latest_frame_lock:
            self.latest_frame = frame

    def _on_mouse(self, event, x, y, flags, userdata):
        # Track cursor position for display loop
        if event == cv2.EVENT_MOUSEMOVE:
            self.cursor_pos = (x, y)

    @staticmethod
    def _bgr_to_hsv_values(bgr_pixel: np.ndarray):
        """
        Convert a single BGR pixel (uint8 shape (3,)) to:
          - OpenCV HSV (H:0–179, S/V:0–255)
          - Normalized HSV (H:0–360°, S/V:0–1.00)
        """
        # Ensure shape (1,1,3)
        bgr_1x1 = bgr_pixel.reshape((1, 1, 3)).astype(np.uint8)
        hsv_1x1 = cv2.cvtColor(bgr_1x1, cv2.COLOR_BGR2HSV)
        h_cv, s_cv, v_cv = hsv_1x1[0, 0].tolist()

        # Convert OpenCV ranges to normalized ranges
        h_deg = (h_cv / 179.0) * 360.0
        s_norm = s_cv / 255.0
        v_norm = v_cv / 255.0
        return (int(h_cv), int(s_cv), int(v_cv)), (h_deg, s_norm, v_norm)

    def _ui_tick(self):
        # Fetch latest frame
        with self.latest_frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()

        if frame is None:
            # Show a blank info screen until frames arrive
            blank = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for /camera/image_raw/compressed ...",
                        (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(self.window_name, blank)
            cv2.waitKey(1)
            return

        h_img, w_img = frame.shape[:2]
        txt_lines = []

        # If cursor is on the image, compute HSV at that pixel
        if self.cursor_pos is not None:
            x, y = self.cursor_pos
            if 0 <= x < w_img and 0 <= y < h_img:
                b, g, r = frame[y, x].tolist()
                (h_cv, s_cv, v_cv), (h_deg, s_norm, v_norm) = self._bgr_to_hsv_values(
                    np.array([b, g, r], dtype=np.uint8)
                )

                # Draw crosshair/marker
                cv2.drawMarker(frame, (x, y), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)

                # Print to console (optional, throttled by mouse move rate)
                # You can comment this out if too chatty:
                self.get_logger().info(
                    f"(x={x}, y={y}) BGR=({b},{g},{r}) "
                    f"HSVcv=({h_cv},{s_cv},{v_cv}) "
                    f"HSVnorm=({h_deg:.1f}°, {s_norm:.3f}, {v_norm:.3f})"
                )

        # Compose overlay text box
        if txt_lines:
            overlay = frame.copy()
            padding = 8
            line_h = 22
            box_w = 460
            box_h = padding * 2 + line_h * len(txt_lines)
            # Ensure the box is inside the image
            box_x = 10
            box_y = 10
            cv2.rectangle(overlay, (box_x, box_y), (box_x + box_w, box_y + box_h), (0, 0, 0), -1)
            alpha = 0.55
            frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

            y_text = box_y + padding + 16
            for line in txt_lines:
                cv2.putText(frame, line, (box_x + padding, y_text),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
                y_text += line_h

        cv2.imshow(self.window_name, frame)
        # Handle key events; quit on 'q' or ESC
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q') or k == 27:
            self.get_logger().info("Exiting on user request.")
            rclpy.shutdown()
            cv2.destroyAllWindows()


def main(argv=None):
    rclpy.init(args=argv)
    node = HSVInspectorNode()
    try:
        # Use rclpy executor while UI is driven by a timer
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)