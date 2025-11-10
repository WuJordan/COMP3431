#!/usr/bin/env python3

# Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
# Author: Claude Sammut, modified by Jordan Wu
# Last Modified: 2025.11.10

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray
import csv

from wall_follower.landmark import marker_type, max_markers, Landmark


class PointTransformer(Node):
    def __init__(self):
        super().__init__('point_transformer')

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.point_subscriber = self.create_subscription(
            PointStamped, '/marker_position', self.point_callback, 10)

        # Publishers
        self.marker_publisher_ = self.create_publisher(
            MarkerArray, 'visualization_marker_array', 10)

        # Marker storage
        self.marker_array = MarkerArray()
        self.marker_array.markers = []
        self.marker_position = [Landmark(i, self.marker_array.markers)
                                for i in range(max_markers)]

        self.get_logger().info("✅ PointTransformer node started. Waiting for marker detections...")

    # === Transform each marker and update stored position ===
    def point_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Transform lookup failed: {str(e)}')
            return

        # Get marker ID from z-value
        which_marker = int(msg.point.z)
        m_type = marker_type[which_marker]  # lookup readable type (e.g. "red")
        msg.point.z = 0.0  # set real z to zero for 2D map

        # Transform to /map frame
        map_point = tf2_geometry_msgs.do_transform_point(msg, transform)

        # Update internal record
        self.marker_position[which_marker].update_position(map_point.point)

        self.get_logger().info(
            f"📍 Detected {m_type} marker at ({map_point.point.x:.3f}, {map_point.point.y:.3f}) in /map frame.")

        self.marker_publisher_.publish(self.marker_array)

    # === Save all landmarks into CSV when program stops ===
    def saveLandmarks(self):
        filename = 'markers.csv'
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # First line: assumed start position (0, 0, 0)
            writer.writerow([0.0, 0.0, 0.0])

            # Then write each detected marker
            for lm in self.marker_position:
                if lm.top_marker is not None:
                    x = lm.top_marker.x
                    y = lm.top_marker.y
                    # You can use lm.type or the marker_type index
                    writer.writerow([x, y, lm.type])

        self.get_logger().info(f"💾 Saved {filename} with all detected markers.")


def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()

    # Register a proper shutdown hook
    def save_on_exit():
        node.get_logger().info("💾 Saving landmarks before shutdown...")
        node.saveLandmarks()

    rclpy.get_default_context().on_shutdown(save_on_exit)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
         node.saveLandmarks()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
