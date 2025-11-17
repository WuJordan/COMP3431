#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import math

class MarkerNavigator(Node):
    def __init__(self):
        super().__init__('navigate_marker')

        # Action client for Nav2
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for action server...")
        self.client.wait_for_server(timeout_sec=20.0)

        # Load markers from CSV
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.start_pose, self.markers = self.load_markers(
            '/home/pi/turtlebot3_ws/markers.csv'
        )

        self.marker_index = 0   # start from first marker

        # Start navigation after node spins up
        self.start_timer = self.create_timer(2.0, self.start_navigation)

    # -----------------------------------------------------

    def load_markers(self, filepath):
        markers = []
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            start_x, start_y, start_heading = map(float, next(reader))
            start_pose = (start_x, start_y, start_heading)

            for row in reader:
                x, y, marker_type = float(row[0]), float(row[1]), row[2]
                markers.append((x, y, marker_type))

        return start_pose, markers

    # -----------------------------------------------------

    def set_initial_pose(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.pose.position.x = self.start_pose[0]
        pose.pose.pose.position.y = self.start_pose[1]

        # Simplest orientation (yaw ignored)
        pose.pose.pose.orientation.w = 0.0

        self.get_logger().info(f"Initial pose set to: {self.start_pose}")
        self.initial_pose_pub.publish(pose)

    # -----------------------------------------------------

    def start_navigation(self):
        self.start_timer.cancel()    # Run once

        self.get_logger().info("Publishing initial pose...")
        self.set_initial_pose()

        time.sleep(2)   # Allow AMCL to settle

        self.get_logger().info("Starting navigation sequence...")
        self.send_next_goal()

    # -----------------------------------------------------

    def send_next_goal(self):
        if self.marker_index >= len(self.markers):
            self.get_logger().info("All markers reached! Navigation complete.")
            return

        x, y, marker_type = self.markers[self.marker_index]
        self.marker_index += 1

        self.get_logger().info(
            f"Sending goal to marker {marker_type} at ({x}, {y})"
        )
        self.send_goal(x, y)

    # -----------------------------------------------------

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 0.0

        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    # -----------------------------------------------------

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected — skipping to next.")
            self.send_next_goal()
            return

        self.get_logger().info("Goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    # -----------------------------------------------------

    def goal_result_callback(self, future):
        self.get_logger().info("Goal reached successfully!")
        self.send_next_goal()

# ----------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MarkerNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
