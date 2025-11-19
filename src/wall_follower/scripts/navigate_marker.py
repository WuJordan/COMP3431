#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import csv
import math
import time

class MarkerNavigator(Node):

    def __init__(self):
        super().__init__('navigate_marker')

        # Action client
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Load markers
        self.start_pose, self.markers = self.load_markers(
            '/home/pi/turtlebot3_ws/markers.csv')

        # Wait for Nav2 action server
        self.get_logger().info("Waiting for NavigateToPose action server...")
        self.client.wait_for_server()

        # Set initial pose
        self.set_initial_pose()

        # Give AMCL time to lock in
        time.sleep(10)

        # Start navigation after setup
        self.current_index = 0
        self.navigate_to_next_marker()

    def load_markers(self, path):
        markers = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            sx, sy, heading = map(float, next(reader))
            for row in reader:
                x, y, mtype = float(row[0]), float(row[1]), row[2]
                markers.append((x, y, mtype))

        markers.append((0.0, 0.0, "start"))
        return (sx, sy, heading), markers

    def set_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'

        # x, y, heading = self.start_pose
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0


        # Convert heading → quaternion
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.w = 1.0

        # msg.pose.covariance = [0.0] * 36

        self.initial_pose_pub.publish(msg)
        self.get_logger().info("Initial pose published")

    def navigate_to_next_marker(self):
        if self.current_index >= len(self.markers):
            self.get_logger().info("Finished all markers!")
            return

        x, y, mtype = self.markers[self.current_index]
        self.get_logger().info(f"Navigating to marker {mtype}: ({x}, {y})")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 0.0

        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("Goal reached.")
        self.current_index += 1
        self.navigate_to_next_marker()

    def feedback_callback(self, feedback):
        pass  # optional

def main():
    rclpy.init()
    node = MarkerNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()