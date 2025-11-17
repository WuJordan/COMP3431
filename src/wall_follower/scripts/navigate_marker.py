#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
# from nav_msgs.msg import Odometry
import math
# import tf_transformations

class MarkerNavigator(Node):
    def __init__(self):
        super().__init__('navigate_marker')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(f"Waiting for server start")
        self.client.wait_for_server(timeout_sec=20.0)

        # Read markers and starting pose

        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.start_pose, self.markers = self.load_markers('/home/pi/turtlebot3_ws/markers.csv')
        
        self.set_initial_pose()
        self.navigate_to_markers()

    def load_markers(self, filepath):
        markers = []
        with open(filepath, 'r') as file:
            reader = csv.reader(file)
            start_x, start_y, start_heading = map(float, next(reader))
            start_pose = (start_x, start_y, start_heading)
            
            for row in reader:
                x, y, marker_type = float(row[0]), float(row[1]), row[2]
                markers.append((x, y, marker_type))
                
        return start_pose, markers

    def set_initial_pose(self):
        # Set initial pose based on the start position and heading
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = rclpy.time.Time()
        initial_pose.pose.pose.position.x = self.start_pose[0]
        initial_pose.pose.pose.position.y = self.start_pose[1]
        initial_pose.pose.pose.orientation.w = self.start_pose[2]

        # Publish to `/initialpose`
        self.get_logger().info(f"Setting initial pose: {self.start_pose[0]}, {self.start_pose[1]}, {self.start_pose[2]}")
        self.initial_pose_publisher.publish(initial_pose)
        self.get_logger().info(f"Setting initial pose complete")
        
        # self.client.wait_for_server(timeout_sec=5.0)
        
        # time.sleep(5)
        # self.navigate_to_markers()

        #self.client.send_goal_async(NavigateToPose.Goal(pose=initial_pose), feedback_callback=self.feedback_callback)
    
    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Identity quaternion for no rotation

        # Wait for the action server to be ready
        self.get_logger().info("Waiting for action server to be ready...")
        self.client.wait_for_server(timeout_sec=20.0)
        
        # if not self.client.server_is_ready():
        #     self.get_logger().error('Action server is not ready!')
        #     return

        self.get_logger().info("Sending goal...")
        self._send_goal_future = self.client.send_goal(goal_msg)
     
        # self._send_goal_future = self.client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self, self._send_goal_future)
     

        # self.client.wait_for_server(timeout_sec=5.0)


        self._send_goal_future.add_done_callback(self.goal_response_callback)
        # self.get_logger().info('HERE NOW :)')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.client.wait_for_server(timeout_sec=5.0)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
    
    def navigate_to_markers(self):
        # self.set_initial_pose()  # Set the initial position on the map
        for x, y, marker_type in self.markers:
            self.get_logger().info(f"Navigating to marker {marker_type} at ({x}, {y})")
            self.send_goal(x, y)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

def main(args=None):
    rclpy.init(args=args)
    navigator = MarkerNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()