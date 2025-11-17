from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='navigate_marker.py',
            name='navigate_marker'
        )
    ])
