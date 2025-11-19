import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav2_params = LaunchConfiguration('nav2_params')
    map_yaml = LaunchConfiguration('map_yaml')
    delay = LaunchConfiguration('delay')

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'nav2_params',
        #     default_value='/home/pi/turtlebot3_ws/src/wall_follower/params/waffle_pi.yaml',
        #     description='Path to nav2 params YAML file'
        # ),
        # DeclareLaunchArgument(
        #     'map_yaml',
        #     default_value='/home/pi/turtlebot3_ws/src/wall_follower/map/map.yaml',
        #     description='Path to generated map YAML (points to .pgm/.yaml)'
        # ),
        # DeclareLaunchArgument(
        #     'delay',
        #     default_value='10.0',
        #     description='Delay (seconds) before launching navigate_marker'
        # ),

        # Launch Nav2 using the turtlebot3_navigation2 package launch file.
        # This runs: ros2 launch turtlebot3_navigation2 navigation2.launch.py params_file:=<nav2_params> map:=<map_yaml>
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                'params_file:=/home/pi/turtlebot3_ws/src/wall_follower/params/waffle_pi.yaml',
                'map:=/home/pi/turtlebot3_ws/src/wall_follower/map/map.yaml'
            ],
            output='screen'
        ),

        # After "delay" seconds, run your navigate_marker.py script directly.
        # Adjust the python executable or path if needed.
        # TimerAction(
        #     period=delay,
        #     actions=[
        #         ExecuteProcess(
        #             cmd=[
        #                 'python3',
        #                 '/home/pi/turtlebot3_ws/src/wall_follower/scripts/navigate_marker.py'
        #             ],
        #             output='screen'
        #         )
        #     ]
        # ),
    ])