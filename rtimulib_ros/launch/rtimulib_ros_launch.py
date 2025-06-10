from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtimulib_ros',
            executable='rtimulib_ros',
            name='rtimulib_ros',
            output='screen',
            parameters=[{'calibration_file_path': '/home/dodo/dev_ws/src/rtimulib_ros/config/'}]
        )
    ])
