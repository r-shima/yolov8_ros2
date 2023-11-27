from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='ip_camera_publisher',
            name='ip_camera_publisher'
        )
    ])