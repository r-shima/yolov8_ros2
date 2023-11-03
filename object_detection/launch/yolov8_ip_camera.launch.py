from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='yolov8_ip_camera.py',
            name='yolov8_ip_camera'
        )
    ])