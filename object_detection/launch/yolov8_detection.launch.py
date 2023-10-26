from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='yolov8_detection.py',
            name='yolov8_detection',
            # parameters=[get_package_share_path('object_detection') /
            #             'config/ip_camera_params.yaml']
        )
    ])