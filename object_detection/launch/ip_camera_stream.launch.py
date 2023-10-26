from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='ip_camera_publisher',
            name='ip_camera_publisher',
            # parameters=[get_package_share_path('object_detection') /
            #             'config/ip_camera_params.yaml']
        )
    ])