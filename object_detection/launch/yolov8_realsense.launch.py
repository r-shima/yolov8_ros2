from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='yolov8_realsense.py',
            name='yolov8_realsense'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ),
            launch_arguments={
                'enable_depth': 'true',
                'align_depth.enable': 'false',
                'enable_color': 'false',
                'enable_fisheye1': 'false',
                'enable_fisheye2': 'false',
                'enable_infra1': 'true',
                'colorizer.enable': 'false',
                'clip_distance': '10.0',
                'tf_publish_rate': '0.0',
                'depth_module.profile': '640x480x30'
            }.items()
        )
    ])