# YOLOv8 Object Detection Using ROS 2
This repository contains a ROS 2 package that performs object detection using an IP camera and a RealSense camera. For the IP camera, Amcrest IP2M-841 was used.

## Quickstart
1. Clone this repository in the src directory of your workspace. In your workspace, build the package by running `colcon build`.
2. Source your workspace by running `source install/setup.bash`.
3. Run one of the following:
    * Run `ros2 launch object_detection ip_camera_stream.launch.py` to stream from the IP camera.
    * Run `ros2 launch object_detection yolov8_ip_camera.launch.py` to detect objects on the IP camera.
    * Run `ros2 launch object_detection yolov8_realsense.launch.py` to detect objects on the RealSense camera.

This repository is a work in progress.