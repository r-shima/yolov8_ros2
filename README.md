# YOLOv8 Object Detection Using an IP Camera
This repository contains a ROS 2 package that publishes RTSP streams from an IP camera and performs object detection. Amcrest IP2M-841 was used.

Before using this package, you will need to set up the camera based on the instructions provided by the manufacturer.
## Quickstart
1. Clone this repository in the src directory of your workspace. In your workspace, build the package by running `colcon build`.
2. Source your workspace by running `source install/setup.bash`.
3. Run `ros2 launch object_detection ip_camera_stream.launch.py` to stream from the camera.
4. Run `ros2 launch object_detection yolov8_detection.launch.py` to detect objects.

This repository is a work in progress.