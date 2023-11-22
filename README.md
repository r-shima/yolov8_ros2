# YOLOv8 Object Detection Using ROS 2
This repository contains ROS 2 packages that run YOLOv8 object detection using an IP camera or a RealSense camera. For the IP camera, Amcrest IP2M-841 was used. For the RealSense camera, D435i was used.
## Set Up for IP Camera
Create a file called `config.json` inside the object_detection package and add the following:
```
{
    "rtsp_url": "rtsp://<username>:<password>@<ip_address>"
}
```
Replace `<username>`, `<password>`, and `<ip_address>` with your camera's actual username, password, and IP address.
## Quickstart
1. Clone this repository in the src directory of your workspace. In your workspace, build the package by running `colcon build`.
2. Source your workspace by running `source install/setup.bash`.
3. Run one of the following:
    * Run `ros2 launch object_detection ip_camera_stream.launch.py` to stream from the IP camera.
    * Run `ros2 launch object_detection yolov8_ip_camera.launch.py` to detect objects on the IP camera.
    * Run `ros2 launch object_detection yolov8_realsense.launch.py` to detect objects on the RealSense camera.

## Packages
* [object_detection](https://github.com/r-shima/yolov8_ros2/tree/main/object_detection): This package performs object detection
* [object_detection_interfaces](https://github.com/r-shima/yolov8_ros2/tree/main/object_detection_interfaces): This package defines custom messages relevant to object detection using the IP camera