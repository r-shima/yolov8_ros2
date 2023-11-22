# object_detection
This package performs object detection using an IP camera or a RealSense camera.
## Nodes
* `ip_camera_publisher`: Publishes IP camera streams
* `yolov8_ip_camera`: Runs YOLOv8 on IP camera streams
* `yolov8_realsense`: Runs YOLOv8 on RealSense camera streams
## Launch Files
* `ip_camera_stream.launch.py`: Runs the `ip_camera_publisher` node
* `yolov8_ip_camera.launch.py`: Runs the `yolov8_ip_camera` node
* `yolov8_realsense.launch.py`: Runs the `yolov8_realsense` node