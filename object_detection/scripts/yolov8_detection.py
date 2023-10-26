#!/usr/bin/env python3

"""
PUBLISHER:
    + /detection_results (object_detection_interfaces/msg/DetectionResult) - A list of detections

PARAMETER:
    + rtsp_url (string) - RTSP URL of the IP camera
"""

import os
import json
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from object_detection_interfaces.msg import ObjectDetection, DetectionResult
from ultralytics import YOLO

class YOLOv8Detection(Node):
    """
    This node runs YOLOv8 object detection on video streams
    """

    def __init__(self):
        """
        Initializes the node, declares parameters, creates publisher, and loads the YOLO model
        """
        super().__init__('yolov8_detection')
        package_prefix_directory = get_package_prefix('object_detection')
        weights_path = os.path.join(package_prefix_directory, 'lib', 'object_detection',
                                    'doors_and_tables.pt')
        config_path = os.path.join(package_prefix_directory, 'lib', 'object_detection',
                                    'config.json')

        with open(config_path, 'r') as file:
            config = json.load(file)

        rtsp_url = config.get('rtsp_url', '')
        self.declare_parameter("rtsp_url", rtsp_url,
                               ParameterDescriptor(description="RTSP URL"))
        self.rtsp_url = self.get_parameter("rtsp_url").get_parameter_value().string_value

        self.model = YOLO(weights_path)
        self.results = self.model.predict(source=self.rtsp_url, show=True, verbose=False,
                                          stream=True)

        self.detection_pub = self.create_publisher(DetectionResult, 'detection_results', 10)
        self.frequency = 20
        self.timer = self.create_timer(1 / self.frequency, self.timer_callback)

    def publish_detections(self, results):
        """
        Publish the detected objects as messages to a ROS topic.

        Args: results: A list of detection results, where each result contains bounding box
                       coordinates, confidence, and class information

        Returns: None
        """
        detection_msg = DetectionResult()

        for result in results:
            num_detections = len(result.boxes.xyxy)

            for i in range(num_detections):
                object_detection = ObjectDetection()

                # Get the xyxy coordinates
                box_xyxy = result.boxes.xyxy[i].cpu().numpy()
                object_detection.x1 = box_xyxy[0].astype(float)
                object_detection.y1 = box_xyxy[1].astype(float)
                object_detection.x2 = box_xyxy[2].astype(float)
                object_detection.y2 = box_xyxy[3].astype(float)

                # Get the confidence value
                object_detection.confidence = result.boxes.conf[i].cpu().item()

                # Get the class value
                object_detection.class_value = int(result.boxes.cls[i].cpu().item())

                detection_msg.detections.append(object_detection)

        self.detection_pub.publish(detection_msg)
    
    def timer_callback(self):
        """
        Callback function for the timer. Detection results are visualized and published.

        Args: None

        Returns: None
        """
        for result in self.results:
            result.plot()
            self.publish_detections(result)

def main(args=None):
    """
    The main function
    """
    rclpy.init(args=args)
    node = YOLOv8Detection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()