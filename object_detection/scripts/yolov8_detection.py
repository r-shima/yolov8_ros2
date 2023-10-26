#!/usr/bin/env python3

import os
import json
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ultralytics import YOLO

class YOLOv8Detection(Node):

    def __init__(self):
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

        self.frequency = 20
        self.timer = self.create_timer(1 / self.frequency, self.timer_callback)

    def timer_callback(self):
        for result in self.results:
            result.plot()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8Detection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()