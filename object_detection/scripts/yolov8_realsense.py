#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_prefix
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO

class YOLOv8RealSense(Node):

    def __init__(self):
        super().__init__('yolov8_realsense')
        self.bridge = CvBridge()
        self.detection_pub = self.create_publisher(String, 'detected_objects', 10)
        self.color_sub = self.create_subscription(
            CompressedImage,
            '/camera/infra1/image_rect_raw/compressed',
            self.grayscale_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/infra1/camera_info',
            self.camera_info_callback,
            10)
        self.frequency = 20
        self.timer = self.create_timer(1 / self.frequency, self.timer_callback)

        # Load pre-trained weights
        package_prefix_directory = get_package_prefix('object_detection')
        weights_path = os.path.join(package_prefix_directory, 'lib', 'object_detection',
                                    'doors_and_tables.pt')
        self.model = YOLO(weights_path)

        self.grayscale_image = None
        self.depth_image = None
        self.intrinsics = None

    def camera_info_callback(self, msg):
        # Set up intrinsic parameters
        self.intrinsics = rs.intrinsics()
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = msg.k[2]
        self.intrinsics.ppy = msg.k[5]
        self.intrinsics.fx = msg.k[0]
        self.intrinsics.fy = msg.k[4]

        # Set distortion model and coefficients
        if msg.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs.distortion.brown_conrady
        elif msg.distortion_model == 'equidistant':
            self.intrinsics.model = rs.distortion.kannala_brandt4

        self.intrinsics.coeffs = [i for i in msg.d]

    def grayscale_callback(self, msg):
        grayscale_image = self.bridge.compressed_imgmsg_to_cv2(msg)

        # Rotate the image 90 degrees counterclockwise
        self.grayscale_image = cv2.rotate(grayscale_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Replicate the grayscale channel to mimic an RGB image
        self.grayscale_image = cv2.cvtColor(self.grayscale_image, cv2.COLOR_GRAY2BGR)

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg)

        # Rotate the image 90 degrees counterclockwise
        self.depth_image = cv2.rotate(depth_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    def process_images(self):
        results = self.model(self.grayscale_image, verbose=False)

        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get bounding box coordinates in (top, left, bottom, right) format
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                
                # Get the class value
                c = box.cls

                # Get depth and calculate real world coordinates
                depth = self.depth_image[int((b[1] + b[3]) / 2), int((b[0] + b[2]) / 2)]
                coords = rs.rs2_deproject_pixel_to_point(self.intrinsics,
                                                         [int((b[0] + b[2]) / 2),
                                                          int((b[1] + b[3]) / 2)], depth)

                detection_info = f"Object: {self.model.names[int(c)]}, Coordinates: {coords}"
                self.detection_pub.publish(String(data=detection_info))

        annotated_frame = results[0].plot()
        cv2.imshow("grayscale_image", annotated_frame)
        cv2.waitKey(1)

    def timer_callback(self):
        if self.grayscale_image is not None and self.depth_image is not None and \
            self.intrinsics is not None:
            self.process_images()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8RealSense()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()