#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ObjectDetectionNode(Node):
    def init(self):
        super().init('object_detection_node')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Image, '/camera/cones_detected', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform object detection using OpenCV or another library
        # Modify this part according to the specific library you use

        # Example: Draw a bounding box around a detected object
        detected_image = cv_image.copy()
        cv2.rectangle(detected_image, (100, 100), (300, 300), (0, 255, 0), 2)

        # Convert the annotated image back to ROS Image message
        annotated_msg = self.bridge.cv2_to_imgmsg(detected_image, encoding='bgr8')

        # Publish the annotated image
        self.publisher.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if name == 'main':
    main()