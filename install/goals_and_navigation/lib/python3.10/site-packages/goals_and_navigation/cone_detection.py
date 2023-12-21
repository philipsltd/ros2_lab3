import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrafficConeDetector(Node):
    def __init__(self):
        super().__init__('cone_detection_node')
        self.camera_subscriber = self.create_subscription(
            Image,
            '/intel_realsense_r200_depth/image_raw',  # Camera feed topic
            self.process_image,
            10)
        self.detection_publisher = self.create_publisher(
            Bool, 
            '/bt_detect', 
            10)
        self.image_converter = CvBridge()

    def process_image(self, img_msg):
        try:
            current_image = self.image_converter.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception as e:
            self.get_logger().info(f'Exception: {str(e)}')
            return

        processed_image, detected_cones = self.find_cones(current_image)
        largest_cone, smallest_cone = self.identify_cone_sizes(detected_cones)

        # Draw bounding box and label
        self.draw_label(processed_image, largest_cone, smallest_cone)

        # Display updated image
        cv2.imshow("Cone Detection", processed_image)
        cv2.waitKey(1)

    def identify_cone_sizes(self, cone_coords):
        max_size, min_size = 0, float('inf')
        largest, smallest = None, None

        for coords in cone_coords:
            x, y, w, h = coords
            size = w * h

            if size > max_size:
                max_size = size
                largest = coords

            if size < min_size:
                min_size = size
                smallest = coords

        return largest or (0, 0, 0, 0), smallest or (0, 0, 0, 0)

    def find_cones(self, img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        orange_range = np.array([0, 180, 0]), np.array([12, 245, 255])
        mask = cv2.inRange(hsv_img, *orange_range)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cone_coords = [(x, y, w, h) for x, y, w, h in [cv2.boundingRect(c) for c in contours] if cv2.contourArea(c) > 100]

        return img, cone_coords

    def draw_label(self, img, large_cone, small_cone):
        x_big, y_big, w_big, h_big = large_cone
        x_small, y_small, w_small, h_small = small_cone
        x, y, w, h = x_big, y_big + h_big - h_small, w_big, y_small + h_small - y_big

        if w * h != 0:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            label_pos = (x, y - 10)
            cv2.putText(img, 'Traffic Cone', label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            self.detection_publisher.publish(Bool(data=True))
        else:
            self.detection_publisher.publish(Bool(data=False))

def main():
    print("Starting Traffic Cone Detection Node")
    rclpy.init()
    detector = TrafficConeDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
