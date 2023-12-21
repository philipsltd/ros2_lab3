import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Bool


class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/intel_realsense_r200_depth/image_raw',  # Camera topic
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.publisher = self.create_publisher(
            Bool, 
            '/bt_detect', 
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().info(str(e))
            return

        # Detect striped pattern on cones
        cones_image, striped_coordinates = self.detect_striped_cones(cv_image)

        # Find the biggest and smallest rectangles
        biggest_rect, smallest_rect = self.find_biggest_smallest_rectangles(striped_coordinates)

        # Calculate the dimensions for the new bounding box
        x_big, y_big, w_big, h_big = biggest_rect
        x_small, y_small, w_small, h_small = smallest_rect
        new_x = x_big
        new_y = y_big + h_big - h_small
        new_w = w_big
        new_h = y_small + h_small - y_big

        new_area = new_w * new_h
        # if new_area >= 4000 or new_area <= -4000 :
            # Draw the new bounding box
        cv2.rectangle(cones_image, (new_x, new_y), ( new_x + new_w, new_y + new_h ), (0, 0, 255), 2)
        label_position = (new_x, new_y - 10)  # Position the label slightly above the rectangle
        cv2.putText(cones_image, 'Cone', label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Draw the new bounding box
        print("Area " + str(new_w*new_h))
        # cv2.rectangle(cones_image, (new_x, new_y), (new_x + new_w, new_y + new_h), (0, 0, 255), 2)
        # Publish to detect copy for bt
        if new_area != 0:
            msg = Bool()
            msg.data = True  # Assign the integer value to the message's data field
            self.publisher.publish(msg)
        else:
            msg = Bool()
            msg.data = False  # Assign the integer value to the message's data field
            self.publisher.publish(msg)

        # Display the image with the new bounding box
        cv2.imshow("teste", cones_image)
        cv2.waitKey(1)

    def find_biggest_smallest_rectangles(self, striped_coordinates):
        biggest_area = 0
        smallest_area = float('inf')
        biggest_rect = None
        smallest_rect = None

        for rect in striped_coordinates:
            x, y, w, h = rect
            area = w * h

            if area > biggest_area:
                biggest_area = area
                biggest_rect = rect
            if area < smallest_area:
                smallest_area = area
                smallest_rect = rect

        return biggest_rect or (0, 0, 0, 0), smallest_rect or (0, 0, 0, 0)


    def detect_striped_cones(self, image):
        # Convert image to HSV color space for better color segmentation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for orange color in HSV
        lower_orange = np.array([0, 180, 0])
        upper_orange = np.array([12, 245, 255]) # 12 > 15 > 18

        # Create a mask using the orange color range
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Find contours of objects in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize list to store cone coordinates
        cones_coordinates = []

        # Loop through detected contours
        for contour in contours:
            # Get bounding box of each contour
            x, y, w, h = cv2.boundingRect(contour)
            
            # Add only contours with a certain area (adjust threshold as needed)
            min_contour_area = 100
            if cv2.contourArea(contour) > min_contour_area:
                cones_coordinates.append((x, y, w, h))

        return image, cones_coordinates
        # # Convert image to HSV color space
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # # Define lower and upper bounds for orange color in HSV
        # lower_orange = np.array([0, 100, 100])
        # upper_orange = np.array([20, 255, 255])

        # # Create masks for orange and white regions
        # mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

        # # Combine masks to get the striped pattern
        # mask_striped = cv2.bitwise_or(mask_orange, mask_orange)  # Placeholder, replace with actual logic

        # # Apply edge detection to the combined mask
        # edges = cv2.Canny(mask_striped, 50, 150)

        # # Detect lines using Hough Line Transform
        # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        # # Extract coordinates of lines detected
        # striped_coordinates = []
        # if lines is not None:
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         striped_coordinates.append((min(x1, x2), min(y1, y2), abs(x2 - x1), abs(y2 - y1)))

        # return image, striped_coordinates


def main(args=None):
    print("SpInInG the nodeeeee woooo")
    rclpy.init(args=args)
    node = ConeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()