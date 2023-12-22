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
            '/depth_camera/image_raw',  # Camera topic
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
            cvImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().info(str(e))
            return

        # Detect striped pattern on cones
        conesImage, stripedCoordinates = self.detectStripedCones(cvImage)

        # Find the biggest and smallest rectangles
        biggestRect, smallestRect = self.findBiggestSmallestRectangles(stripedCoordinates)

        # Calculate the dimensions for the new bounding box
        xBig, yBig, wBig, hBig = biggestRect
        xSmall, ySmall, wSmall, hSmall = smallestRect
        newX = xBig
        newY = yBig + hBig - hSmall
        newW = wBig
        newH = ySmall + hSmall - yBig

        newArea = newW * newH
        # if newArea >= 4000 or newArea <= -4000 :
            # Draw the new bounding box
        cv2.rectangle(conesImage, (newX, newY), ( newX + newW, newY + newH ), (0, 0, 255), 2)

        # Draw the new bounding box
        print("Area " + str(newW*newH))
        # Publish to detect copy for bt
        if newArea != 0:
            msg = Bool()
            msg.data = True  # Assign the integer value to the message's data field
            self.publisher.publish(msg)
        else:
            msg = Bool()
            msg.data = False  # Assign the integer value to the message's data field
            self.publisher.publish(msg)

        # Display the image with the new bounding box
        cv2.imshow("Detection", conesImage)
        cv2.waitKey(1)

    def findBiggestSmallestRectangles(self, stripedCoordinates):
        biggestArea = 0
        smallestArea = float('inf')
        biggestRect = None
        smallestRect = None

        for rect in stripedCoordinates:
            x, y, w, h = rect
            area = w * h

            if area > biggestArea:
                biggestArea = area
                biggestRect = rect
            if area < smallestArea:
                smallestArea = area
                smallestRect = rect

        return biggestRect or (0, 0, 0, 0), smallestRect or (0, 0, 0, 0)


    def detectStripedCones(self, image):
        # Convert image to HSV color space for better color segmentation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for orange color in HSV
        lowerOrange = np.array([0, 180, 0])
        upperOrange = np.array([12, 245, 255]) # 12 > 15 > 18

        # Create a mask using the orange color range
        mask = cv2.inRange(hsv, lowerOrange, upperOrange)

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Find contours of objects in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize list to store cone coordinates
        conesCoordinates = []

        # Loop through detected contours
        for contour in contours:
            # Get bounding box of each contour
            x, y, w, h = cv2.boundingRect(contour)
            
            # Add only contours with a certain area (adjust threshold as needed)
            minContourArea = 100
            if cv2.contourArea(contour) > minContourArea:
                conesCoordinates.append((x, y, w, h))

        return image, conesCoordinates

def main(args=None):
    print("Spinning")
    rclpy.init(args=args)
    node = ConeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()