import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera_on_rod/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Image, 'processed_image', 10)
        self.bridge = CvBridge()

    def detect_color(self, img, color_ranges):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = None
        for (lower, upper, color_name) in color_ranges:
            current_mask = cv2.inRange(hsv, lower, upper)
            if mask is None:
                mask = current_mask
            else:
                mask = cv2.bitwise_or(mask, current_mask)
            contours, _ = cv2.findContours(current_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(img, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return img, mask

    def image_callback(self, msg):
        color_ranges = [
            ((0, 120, 70), (10, 255, 255), 'red_object'),
            ((36, 100, 100), (86, 255, 255), 'green_object')
            #((90, 50, 70), (128, 255, 255), 'blue_object')
        ]
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error('Could not convert from {!s} to "bgr8".'.format(msg.header.frame_id))
        else:
            processed_image, mask = self.detect_color(cv_image, color_ranges)
            cv2.imshow("Frame", processed_image)

            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    color_detector = ColorDetector()
    rclpy.spin(color_detector)
    color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
