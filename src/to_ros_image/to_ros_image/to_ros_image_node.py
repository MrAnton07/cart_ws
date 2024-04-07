import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node): #Нода для преобразования cv2.frame в Image (ROS) формат для работы на реальном роботе
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.cap = cv2.VideoCapture(0)  
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Could not read frame from camera')
            return
        # Конвертируем изображение OpenCV в ROS сообщение и публикуем
        ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.cap.release()
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
