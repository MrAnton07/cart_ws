import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from skimage.measure import ransac
from skimage.measure import LineModelND
from rclpy.time import Time
import matplotlib.pyplot as plt

def slope_angle(x1, y1, x2, y2): #Функция вычисления угла наклона прямой(Красная линия)
    m = (y2 - y1) / (x2 - x1)
    angle_radians = np.arctan(m)
    angle_degrees = np.degrees(angle_radians)
    
    return angle_degrees

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription_key_mask = self.create_subscription(
            String,
            'key_mask_topic',
            self.key_mask_topic_callback,
            10)
        self.publisher = self.create_publisher(
            Float32MultiArray,
            "black_line",
            10
        )
        self.publisher_red = self.create_publisher(
            Bool,
            "has_pink",
            10
        )
        self.publisher_color_under_bot = self.create_publisher(
            String,
            "color_under_bot_topic",
            10
        )
        self.key = ["black", "orange"]
        self.has_red = Bool()
        self.theta = 100.0
        self.on_color = []
        self.line = []
        self.subscription  
        self.bridge = CvBridge()
        self.last_time = self.get_clock().now()
        self.last_time_trig = self.get_clock().now()

    def key_mask_topic_callback(self, msg):
        self.key = msg.data.split(',')

    def image_callback(self, msg):
        message = Float32MultiArray()
        on_color_msg = String()
        try:
            # Преобразование ROS Image сообщения в изображение OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error('Could not convert image: %s' % e)
            return
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #Определяем параметры для каждого цвета
        pink_lower = np.array([290/2, 100, 52])  
        pink_upper = np.array([310/2, 255, 255])
        green_lower = np.array([40, 40, 40])
        green_upper = np.array([80, 255, 255])
        blue_lower = np.array([100, 150, 0])
        blue_upper = np.array([140, 255, 255])
        orange_lower = np.array([5, 100, 100])
        orange_upper = np.array([20, 255, 255])
        low_b = np.uint8([15, 15, 15])
        high_b = np.uint8([0, 0, 0])

        # Создаём маски для каждого цвета
        mask_green = cv2.inRange(hsv, green_lower, green_upper)
        mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)
        mask_orange = cv2.inRange(hsv, orange_lower, orange_upper)
        mask_pink = cv2.inRange(hsv, pink_lower, pink_upper)
        mask_black = cv2.inRange(cv_image, high_b, low_b)

        mask_dict = dict(blue = mask_blue, black = mask_black, green = mask_green, pink = mask_pink, orange = mask_orange)

        self.on_color = [mask_name for mask_name in self.key if np.any(mask_dict[mask_name])] #Проверяем, какие цвета сейчас под нами
        on_color_msg.data = ','.join(self.on_color)
        if on_color_msg.data:
            self.publisher_color_under_bot.publish(on_color_msg)
        else:
            on_color_msg.data = ""
            self.publisher_color_under_bot.publish(on_color_msg)
        print(self.on_color)


        # Шаг 4: Проверка наличия пурпурного цвета
        self.has_red.data = bool(np.any(mask_pink))

        print("На изображении есть пурпурный цвет:" , self.has_red.data)
        current_time_trig = self.get_clock().now()
        eps_trig = current_time_trig - self.last_time_trig
        if self.has_red.data and eps_trig.nanoseconds > 1e9*3:
            self.last_time_trig = current_time_trig
            self.publisher_red.publish(self.has_red)
            self.has_red.data = False
            return
        self.publisher_red.publish(self.has_red)

        mask = cv2.bitwise_or(mask_dict[self.key[0]], mask_dict[self.key[1]])
        
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 1)

        y, x = np.where(mask == 255)
        print(x)
        x_mean = (0.0 if x.size==0 else np.mean(x)) #Средняя координата линии по x
        print(x_mean)
        print(self.theta)
        coords = np.column_stack((x, y))
        current_time = self.get_clock().now()
        eps = current_time - self.last_time
        message.data = [x_mean, self.theta]
        if eps.nanoseconds > 1e9:  #Использование алгоритма RANSAC для аппроксимации найденной маски в линию
            self.last_time = current_time
            try:
                model, inliers = ransac(coords, LineModelND, min_samples=2, residual_threshold=250, max_trials=1)
                line_y = model.predict_y(x)
                
                x_min = x[inliers].min()
                x_max = x[inliers].max()

                # Вычисляем соответствующие y координаты для крайних точек x
                y_min = int(line_y[inliers][x[inliers].argmin()])
                y_max = int(line_y[inliers][x[inliers].argmax()])
                # print(slope_angle(x_min, y_min, x_max, y_max))
                self.line = [(x_min, y_min), (x_max, y_max)]
                self.theta = slope_angle(x_min, y_min, x_max, y_max)
                message.data = [x_mean, self.theta]      
                self.publisher.publish(message)
                # Рисуем линию на изображении
                
            except:
                pass
        self.publisher.publish(message)
        try:
            cv2.line(cv_image, self.line[0], self.line[1], (0, 0, 255), 2) # Используем красный цвет для линии и рисуем её на выходном изображении
        except:
            pass

        #Отображение выходных фреймов, "то, что видит робот"
        cv2.imshow("Frame", cv_image)
        cv2.imshow("Mask", mask)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber_node = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber_node)
    except KeyboardInterrupt:
        pass

    image_subscriber_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

