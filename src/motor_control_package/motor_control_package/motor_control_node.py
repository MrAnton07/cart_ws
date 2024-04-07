import serial
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('black_line_topic_subscriber')
        self.publisher = self.create_publisher(
            Twist,
            "/diff_cont/cmd_vel_unstamped",
            10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'black_line',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        message = Twist()
        #Если линия слишком далеко от центра камеры - приблизиться к линии
        theta = msg.data[-1]
        print(theta)
        if (0 < theta < 30):
            message.angular.z = -0.08
            message.linear.x = -0.01
        if (0 > theta > -30):
            message.angular.z = 0.08
            message.linear.x = -0.01
        else:
            message.linear.x = -0.3
            message.angular.z = (320 - msg.data[0])*0.003
        self.publisher.publish(message)

        
        #Когда линия близко к центру - выровняться по линии

def main(args=None):
    #eventlet.sleep(1)
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()