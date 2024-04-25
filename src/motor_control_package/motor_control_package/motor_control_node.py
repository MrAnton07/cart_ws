import serial
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=10)
        self.subscription = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            self.motor_control_callback,
            10)
        
    def motor_control_callback(self, msg):
        self.ser.reset_input_buffer()
        print(msg.linear.x, msg.angular.z)
        self.ser.write(str(msg.linear.x).encode())
        self.ser.write(b",")
        self.ser.write(str(msg.angular.z).encode())
        self.ser.write(b"\n")
        print("asfdfsda")
        self.ser.reset_input_buffer()

def main(args=None):
    rclpy.init(args=args)

    motor_control_node = MotorControlNode()

    rclpy.spin(motor_control_node)

    motor_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()