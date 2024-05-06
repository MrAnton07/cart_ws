import serial
import rclpy
import eventlet
import threading
from rclpy.clock import Clock
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class StateMachine(Node):

    def __init__(self):
        super().__init__('main_algorithm')
        self.publisher = self.create_publisher(
            Twist,
            "/diff_cont/cmd_vel_unstamped",
            10)
        self.mask_key_publisher = self.create_publisher(
            String,
            "key_mask_topic",
            10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'black_line',
            self.black_line_callback,
            10)
        self.subscription_pink = self.create_subscription(
            Bool,
            'has_pink',
            self.has_pink_callback,
            10)
        self.subscription_task = self.create_subscription(
            String,
            'task_topic',
            self.task_manager,
            10)
        self.subscription_color_under_bot = self.create_subscription(
            String,
            'color_under_bot_topic',
            self.get_color,
            10)
        
        
        self.rotation_speed = 0.7
        self.last_time = self.get_clock().now()
        self.I = 0.0
        self.in_progress = False
        self.has_black_line = False
        self.task = "A"
        self.flag = False
        self.color_under_bot = []
        self.has_pink = False
        self.how_straight = 0

    def get_color(self, msg):
        self.color_under_bot = msg.data.split(',')
        print("Color Is ", self.color_under_bot)

    def task_manager(self, msg):
        message = String()
        self.task = msg.data
        if msg.data == "A":
            color = "black"
        elif msg.data == "B": 
            color = "green"
        elif msg.data == "C": 
            color = "blue"
        elif msg.data == "D": 
            color = "orange"
        else: pass
        message.data = ','.join([color, self.color_under_bot[0]])
        self.mask_key_publisher.publish(message)

    def rotate_until_color(self):
        while(self.in_progress == True):
            eventlet.sleep(0.1)
            pass
        self.get_logger().info(f'RUC')
        self.in_progress = True
        message = Twist()
        print("1 While")
        color = self.color_under_bot
        while(not ("" in color)):
            message.linear.x = 0.0
            message.angular.z = 0.7
            self.publisher.publish(message)
            color = self.color_under_bot
            print(self.color_under_bot)
            eventlet.sleep(0.1)
        print("2 While")
        while("" in self.color_under_bot):
            message.linear.x = 0.0
            message.angular.z = 0.7
            self.publisher.publish(message)
            color = self.color_under_bot # Убрать
            eventlet.sleep(0.1)
        print("End")
        self.flag = False
        self.in_progress = False

    def back_until_pink(self):
        while(self.in_progress == True):
            eventlet.sleep(0.1)
            pass
        self.get_logger().info(f'BUP')
        self.in_progress = True
        message = Twist()
        self.task = "ON_BASE"
        while(not self.has_pink):
            message.linear.x = 0.10
            message.angular.z = self.how_straight*0.001
            self.publisher.publish(message)
            eventlet.sleep(0.1)
        while(self.has_pink):
            message.linear.x = -0.15
            message.angular.z = self.how_straight*0.001
            self.publisher.publish(message)
            eventlet.sleep(0.1)
        message.linear.x = -0.01
        message.angular.z = self.how_straight*0.001
        self.publisher.publish(message)
        eventlet.sleep(0.4)
        # while(self.how_straight > 5):
        #     message.linear.x = 0.0
        #     message.angular.z = self.how_straight*0.005
        #     self.publisher.publish(message)
        #     eventlet.sleep(0.1)
        self.in_progress = False
        


    def has_pink_callback(self, msg): #Если мы наткнулись на пурпурный - в зависимости от команды - включить соответствующую маску, 
        message = Twist()
        self.has_pink = msg.data
        if msg.data == True and self.flag == False:
            if self.task == "A":
                self.flag = True

                rotating_thread = threading.Thread(target=self.rotate_until_color)
                rotating_thread.start()
                
                if "black" in self.color_under_bot:
                    eventlet.sleep(0.1)
                    getting_back = threading.Thread(target=self.back_until_pink)
                    getting_back.start()

            if self.task == "D":
                self.flag = True
                message.linear.x = 0.0
                message.angular.z = 0.0
                self.publisher.publish(message)
                eventlet.sleep(3)
                
                rotating_thread = threading.Thread(target=self.rotate_until_color)
                rotating_thread.start()

            if self.task == "ON_BASE":
                pass

    def black_line_callback(self, msg):
        self.how_straight = (320-msg.data[0])
        message = Twist()
        if self.flag or self.task == "ON_BASE":
            pass
        #Если линия слишком далеко от центра камеры - приблизиться к линии
        else:
            current_time = self.get_clock().now()
            dt = current_time - self.last_time
            if dt.nanoseconds > 1e8:
                self.last_time = current_time
                self.I = (self.I + (self.how_straight)*dt.nanoseconds/1e9)
                
            kP = 0.005
            kI = 0.0003
            P = self.how_straight
            dev = abs(P*0.1)
            I = self.I

            x_speed = -((1/dev if (dev > 1.0) else 1.0)*0.2+0.03)
            message.linear.x = (x_speed if (abs(x_speed) < 0.8) else -0.8)#-(100/abs(P) if (abs(P) > 2) else 50)*0.01#1/(1 if how_straight > 1 else how_straight)     P*kP + 
            message.angular.z = P*kP #+ I*kI
            self.publisher.publish(message)

def main(args=None):
    #eventlet.sleep(1)
    rclpy.init(args=args)

    state_machine = StateMachine()

    rclpy.spin(state_machine)

    state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()