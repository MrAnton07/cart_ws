import serial
import rclpy
import eventlet
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
        
        self.last_time = self.get_clock().now()
        self.I = 0.0
        self.counter = 0
        self.has_black_line = False
        self.task = "N"
        self.flag = False
        self.color_under_bot = []
        self.has_pink = False

    def get_color(self, msg):
        self.color_under_bot = msg.data.split(',')

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

    def has_pink_callback(self, msg): #Если мы наткнулись на пурпурный - в зависимости от команды - включить соответствующую маску, 
        message = Twist()
        if msg.data == True and self.flag == False:
            self.has_pink = True
            message.linear.x = 0.0
            message.angular.z = 0.0
            self.publisher.publish(message)
            # if msg.data == "A":
            #     color = "black"
            # elif msg.data == "B": 
            #     color = "green"
            # elif msg.data == "C": 
            #     color = "blue"
            # elif msg.data == "D": 
            #     color = "orange"
            # else: pass
            eventlet.sleep(3)
            self.task = "A" #Если нет команд - едем на базу
            ###################
            for i in range(20):
                message.angular.z = 0.7
                self.publisher.publish(message)
                eventlet.sleep(0.05)
            self.flag = True
            self.has_black_line = False
        else:
            self.has_pink = False
        print(self.has_pink)

    def black_line_callback(self, msg):
        message = Twist()
        if self.flag:
            message.angular.z = 0.7
            self.publisher.publish(message)
            if msg.data[0] and self.counter > 7:
                self.has_black_line = True
                self.flag = False
                print("SDF")
            else:
                self.has_black_line = False
                print("NO_BLACK_LINE")
            self.counter+=1
        #Если линия слишком далеко от центра камеры - приблизиться к линии
        else:
            how_straight = (320-msg.data[0])
            current_time = self.get_clock().now()
            dt = current_time - self.last_time
            if dt.nanoseconds > 1e8:
                self.last_time = current_time
                self.I = (self.I + (how_straight)*dt.nanoseconds/1e9)
                
            self.counter = 0
            kP = 0.005
            kI = 0.0003
            P = how_straight
            dev = abs(P*0.1)
            I = self.I

            theta = msg.data[-1]
            print(theta)
            if(self.has_pink != True):
                x_speed = -((1/dev if (dev > 1.0) else 1.0)*0.2+0.03)
                message.linear.x = x_speed if (abs(x_speed) < 0.8) else -0.8#-(100/abs(P) if (abs(P) > 2) else 50)*0.01#1/(1 if how_straight > 1 else how_straight)     P*kP + 
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