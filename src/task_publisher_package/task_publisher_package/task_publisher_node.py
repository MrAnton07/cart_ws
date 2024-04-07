import sys
import select
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TaskPublisher(Node): #Нода для отправки заданий на точки (в зависимости от желаемой точки жмём a b c или d на клавиатуре)
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'task_topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.old_attrs = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def timer_callback(self):
        if self.key_poll():
            key = sys.stdin.read(1)
            if key.lower() in ['a', 'b', 'c', 'd']:
                msg = String()
                msg.data = f"{key.upper()}"
                self.publisher_.publish(msg)
                self.get_logger().info(f"Publishing: '{msg.data}'")

    def key_poll(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attrs)


def main(args=None):
    rclpy.init(args=args)
    task_publisher = TaskPublisher()
    try:
        rclpy.spin(task_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        task_publisher.shutdown()
        task_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
