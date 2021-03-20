import rclpy
from rclpy.node import Node
import curses

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtle_maneuvering.turtleSpeed import TurleSpeed


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t_speed = TurleSpeed()


    def start(self):
        screen = curses.initscr()
        curses.noecho()
        curses.cbreak()
        screen.keypad(1)
        screen.refresh()
        end_of_node = self.t_speed.set_trtl_speed(screen)
        
        return end_of_node

    def timer_callback(self):
        end = self.start()
        if not end:

            msg = self.t_speed.get_trtl_speed()




            self.publisher_.publish(msg)
            print(msg.linear.x)
            self.get_logger().info(f'Publishing: linear[{msg.linear.x}, \
                                {msg.linear.y}, {msg.linear.z}], \
                                    angular[{msg.angular.x}, {msg.angular.y}, \
                                    {msg.angular.z}]')
        else:
            curses.endwin()
    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

