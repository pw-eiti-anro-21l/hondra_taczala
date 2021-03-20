import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType


class KeyboardParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('keyboard_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameters(namespace='', parameters=[
            ('up', 'g'),
            ('down', 'd'),
            ('left', 'l'),
            ('right', 'p')
        ])

    def timer_callback(self):
    #     my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        params = [self.get_parameter(param).get_parameter_value().string_value 
                  for param in ['up', 'down', 'left', 'right']]
        self.get_logger().info(f'Params: {params}')

    #     my_new_param = rclpy.parameter.Parameter(
    #         'my_parameter',
    #         rclpy.Parameter.Type.STRING,
    #         'world'
    #     )
    #     all_new_parameters = [my_new_param]
    #     self.set_parameters(all_new_parameters)


def main():
    rclpy.init()
    node = KeyboardParam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
