import rclpy
from rclpy.node import Node

from interpolation_interfaces.srv import GoToPosition


class Service(Node):
    def __init__(self):
        super().__init__('interpolation_service')
        self.srv = self.create_service(GoToPosition, 'jint_control_service',
                                       self.interpolation_service_callback)

    def interpolation_service_callback(self, req, res):
        res.confirmation = 'confirmed'
        self.get_logger().info(f't:{req.translation},\
             r1:{req.first_rotation}, r2:{req.second_rotation}, \
                 time:{req.time}')
        return res


def main(args=None):
    rclpy.init(args=args)
    interpolation_service = Service()
    rclpy.spin(interpolation_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
