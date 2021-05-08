import sys

import rclpy
from interpolation_interfaces.srv import ointGoToPosition
from rclpy.node import Node


class OintClient(Node):
    def __init__(self):
        super().__init__('oint_interpolation_client')
        self.cli = self.create_client(GoToPosition, 'oint_control_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GoToPosition.Request()

    def send_request(self):
        self.req.x_t = float(sys.argv[1])
        self.req.y_t = float(sys.argv[2])
        self.req.z_t = float(sys.argv[3])
        self.req.r_r = float(sys.argv[4])
        self.req.p_r = float(sys.argv[5])
        self.req.y_r = float(sys.argv[6])
        self.req.time = float(sys.argv[7])
        self.req.option = sys.argv[8]
        self.req.version = sys.argv[9]


        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    interpolation_client = Client()
    interpolation_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(interpolation_client)
        if interpolation_client.future.done():
            try:
                response = interpolation_client.future.result()
            except Exception as e:
                interpolation_client.get_logger().info(
                    f'Service call failed {e}')
            else:
                interpolation_client.get_logger().info(
                    f'Client obtaied {interpolation_client.req.translation}, \
                        {interpolation_client.req.first_rotation}, \
                            {interpolation_client.req.first_rotation} \
                                result {response.confirmation}')
            break
        interpolation_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
