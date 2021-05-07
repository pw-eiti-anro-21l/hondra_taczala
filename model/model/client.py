import sys

import rclpy
from rclpy.node import Node

from interpolation_interfaces.srv import GoToPosition


class Client(Node):
    def __init__(self):
        super().__init__('interpolation_client')
        self.cli = self.create_client(GoToPosition, 'jint_control_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GoToPosition.Request()

    def send_request(self):
        self.req.translation = float(sys.argv[1])
        self.req.first_rotation = float(sys.argv[2])
        self.req.second_rotation = float(sys.argv[3])
        self.req.time = float(sys.argv[4])

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
