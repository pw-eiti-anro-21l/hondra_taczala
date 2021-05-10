import sys

import rclpy
from interpolation_interfaces.srv import OintGoToPosition
from rclpy.node import Node


class OintClient(Node):
    def __init__(self):
        super().__init__('oint_interpolation_client')
        self.cli = self.create_client(OintGoToPosition, 'oint_control_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = OintGoToPosition.Request()

    def send_request(self):
        try:
            self.req.x_t = float(sys.argv[1])
            self.req.y_t = float(sys.argv[2])
            self.req.z_t = float(sys.argv[3])
            self.req.r_r = float(sys.argv[4])
            self.req.p_r = float(sys.argv[5])
            self.req.y_r = float(sys.argv[6])
            self.req.time = float(sys.argv[7])
            if self.req.time<=0:
                print('zle dane')
                sys.exit()
            self.req.option = sys.argv[8]
            self.req.version = sys.argv[9]


            self.future = self.cli.call_async(self.req)
            
        except Exception:
            print('zle dane')
            sys.exit()


def main(args=None):
    rclpy.init(args=args)
    interpolation_client = OintClient()
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
                    f'Client obtaied {interpolation_client.req.x_t}, \
                        {interpolation_client.req.y_t}, \
                            {interpolation_client.req.z_t} \
                                result {response.confirmation}')
            break
        interpolation_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
