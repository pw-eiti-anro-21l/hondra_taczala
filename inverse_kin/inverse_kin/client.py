import os
import sys
from math import pi

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from interpolation_interfaces.srv import GoToPosition
from rclpy.node import Node


class Client(Node):
    def __init__(self):
        super().__init__("interpolation_client")
        self.cli = self.create_client(GoToPosition, "jint_control_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = GoToPosition.Request()
        file_name = "model.yaml"
        self.readParams(file_name)

    def readParams(self, f_name) -> None:
        p = os.path.join(get_package_share_directory("inverse_kin"), f_name)
        with open(p, "r") as file:
            params = yaml.load(file, Loader=yaml.FullLoader)
            self.joint_upper_limit = float(params["joint_upper_limit"])

    def send_request(self):
        translation = float(sys.argv[1])
        first_rotation = float(sys.argv[2])
        second_rotation = float(sys.argv[3])
        time = float(sys.argv[4])
        interpolation_type = str(sys.argv[5])
        try:
            if translation > 0 and translation < self.joint_upper_limit:
                self.req.translation = translation
            else:
                raise ValueError()
            if abs(first_rotation) <= pi:
                self.req.first_rotation = first_rotation
            else:
                raise ValueError()
            if abs(second_rotation) <= pi / 2:
                self.req.second_rotation = second_rotation
            else:
                raise ValueError()
            if time > 0:
                self.req.time = time
            else:
                raise ValueError()
            if interpolation_type == "linear" or interpolation_type == "polynomial":
                self.req.interpolation_type = interpolation_type
            else:
                raise ValueError()
        except ValueError:
            raise Exception("Wrong parameters")
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    interpolation_client = Client()
    interpolation_client.send_request()
    interpolation_client.get_logger().info("Interpolating ...")

    while rclpy.ok():
        rclpy.spin_once(interpolation_client)
        if interpolation_client.future.done():
            try:
                response = interpolation_client.future.result()
            except Exception as e:
                interpolation_client.get_logger().info(f"Service call failed {e}")
            else:
                interpolation_client.get_logger().info(f"{response.confirmation}")
            break
        interpolation_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
