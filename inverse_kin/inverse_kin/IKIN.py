import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, PoseStamped
from rclpy.qos import QoSProfile
from rclpy.clock import ROSClock
from interpolation_interfaces.srv import OintGoToPosition
import math
import yaml

from math import atan, sin, cos, pi, sqrt, acos, asin, atan2


import os
from ament_index_python.packages import get_package_share_directory


class IKIN(Node):
    def __init__(self):
        super().__init__("IKIN")
        self.subscription = self.create_subscription(
            PoseStamped, "/ointPoseStamped", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        file_name = "model_link_params.yaml"
        self.readParams(file_name)
        self.x = 0
        self.y = 0
        self.z = 0

    def readParams(self, f_name):
        p = os.path.join(get_package_share_directory("inverse_kin"), f_name)
        with open(p, "r") as file:
            params = yaml.load(file, Loader=yaml.FullLoader)
            self.a = float(params["a"])
            self.box_height = float(params["box_z"])
            self.cylinder_radius = float(params["cylinder_radius"])
            self.tool_length = float(params["tool_x"])
            self.d_start = float(params["d"])
            self.d = 0.0
            self.theta1 = 0.0
            self.theta2 = 0.0

    def computeParams(self, x, y, z):
        th2_1 = acos(
            (x ** 2 + y ** 2 - self.a ** 2 - self.tool_length ** 2)
            / (2 * self.a * self.tool_length)
        )
        th1_1 = -asin(self.tool_length * sin(th2_1) / (sqrt(x ** 2 + y ** 2))) + atan2(
            y, x
        )

        th2_2 = -acos(
            (x ** 2 + y ** 2 - self.a ** 2 - self.tool_length ** 2)
            / (2 * self.a * self.tool_length)
        )
        th1_2 = -asin(self.tool_length * sin(th2_2) / (sqrt(x ** 2 + y ** 2))) + atan2(
            y, x
        )

        th2_3 = acos(
            (x ** 2 + y ** 2 - self.a ** 2 - self.tool_length ** 2)
            / (2 * self.a * self.tool_length)
        )
        th1_3 = (
            pi
            - asin(self.tool_length * sin(th2_3) / (sqrt(x ** 2 + y ** 2)))
            + atan2(y, x)
        )

        th2_4 = -acos(
            (x ** 2 + y ** 2 - self.a ** 2 - self.tool_length ** 2)
            / (2 * self.a * self.tool_length)
        )
        th1_4 = (
            pi
            - asin(self.tool_length * sin(th2_4) / (sqrt(x ** 2 + y ** 2)))
            + atan2(y, x)
        )

        difference1 = th2_1 - self.theta2 + th1_1 - self.theta1
        difference2 = th2_2 - self.theta2 + th1_2 - self.theta1
        difference3 = th2_3 - self.theta2 + th1_3 - self.theta1
        difference4 = th2_4 - self.theta2 + th1_4 - self.theta1

        best_difference = min(difference1, difference2, difference3, difference4)

        if best_difference == difference1:
            theta1 = th1_1
            theta2 = th2_1
        elif best_difference == difference2:
            theta1 = th1_2
            theta2 = th2_2
        elif best_difference == difference3:
            theta1 = th1_3
            theta2 = th2_3
        elif best_difference == difference4:
            theta1 = th1_4
            theta2 = th2_4
        if abs(theta2) > 1.57:
            raise Exception
        self.theta1 = theta1
        self.theta2 = theta2
        self.d = z - self.box_height - self.cylinder_radius

    def listener_callback(self, msg):
        msg = msg.pose
        self.joint_publisher = self.create_publisher(
            JointState, "joint_states", QoSProfile(depth=10)
        )
        self.joint_state = JointState()

        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = [
            "first_to_translator",
            "translator_to_second",
            "second_to_tool",
        ]
        self.joint_state.position = [self.d, self.theta1, self.theta2]
        if msg.position.z <= (
            self.d_start + self.cylinder_radius + self.box_height
        ) and msg.position.z >= (self.cylinder_radius + self.box_height):
            # if math.sqrt((msg.position.x)**2 + (msg.position.y)**2) <= self.a + self.tool_length and math.sqrt((msg.position.x)**2 + (msg.position.y)**2) >= self.a:

            try:
                self.computeParams(msg.position.x, msg.position.y, msg.position.z)
                self.joint_publisher.publish(self.joint_state)

            except:
                self.get_logger().warn("Nie ma dobrego ustawienia stawów")
            # if len(params) != 0:

            #     self.theta1 = params[0][0]
            #     self.theta2 = params[0][1]
            #     self.d = params[0][2]

            #     pass
            # else:
            #     self.get_logger().warn(
            #         "Zła odleglosc od robota - nie można określić położenia stawów"
            #     )
            #     self.d = 2.0
            #     self.theta1 = 1.0
            #     self.theta2 = 1.0
        else:
            self.get_logger().warn("Zła wysokość - nie można określić położenia stawów")

        # update transform
        # (moving in a circle with radius=2)


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(
        pitch / 2
    ) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(
        pitch / 2
    ) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    Ikin = IKIN()
    rclpy.spin(Ikin)
    Ikin.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
