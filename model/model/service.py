import time

import rclpy
from geometry_msgs.msg import PoseStamped
from interpolation_interfaces.srv import GoToPosition
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker


class Service(Node):
    def __init__(self):
        super().__init__('interpolation_service')
        self.srv = self.create_service(GoToPosition, 'jint_control_service',
                                       self.interpolation_service_callback)
        self.setup_publisher()

    def setup_publisher(self) -> None:
        self.joint_publisher = self.create_publisher(
            JointState, 'joint_states', QoSProfile(depth=10))
        self.joint_state = JointState()
        self.joint_state.name = ['first_to_translator',
                                 'translator_to_second', 'second_to_tool']

    def interpolate_linear(
            self, end_pos, time, T, iteration, start_pos=0) -> float:
        return start_pos + (end_pos-start_pos)/time*T*iteration

    def interpolate_polynomial(
            self, end_pos, time, T, iteration, start_pos=0) -> float:

        t = T*iteration

        a0 = start_pos
        a1 = 0
        a2 = 3*(end_pos - start_pos)/time**2
        a3 = -2*(end_pos - start_pos)/time**3

        return a0 + a1*t + a2*t**2 + a3*t**3

    def publish_pos(self, position: list) -> None:
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.position = position
        self.joint_publisher.publish(self.joint_state)

    def interpolation_service_callback(self, req, res):
        T = 0.1
        self.get_logger().info(f'\nt:{req.translation},\n \
            r1:{req.first_rotation},\n \
            r2:{req.second_rotation},\n \
            time:{req.time},\n \
            type: {req.interpolation_type}')
        res.confirmation = 'Interpolated successfully'

        end_positions = [req.translation, req.first_rotation,
                         req.second_rotation]
        steps = int(req.time/T) + 1
        for k in range(1, steps):
            if req.interpolation_type == 'linear':
                positions = [self.interpolate_linear(end, req.time, T, k)
                             for end in end_positions]
            elif req.interpolation_type == 'polynomial':
                positions = [self.interpolate_polynomial(end, req.time, T, k)
                             for end in end_positions]
            else:
                raise ValueError('Wrong interpolation type')
            self.publish_pos(positions)
            time.sleep(T)

        return res


def main(args=None):
    rclpy.init(args=args)
    interpolation_service = Service()
    rclpy.spin(interpolation_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()