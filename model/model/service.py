import time

import rclpy
from interpolation_interfaces.srv import GoToPosition
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState


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

    def interpolate(self, end_pos, time, T, iteration, start_pos=0) -> float:
        return start_pos + (end_pos-start_pos)/time*T*iteration

    def publish_pos(self, position: list) -> None:
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.position = position
        self.joint_publisher.publish(self.joint_state)

    def interpolation_service_callback(self, req, res):
        T = 0.1
        res.confirmation = 'confirmed'
        self.get_logger().info(f't:{req.translation},\
             r1:{req.first_rotation}, r2:{req.second_rotation}, \
                 time:{req.time}')
        
        end_positions = [req.translation, req.first_rotation,
                         req.second_rotation]
        steps = int(req.time/T) + 1
        for k in range(1, steps):
            positions = [self.interpolate(end, req.time, T, k)
                         for end in end_positions]
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
