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

    def publisher(self):
        self.joint_publisher = self.create_publisher(
            JointState, 'joint_states', QoSProfile(depth=10))
        joint_state = JointState()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['a', 'b', 'c']
        joint_state.position = [10.0, 10.0, 10.0]
        self.joint_publisher.publish(joint_state)

    def interpolation_service_callback(self, req, res):
        res.confirmation = 'confirmed'
        self.get_logger().info(f't:{req.translation},\
             r1:{req.first_rotation}, r2:{req.second_rotation}, \
                 time:{req.time}')
        self.publisher()
        return res


def main(args=None):
    rclpy.init(args=args)
    interpolation_service = Service()
    rclpy.spin(interpolation_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
