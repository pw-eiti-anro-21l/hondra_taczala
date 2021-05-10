import os
import time

import PyKDL
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Point
from interpolation_interfaces.srv import GoToPosition
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


class Service(Node):
    def __init__(self):
        super().__init__('interpolation_service')
        self.srv = self.create_service(GoToPosition, 'jint_control_service',
                                       self.interpolation_service_callback)
        file_name = 'model.yaml'
        self.readParams(file_name)
        self.setup_publisher()
        self.setup_marker()

    def setup_marker(self):
        self.marker = Marker()
        self.marker.id = 0
        self.marker.header.frame_id = "base_link"
        self.marker.type = 7
        self.marker.header.stamp
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.01
        self.marker.scale.y = 0.01
        self.marker.scale.z = 0.01
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0
        self.marker_publisher = self.create_publisher(
            Marker, '/marker_line', QoSProfile(depth=10))

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

    def readParams(self, f_name) -> None:
        p = os.path.join(get_package_share_directory('model'), f_name)
        with open(p, 'r') as file:
            params = yaml.load(file, Loader=yaml.FullLoader)
            self.a = float(params['a'])
            self.box_height = float(params['boxZ'])
            self.cylinder_radius = float(params['cylinder_radius'])
            self.tool_length = float(params['tool_x'])
            self.d = 0.0
            self.theta1 = 0.0
            self.theta2 = 0.0

    def get_item_position(self) -> PyKDL.Frame:
        frame1 = PyKDL.Frame(
            PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(
                0, 0, self.d+self.box_height+self.cylinder_radius))
        frame2 = PyKDL.Frame(
            PyKDL.Rotation.RPY(0, 0, self.theta1), PyKDL.Vector(0, 0, 0))
        frame3 = PyKDL.Frame(
            PyKDL.Rotation.RPY(0, 0, self.theta2), PyKDL.Vector(self.a, 0, 0))
        tool = PyKDL.Frame(
            PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(self.tool_length, 0, 0))

        allFramesInOne = frame1*frame2*frame3*tool

        return allFramesInOne

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
            self.publish_markers(positions)
            time.sleep(T)

        return res

    def publish_markers(self, positions):
        [self.d, self.theta1, self.theta2] = [x for x in positions]
        itemPosition = self.get_item_position().p
        point = Point()
        [point.x, point.y, point.z] = [x for x in itemPosition]
        self.marker.points.append(point)
        self.marker_publisher.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    interpolation_service = Service()
    rclpy.spin(interpolation_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
