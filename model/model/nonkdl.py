import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, PoseStamped
from rclpy.qos import QoSProfile

import numpy as np
from math import sin, cos
from scipy.spatial.transform import Rotation as R

import yaml

import os
from ament_index_python.packages import get_package_share_directory


class nonKDL_D(Node):
    def __init__(self):
        super().__init__('nonKDL_D')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        file_name = 'model_link_params.yaml'
        self.readParams(file_name)

    def readParams(self, f_name):
        p = os.path.join(get_package_share_directory('model'), f_name)
        with open(p, 'r') as file:
            params = yaml.load(file, Loader=yaml.FullLoader)
            self.a = float(params['a'])
            self.box_height = float(params['box_z'])
            self.cylinder_radius = float(params['cylinder_radius'])
            self.tool_length = float(params['tool_x'])

    def rotX(self, alpha) -> np.array:
        return np.array([[1, 0, 0, 0],
                         [0, cos(alpha), -sin(alpha), 0],
                         [0, sin(alpha), cos(alpha), 0],
                         [0, 0, 0, 1]
                         ])

    def transX(self, a) -> np.array:
        return np.array([[1, 0, 0, a],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]
                         ])

    def rotZ(self, theta) -> np.array:
        return np.array([[cos(theta), -sin(theta), 0, 0],
                         [sin(theta), cos(theta), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]
                         ])

    def transZ(self, d) -> np.array:
        return np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, d],
                         [0, 0, 0, 1]
                         ])

    def get_item_position(self) -> np.array:
        dh = [[0, self.d, 0, 0],
              [0, 0, 0, self.theta1],
              [self.a, 0, 0, self.theta2]]
        align_z = self.box_height + self.cylinder_radius
        transformation = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, align_z],
                                   [0, 0, 0, 1]
                                   ])
        for row in dh:
            a, d, alpha, th = row
            rotationX = self.rotX(alpha)
            translationX = self.transX(a)
            rotationZ = self.rotZ(th)
            translationZ = self.transZ(d)

            frame = rotationX @ translationX @ rotationZ @ translationZ
            transformation = transformation @ frame
        transformation = transformation @ self.transX(self.tool_length)
        return transformation

    def listener_callback(self, msg):
        self.d = msg.position[0]
        self.theta1 = msg.position[1]
        self.theta2 = msg.position[2]

        itemPosition = self.get_item_position()
        xyz = itemPosition[:3, 3]
        quat = R.from_matrix(itemPosition[:3, :3]).as_quat()

        pose_publisher = self.create_publisher(
            PoseStamped, '/pose_stamped_nonKDL_version', QoSProfile(depth=10))
        pose = PoseStamped()
        pose.header.frame_id = "base_link"

        # na podstawie dokumentacji PoseStamped:
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        # pose.pose.orientation = Quaternion(x=0.0,y=0.0,z=0.0,w=0.0)
        pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        pose_publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    nonKDL_DKIN = nonKDL_D()
    rclpy.spin(nonKDL_DKIN)
    nonKDL_DKIN.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
