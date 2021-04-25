import PyKDL
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, PoseStamped
from rclpy.qos import QoSProfile
from rclpy.clock import ROSClock

import yaml

import os
from ament_index_python.packages import get_package_share_directory

class KDL_D(Node):
    def __init__(self):
        super().__init__('KDL_D')
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
            self.d = 0.0
            self.theta1 = 0.0
            self.theta2 = 0.0

    def get_item_position(self):


        frame1 = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,0), PyKDL.Vector(0, 0, self.d+self.box_height+self.cylinder_radius))
        frame2 = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,self.theta1), PyKDL.Vector(0, 0, 0))
        frame3 = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,self.theta2), PyKDL.Vector(self.a, 0, 0))
        tool = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,0), PyKDL.Vector(self.tool_length, 0, 0))

        allFramesInOne = frame1*frame2*frame3*tool

        return allFramesInOne



    def listener_callback(self, msg):
        self.d = msg.position[0]
        self.theta1 = msg.position[1]
        self.theta2 = msg.position[2]
        
        itemPosition = self.get_item_position()
        xyz = itemPosition.p
        rot = itemPosition.M
        quat= rot.GetQuaternion()

        pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped_KDL_version', QoSProfile(depth=10))
        pose = PoseStamped()
        now = self.get_clock().now()
        pose.header.stamp = ROSClock().now().to_msg()
        pose.header.frame_id = "base_link"

        #na podstawie dokumentacji PoseStamped:
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        pose_publisher.publish(pose)



def main(args=None):
    rclpy.init(args=args)
    KDL_DKIN = KDL_D()
    rclpy.spin(KDL_DKIN)
    KDL_DKIN.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()