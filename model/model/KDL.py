import PyKDL
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, PoseStamped
from rclpy.qos import QoSProfile



class KDL_D(Node):
    def __init__(self):
            super().__init__('KDL_D')
            self.subscription = self.create_subscription(
                JointState,
                'joint_states',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning

    def get_item_position(self):
        # chain = PyKDL.Chain()
        # frame1 = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,0), PyKDL.Vector(0, 0, self.d))
        # joint1 = PyKDL.Joint()
        # chain.addSegment(PyKDL.Segment(joint1,frame1))
        # frame2 = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,self.theta1), PyKDL.Vector(0, 0, 0))
        # joint2 = PyKDL.Joint()
        # chain.addSegment(PyKDL.Segment(joint2,frame2))

        # frame3 = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,self.theta2), PyKDL.Vector(2, 0, 0))
        # joint3 = PyKDL.Joint()
        # chain.addSegment(PyKDL.Segment(joint3,frame3))
        
        # fk=PyKDL.ChainFkSolverPos_recursive(chain)
        # finalFrame=PyKDL.Frame()
        # arr = PyKDL.JntArray(3)
        # arr[0] = self.d
        # arr[1] = self.theta1
        # arr[2] = self.theta2
        a = 3
        theta_z_p = 0
        frame1 = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,0), PyKDL.Vector(0, 0, self.d+0.2))
        frame2 = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,self.theta1), PyKDL.Vector(0, 0, 0))
        frame3 = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,self.theta2), PyKDL.Vector(a, 0, 0))
        tool = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,0), PyKDL.Vector(0.5, 0, 0))

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
        pose.header.frame_id = "base_link"

        #na podstawie dokumentacji PoseStamped:
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        # pose.pose.orientation = Quaternion(x=0.0,y=0.0,z=0.0,w=0.0)
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