import time

import rclpy
from interpolation_interfaces.srv import ointGoToPosition
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion



class Service(Node):
    def __init__(self):
        super().__init__('oint_interpolation_service')
        self.srv = self.create_service(GoToPosition, 'oint_control_service',
                                       self.interpolation_service_callback)
        self.setup_publisher()
        self.start_position = [0,0,0]
        self.start_orientation = [0,0,0]


    def setup_publisher(self) -> None:
        self.pose_stamped_publisher = self.create_publisher(
            PoseStamped, 'pose_stamped', QoSProfile(depth=10))
        self.pose_stamped = PoseStamped()
        
    def interpolate(self, end_pos, time, T, iteration, start_pos=0) -> float:
        return start_pos + (end_pos-start_pos)/time*T*iteration

    def publish_pos(self) -> None:
        now = self.get_clock().now()
        self.pose_stamped_publisher.publish(self.pose_stamped)

     


    def interpolation_service_callback(self, req, res):
        T = 0.1
        steps = int(req.time/T) + 1
        
        res.confirmation = 'confirmed'

        if req.version==1:
            if req.option==linear:           
                for k in range(1, steps):
                    self.pose_stamped.header.frame_id = "base_link"


                    self.pose_stamped.pose.position.x = self.start_position[0]+(req.x_t-self.start_position)/steps*k
                    self.pose_stamped.pose.position.y = self.start_position[1]+(req.y_t-self.start_position)/steps*k
                    self.pose_stamped.pose.position.z = self.start_position[2]+(req.z_t-self.start_position)/steps*k
                    self.pose_stamped.pose.orientation(Quaternion(
                    x=self.start_orientation[0],y=self.start_orientation[1],z=self.start_orientation[2],w=0.0))
                    
                    self.publish_pos()
                    time.sleep(T)
            else:
                pass
        else:
            if req.option==linear:
                for k in range(1, steps):
                    
                    self.pose_stamped.position.x = self.start_position[0]+(req.x_t-self.start_position)/steps*k
                    self.pose_stamped.position.y = self.start_position[1]+(req.y_t-self.start_position)/steps*k
                    self.pose_stamped.position.z = self.start_position[2]+(req.z_t-self.start_position)/steps*k
                    self.pose_stamped.orientation(Quaternion(w=0.0,
                    x=self.start_orientation[0]+(req.r_r-self.start_orientation[0])/steps*k,
                    y=self.start_orientation[1]+(req.p_r-self.start_orientation[1])/steps*k,
                    z=self.start_orientation[2]+(req.y_r-self.start_orientation[2])/steps*k))
                    
                    self.publish_pos(positions)
                    time.sleep(T)
            else:
                pass

        return res


def main(args=None):
    rclpy.init(args=args)
    interpolation_service = Service()
    rclpy.spin(interpolation_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
