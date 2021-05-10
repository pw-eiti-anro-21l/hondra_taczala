import time

import rclpy
from interpolation_interfaces.srv import OintGoToPosition
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion,PoseStamped, Point
import PyKDL
from visualization_msgs.msg import MarkerArray, Marker


class Service(Node):
    def __init__(self):
        super().__init__('oint_interpolation_service')
        self.srv = self.create_service(OintGoToPosition, 'oint_control_service',
                                       self.interpolation_service_callback)
        self.setup_publisher()
        self.start_position = [0,0,0]
        self.start_orientation = [0,0,0]
        self.marker = Marker()



    def setup_publisher(self) -> None:
        self.pose_stamped_publisher = self.create_publisher(
            PoseStamped, '/pose_stamped', QoSProfile(depth=10))
        self.marker_publisher = self.create_publisher(
            Marker, '/marker_line', QoSProfile(depth=10))
        
    def interpolate(self, end_pos, time, T, iteration, start_pos=0) -> float:
        return start_5pos + (end_pos-start_pos)/time*T*iteration

    def publish_pos(self, x_D,y_D,z_D,orientation_D) -> None:
        pose_publisher = self.create_publisher(PoseStamped, '/ointPoseStamped', QoSProfile(depth=10))
        pose = PoseStamped()
        now = self.get_clock().now()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = "base_link"

        #na podstawie dokumentacji PoseStamped:
        # pose.pose.position.x = 1.0
        pose.pose.position.x = x_D

        pose.pose.position.y = y_D
        pose.pose.position.z = z_D
        pose.pose.orientation = orientation_D

        
        self.marker.id = 0
        self.marker.header.frame_id="base_link"
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


        point = Point()
        
        point.x = pose.pose.position.x
        point.y = pose.pose.position.y
        point.z = pose.pose.position.z
        self.marker.points.append(point)
    
        pose_publisher.publish(pose)

        self.marker_publisher.publish(self.marker)    

     


    def interpolation_service_callback(self, req, res):
        T = 0.1
        steps = int(req.time/T) + 1
        
        res.confirmation = 'confirmed'

        if req.version=='1':
            if req.option=='linear':           
                for k in range(1, steps):


                    x = self.start_position[0]+(req.x_t-self.start_position[0])/steps*k
                    y = self.start_position[1]+(req.y_t-self.start_position[1])/steps*k
                    z = self.start_position[2]+(req.z_t-self.start_position[2])/steps*k
                    X_ob = self.start_orientation[0]
                    Y_ob = self.start_orientation[1]
                    Z_ob = self.start_orientation[2]
                    rot = PyKDL.Rotation().RPY(X_ob,Y_ob,Z_ob)
                    quat = rot.GetQuaternion()
                    q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                    self.publish_pos(x,y,z,q)
                    time.sleep(T)
            else:
                for k in range(1, steps):
                    smallT = 0.2*req.time
                    smallTsteps = int(smallT/T) + 1
                    if k<smallTsteps:
                        SX = req.x_t-self.start_position[0] 
                        VX = 20*SX/(16*steps)
                        vx = VX*k/smallTsteps
                        x = self.start_position[0]+vx*k/2
                        SY = req.y_t-self.start_position[1]
                        VY = 20*SY/(16*steps)
                        vy = VY*k/smallTsteps
                        y = self.start_position[1]+vy*k/2
                        SZ = req.z_t-self.start_position[2]
                        VZ = 20*SZ/(16*steps)
                        vz = VZ*k/smallTsteps
                        z = self.start_position[2]+vz*k/2
                    elif k > (steps-smallTsteps):
                        SX = req.x_t-self.start_position[0] 
                        VX = 20*SX/(16*steps)
                        vx = VX*(steps-k)/smallTsteps
                        x = self.start_position[0]+VX*(steps-2*smallTsteps)+2*smallTsteps*VX/2 - (steps-k)*vx/2
                        SY = req.y_t-self.start_position[1]
                        VY = 20*SY/(16*steps)
                        vy = VY*(steps-k)/smallTsteps
                        y = self.start_position[1]+VY*(steps-2*smallTsteps)+2*smallTsteps*VY/2 - (steps-k)*vy/2
                        SZ = req.z_t-self.start_position[2]
                        VZ = 20*SZ/(16*steps)
                        vz = VZ*(steps-k)/smallTsteps
                        z = self.start_position[2]+VZ*(steps-2*smallTsteps)+2*smallTsteps*VZ/2 - (steps-k)*vz/2
                    else:
                        SX = req.x_t-self.start_position[0]
                        VX = 20*SX/(16*steps)
                        VY = 20*SY/(16*steps)
                        VZ = 20*SZ/(16*steps)
                        SY = req.y_t-self.start_position[1]
                        SZ = req.z_t-self.start_position[2]
                        x = self.start_position[0]+VX*(k-smallTsteps) + smallTsteps*VX/2
                        y = self.start_position[1]+VY*(k-smallTsteps) + smallTsteps*VY/2
                        z = self.start_position[2]+VZ*(k-smallTsteps) + smallTsteps*VZ/2
                    X_ob = self.start_orientation[0]
                    Y_ob = self.start_orientation[1]
                    Z_ob = self.start_orientation[2]
                    rot = PyKDL.Rotation().RPY(X_ob,Y_ob,Z_ob)
                    quat = rot.GetQuaternion()
                    q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                    self.publish_pos(x,y,z,q)
                    time.sleep(T)

                    


        else:
            if req.option=='linear':
                for k in range(1, steps):
                    
                    x = self.start_position[0]+(req.x_t-self.start_position[0])/steps*k
                    y = self.start_position[1]+(req.y_t-self.start_position[1])/steps*k
                    z = self.start_position[2]+(req.z_t-self.start_position[2])/steps*k
                    rot = PyKDL.Rotation().RPY(float((self.start_orientation[0]+(req.r_r-self.start_orientation[0])/steps*k)),float((self.start_orientation[1]+(req.p_r-self.start_orientation[1])/steps*k)),float((self.start_orientation[2]+(req.y_r-self.start_orientation[2])/steps*k)))
                    quat = rot.GetQuaternion()
                    X_ob = float(self.start_orientation[0]+(req.r_r-self.start_orientation[0])/steps*k)
                    Y_ob = float(self.start_orientation[1]+(req.p_r-self.start_orientation[1])/steps*k)
                    Z_ob = float(self.start_orientation[2]+(req.y_r-self.start_orientation[2])/steps*k)
                    q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                    self.publish_pos(x,y,z,q)
                    time.sleep(T)
            else:
                for k in range(1, steps):
                    smallT = 0.2*req.time
                    smallTsteps = int(smallT/T) + 1
                    if k<smallTsteps:
                        SX = req.x_t-self.start_position[0] 
                        VX = 20*SX/(16*steps)
                        vx = VX*k/smallTsteps
                        x = self.start_position[0]+vx*k/2
                        SY = req.y_t-self.start_position[1]
                        VY = 20*SY/(16*steps)
                        vy = VY*k/smallTsteps
                        y = self.start_position[1]+vy*k/2
                        SZ = req.z_t-self.start_position[2]
                        VZ = 20*SZ/(16*steps)
                        vz = VZ*k/smallTsteps
                        z = self.start_position[2]+vz*k/2

                        SROTX = req.r_r-self.start_orientation[0]
                        VROTX = 20*SROTX/(16*steps)
                        vROTx = VROTX*k/smallTsteps
                        SROTY = req.p_r-self.start_orientation[1]
                        VROTY = 20*SROTY/(16*steps)
                        vROTy = VROTY*k/smallTsteps
                        SROTZ = req.y_r-self.start_orientation[2]
                        VROTZ = 20*SROTZ/(16*steps)
                        vROTz = VROTZ*k/smallTsteps
                        
                        X_ob = float(self.start_orientation[0]+vROTx*k/2)
                        Y_ob = float(self.start_orientation[1]+vROTy*k/2)
                        Z_ob = float(self.start_orientation[2]+vROTz*k/2)
                        rot = PyKDL.Rotation().RPY(X_ob,Y_ob,Z_ob)
                        quat = rot.GetQuaternion()
                        q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                        self.publish_pos(x,y,z,q)
                        time.sleep(T)



                    elif k > (steps-smallTsteps):
                        SX = req.x_t-self.start_position[0] 
                        VX = 20*SX/(16*steps)
                        vx = VX*(steps-k)/smallTsteps
                        x = self.start_position[0]+VX*(steps-2*smallTsteps)+2*smallTsteps*VX/2 - (steps-k)*vx/2
                        SY = req.y_t-self.start_position[1]
                        VY = 20*SY/(16*steps)
                        vy = VY*(steps-k)/smallTsteps
                        y = self.start_position[1]+VY*(steps-2*smallTsteps)+2*smallTsteps*VY/2 - (steps-k)*vy/2
                        SZ = req.z_t-self.start_position[2]
                        VZ = 20*SZ/(16*steps)
                        vz = VZ*(steps-k)/smallTsteps
                        z = self.start_position[2]+VZ*(steps-2*smallTsteps)+2*smallTsteps*VZ/2 - (steps-k)*vz/2
                        
                        SROTX = req.r_r-self.start_orientation[0]
                        VROTX = 20*SROTX/(16*steps)
                        vROTx = VROTX*(steps-k)/smallTsteps
                        SROTY = req.p_r-self.start_orientation[1]
                        VROTY = 20*SROTY/(16*steps)
                        vROTy = VROTY*(steps-k)/smallTsteps
                        SROTZ = req.y_r-self.start_orientation[2]
                        VROTZ = 20*SROTZ/(16*steps)
                        vROTz = VROTZ*(steps-k)/smallTsteps
                        
                        X_ob = float(self.start_orientation[0]+VROTX*(steps-2*smallTsteps)+2*smallTsteps*VROTX/2 - (steps-k)*vROTx/2)
                        Y_ob = float(self.start_orientation[1]+VROTY*(steps-2*smallTsteps)+2*smallTsteps*VROTY/2 - (steps-k)*vROTy/2)
                        Z_ob = float(self.start_orientation[2]+VROTZ*(steps-2*smallTsteps)+2*smallTsteps*VROTZ/2 - (steps-k)*vROTz/2)
                        rot = PyKDL.Rotation().RPY(X_ob,Y_ob,Z_ob)
                        quat = rot.GetQuaternion()
                        q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                        self.publish_pos(x,y,z,q)
                        time.sleep(T)
                    else:
                        SX = req.x_t-self.start_position[0]
                        VX = 20*SX/(16*steps)
                        VY = 20*SY/(16*steps)
                        VZ = 20*SZ/(16*steps)
                        SY = req.y_t-self.start_position[1]
                        SZ = req.z_t-self.start_position[2]
                        x = self.start_position[0]+VX*(k-smallTsteps) + smallTsteps*VX/2
                        y = self.start_position[1]+VY*(k-smallTsteps) + smallTsteps*VY/2
                        z = self.start_position[2]+VZ*(k-smallTsteps) + smallTsteps*VZ/2
                        SROTX = req.r_r-self.start_orientation[0]
                        VROTX = 20*SROTX/(16*steps)
                        vROTx = VROTX*k/smallTsteps
                        SROTY = req.p_r-self.start_orientation[1]
                        VROTY = 20*SROTY/(16*steps)
                        vROTy = VROTY*k/smallTsteps
                        SROTZ = req.y_r-self.start_orientation[2]
                        VROTZ = 20*SROTZ/(16*steps)
                        vROTz = VROTZ*k/smallTsteps
                        
                        X_ob = float(self.start_orientation[0]+VROTX*(k-smallTsteps) + smallTsteps*VROTX/2)
                        Y_ob = float(self.start_orientation[1]+VROTY*(k-smallTsteps) + smallTsteps*VROTY/2)
                        Z_ob = float(self.start_orientation[2]+VROTZ*(k-smallTsteps) + smallTsteps*VROTZ/2)
                        rot = PyKDL.Rotation().RPY(X_ob,Y_ob,Z_ob)
                        quat = rot.GetQuaternion()
                        q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                        self.publish_pos(x,y,z,q)
                        time.sleep(T)
                    self.publish_pos(x,y,z,q)
                    time.sleep(T)
        self.start_position = [x,y,z]
        self.start_orientation = [X_ob, Y_ob, Z_ob]
        return res


def main(args=None):
    rclpy.init(args=args)
    interpolation_service = Service()
    rclpy.spin(interpolation_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
