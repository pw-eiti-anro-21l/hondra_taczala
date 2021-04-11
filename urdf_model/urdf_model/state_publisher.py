#! /usr/bin/env python
from math import sin, cos, pi
import pandas as pd
from ament_index_python.packages import get_package_share_directory

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
import os
# from urdf_creator import createUrdf
import csv
def createUrdf(params):
  # data = pd.read_csv('../resource/dh.csv')

  a_table = []
  d_table = []
  alpha_table = []
  theta_table = []
  urdf_file_name = 'robot.urdf.xml'
  urdf_path = os.path.join(
    get_package_share_directory('urdf_model'),
    urdf_file_name)

  #zapisanie wartości tabeli DH
  for i in range(3):
    a_table.append(params[f'a{i+1}'])
    d_table.append(params[f"d{i+1}"])
    alpha_table.append(params[f'alpha{i+1}'])
    theta_table.append(params[f"theta{i+1}"])
    


  with open(urdf_path,'w') as file:
    file.write('<robot name="robot_lab2">\n')   
    #napisanie podstawy pod robota
    file.write("""
<link name="part1">
<visual>
<origin xyz="0 0 0.2"/>
<geometry>
<box size="2 2 0.4"/>
</geometry>
<material name="green">
<color rgba="0 1 0 1"/>
</material>
</visual>
</link>
""")
    for i, cos in enumerate(a_table):

      a = a_table[i]
      d = d_table[i]
      alpha = alpha_table[i]
      theta = theta_table[i]


      #ustawienie parametrów
      if a!=0:
          joint_type = "revolute"
          geometry_type_ = "cylinder"
      elif d != 0:
              joint_type = "prismatic"
              geometry_type_ = "cylinder"
      else:
          joint_type = "revolute"
          geometry_type_ = "cylinder"
      
      len_ = math.sqrt(a**2+d**2)
      geometry_parameters = f'radius="0.05" length="{len_}"'
      if len_ !=0:
          ang = math.acos(d/len_)
      else:
          ang = 0
      if i==0:
        d=d-1.2
      #napisanie jointa
      file.write(f"""
<joint name="joint_{str(i+1)+str(i+2)}" type="{joint_type}">
<origin xyz="{a} 0 {d}" rpy="{alpha} 0 {theta}"/>
<parent link="part{i+1}"/>
<child link="part{i+2}"/>
<axis xyz="0 0 1" />
<limit upper="3.14" lower="-3.14" effort="10" velocity="10" />
</joint>
""")
          #napisanie części głównych
      if i==0:
        d+=1.2
      file.write(f"""
<link name="part{i+2}">
  <visual>
      <origin xyz="{(-1)*a/2} 0 {-d/2}" rpy="0 {ang} 0"/>
      <geometry>
      <{geometry_type_} {geometry_parameters}/>
      </geometry>
      <material name="green">
      <color rgba="0 1 0 1" />
      </material>
  </visual>
</link>
  """
          )
    file.write(f"""
<joint name="joint_45" type="revolute">
<origin xyz="0 0 0" rpy="0 0 {params["theta4"]}"/>
<parent link="part4"/>
<child link="part5"/>
<axis xyz="0 0 1" />
<limit upper="3.14" lower="-3.14" effort="10" velocity="10" />
</joint>   
<link name="part5">
  <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
      <box size="0.2 0.2 0.2"/>
      </geometry>
      <material name="green">
      <color rgba="0 1 0 1" />
      </material>
  </visual>
</link>
""")
    file.write('</robot>\n')


class StatePublisher(Node):

  def __init__(self):
    rclpy.init()
    super().__init__('state_publisher')

    self.params={}
    #bez deklaracji poniżej, wyrzuca błąd w dodawaniu
    self.declare_parameters(
      namespace='',
      parameters=[
        ('theta1', 0.0),
        ('theta2', 0.0),
        ('theta3', 0.0),
        ('a1', 0.0),
        ('a2', 0.0),
        ('a3', 1.0),
        ('d1', 1.0),
        ('d2', 0.0),
        ('d3', 0.0),
        ('alpha1', 0.0),
        ('alpha2', 0.0),
        ('alpha3', 0.0),

      ])
    self.params['a1'] = self.get_parameter('a1')._value#bez value przypisywane obiekty które psuły działania matematyczne
    self.params['a2'] = self.get_parameter('a2')._value
    self.params['a3'] = self.get_parameter('a3')._value
    self.params['d1'] = self.get_parameter('d1')._value
    print(self.params['d1'])
    print(self.params['d1'])
    print(self.params['d1'])
    print(self.params['d1'])
    print(self.params['d1'])
    print(self.params['d1'])
    print(self.params['d1'])

    self.params['d2'] = self.get_parameter('d2')._value
    self.params['d3'] = self.get_parameter('d3')._value
    self.params['alpha1'] = self.get_parameter('alpha1')._value
    self.params['alpha2'] = self.get_parameter('alpha2')._value
    self.params['alpha3'] = self.get_parameter('alpha3')._value
    self.params['theta1'] = self.get_parameter('theta1')._value
    self.params['theta2'] = self.get_parameter('theta2')._value
    self.params['theta3'] = 0.0
    self.params['theta4'] = self.get_parameter('theta3')._value

    createUrdf(self.params)
    qos_profile = QoSProfile(depth=10)
    self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
    self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
    self.nodeName = self.get_name()
    self.get_logger().info("{0} started".format(self.nodeName))

    degree = pi / 180.0
    loop_rate = self.create_rate(30)


    # robot state
    self.theta_1 = 3.14/2#tego nie zmieniamy
    self.theta_2 = 0.0
    self.theta_3 = 0.0#tego nie zmieniamy
    self.theta_4 = -2.14/2

    # message declarations
    self.odom_trans = TransformStamped()
    self.odom_trans.header.frame_id = 'odom'
    self.odom_trans.child_frame_id = 'part1'
    self.joint_state = JointState()

    try:
      while rclpy.ok():
          rclpy.spin_once(self)

          # update joint_state
          now = self.get_clock().now()
          self.joint_state.header.stamp = now.to_msg()
          self.joint_state.name = ['joint_12', 'joint_23', 'joint_34', 'joint_45']
          self.joint_state.position = [self.theta_1, self.theta_2, self.theta_3, self.theta_4]

          # update transform
          # (moving in a circle with radius=2)
          self.odom_trans.header.stamp = now.to_msg()
          
          self.odom_trans.transform.translation.x = 0.0
          self.odom_trans.transform.translation.y = 0.0
          self.odom_trans.transform.translation.z = 0.0
          self.odom_trans.transform.rotation = \
            euler_to_quaternion(0.0, 0.0, 0.0) # roll,pitch,yaw


          self.joint_pub.publish(self.joint_state)
          self.broadcaster.sendTransform(self.odom_trans)

          self.theta_2 += 0.001 % (2*pi)

          self.theta_4 += 0.01 % (2*pi)

          # loop_rate.sleep()
    except KeyboardInterrupt:
      pass

def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
  node = StatePublisher()

if __name__ == '__main__':
  main()