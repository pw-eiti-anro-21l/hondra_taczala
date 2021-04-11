import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf_file_name = 'robot.urdf.xml'
  rviz2_file_name = "r2d2.rviz"
  params_path = "dhparams.yaml"

  print("urdf_file_name : {}".format(urdf_file_name))

#   urdf = os.path.join(
#       get_package_share_directory('urdf_model'),
#       urdf_file_name)
#   params = os.path.join(
#       get_package_share_directory('urdf_model'),
#       parameters.yaml)
#   print(urdf)
#   print("a\n")
#   print("a\n")
#   print("a\n")
# #   urdf = 

#     l2package = "urdf_model"
#     path = get_package_share_directory(l2package)
#     absolute_path = os.path.join(get_package_share_directory('urdf_model'), file)
  rviz_path = os.path.join(
        get_package_share_directory('urdf_model'),
        rviz2_file_name)
  urdf_path = os.path.join(
        get_package_share_directory('urdf_model'),
        urdf_file_name)
  params_path = os.path.join(
        get_package_share_directory('urdf_model'),
        params_path)

  return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[{'use_sim_time': use_sim_time}],
          arguments=[urdf_path]),
      Node(
          package='urdf_model',
          executable='state_publisher',
          name='state_publisher',
          output='screen',
          parameters=[params_path]),
      Node(
          package='rviz2',
          executable='rviz2',
          name='robot',
          output='screen',
          parameters=[{'use_sim_time': use_sim_time,}],
          arguments=['-d',rviz_path]),
  ])