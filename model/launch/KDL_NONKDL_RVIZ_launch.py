import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file_name = 'move.urdf.xacro.xml'
    urdf_file_name = 'move-model.urdf.xml'
    rviz_file_name = 'r2d2.rviz'
    xacro = os.path.join(
        get_package_share_directory('model'),
        xacro_file_name)
    urdf = os.path.join(
        get_package_share_directory('model'),
        urdf_file_name)
    rviz = os.path.join(
        get_package_share_directory('model'),
        rviz_file_name)
    os.system(f'xacro {xacro} -o {urdf}')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='model',
            executable='KDL',
            name='KDL',
            output='screen'
        ),
        Node(
            package='model',
            executable='nonKDL',
            name='nonKDL',
            output='screen'
        )
    ])
