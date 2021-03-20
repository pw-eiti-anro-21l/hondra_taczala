import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('turtle_maneuvering'),
        'config',
        'params.yaml'
    )

    node = Node(
        package='turtle_maneuvering',
        executable='param_talker',
        name='keyboard_param_node',
        parameters=[config]
    )

    ld.add_action(node)
    return ld
