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




    return LaunchDescription([

    Node(
        package='turtle_maneuvering',
        executable='move',
        output='screen',
        prefix=["gnome-terminal ", "-- "]
        
        
    ),
    Node(
        package='turtle_maneuvering',
        executable='param_talker',
        parameters=[config],
        output='screen'

    ),
    Node(
        package='turtlesim',
        executable='turtlesim_node'
    )
    
    ]
    )
    # ld.add_action(node)
    # ld.add_action(node1)
    # ld.add_action(node2)
    # return ld
