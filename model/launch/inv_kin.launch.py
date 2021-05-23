import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    xacro_file_name = "move.urdf.xacro.xml"
    urdf_file_name = "move-model.urdf.xml"
    rviz_file_name = "interpolation.rviz"
    xacro = os.path.join(get_package_share_directory("model"), xacro_file_name)
    urdf = os.path.join(get_package_share_directory("model"), urdf_file_name)
    rviz = os.path.join(get_package_share_directory("model"), rviz_file_name)
    os.system(f"xacro {xacro} -o {urdf}")

    with open(urdf, "r") as infp:
        robot_desc = infp.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time, "robot_description": robot_desc}
                ],
                arguments=[urdf],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["-d", rviz],
            ),
            Node(package="model", executable="ikin", name="ikin", output="screen"),
        ]
    )
