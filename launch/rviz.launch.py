from pathlib import Path
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_file = (
        Path(get_package_share_directory("sketch_follower")) / "config" / "config.rviz"
    )

    xacro_file = (
        Path(get_package_share_directory("sketch_follower"))
        / "robot_description"
        / "urdf"
        / "3d_4dof.xacro"
    )

    urdf_output = ParameterValue(Command(["xacro ", str(xacro_file)]))

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config_file)],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_description": urdf_output,
                    },
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
        ]
    )
