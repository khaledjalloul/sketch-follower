from pathlib import Path
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_file = (
        Path(get_package_share_directory("sketch_follower")) / "config" / "config.rviz"
    )

    robot_controllers = (
        Path(get_package_share_directory("sketch_follower"))
        / "config"
        / "velocity_control.yaml"
    )

    xacro_file_path = (
        Path(get_package_share_directory("sketch_follower"))
        / "robot_description"
        / "urdf"
        / "model.xacro"
    )
    robot_description = ParameterValue(Command(["xacro ", str(xacro_file_path)]))

    remappings = [
        (
            "/velocity_controller/commands",
            "/sketch_follower/velocity_controller/commands",
        ),
        (
            "/joint_states",
            "/sketch_follower/joint_states",
        ),
    ]

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[str(robot_controllers)],
        remappings=remappings,
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
        remappings=remappings,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(rviz_config_file)],
        remappings=remappings,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        remappings=remappings,
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--param-file", str(robot_controllers)],
        remappings=remappings,
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    cursor_publisher_node = Node(
        package="sketch_follower",
        executable="cursor_publisher",
        name="cursor_publisher",
        output="screen",
    )

    controller_node = Node(
        package="sketch_follower",
        executable="controller",
        name="controller",
        output="screen",
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_pub_node,
            robot_controller_spawner,
            cursor_publisher_node,
            controller_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_state_broadcaster_after_robot_controller_spawner,
        ]
    )
