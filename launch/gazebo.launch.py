from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gz_launch_path = (
        Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py"
    )

    models_dir_path = Path(get_package_share_directory("sketch_follower")) / "models"
    model_path = models_dir_path / "model.sdf"

    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(models_dir_path)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz_launch_path)),
                launch_arguments={
                    "gz_args": [str(model_path)],
                    "on_exit_shutdown": "True",
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[],
                remappings=[],
                output="screen",
            ),
        ]
    )
