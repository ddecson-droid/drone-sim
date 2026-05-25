"""Launch ros_gz_bridge with YAML configuration."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("drone_bringup")
    config_path = os.path.join(pkg_dir, "config", "gz_bridge.yaml")

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock",
    ))

    ld.add_action(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{
            "config_file": config_path,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        arguments=["--ros-args", "--log-level", "info"],
    ))

    return ld
