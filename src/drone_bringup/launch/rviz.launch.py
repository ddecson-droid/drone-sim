"""Launch RViz2 with the drone simulation configuration."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("drone_bringup")
    config_path = os.path.join(pkg_dir, "config", "rviz_config.rviz")

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock",
    ))
    ld.add_action(DeclareLaunchArgument(
        "rviz_config", default_value=config_path,
        description="Path to RViz configuration file",
    ))

    ld.add_action(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    ))

    return ld
