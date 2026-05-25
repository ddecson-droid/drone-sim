"""Launch keyboard teleop node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation time",
    ))

    ld.add_action(Node(
        package="drone_control",
        executable="keyboard_teleop",
        name="keyboard_teleop",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    ))

    return ld
