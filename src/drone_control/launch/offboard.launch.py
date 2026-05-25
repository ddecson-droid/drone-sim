"""Launch offboard control node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "takeoff_height", default_value="-5.0",
        description="Takeoff height in meters (NED, negative = up)",
    ))
    ld.add_action(DeclareLaunchArgument(
        "hover_time", default_value="10.0",
        description="Hover time in seconds",
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation time",
    ))

    ld.add_action(Node(
        package="drone_control",
        executable="offboard_control",
        name="offboard_control",
        output="screen",
        parameters=[{
            "takeoff_height": LaunchConfiguration("takeoff_height"),
            "hover_time": LaunchConfiguration("hover_time"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    ))

    return ld
