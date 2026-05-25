"""Launch utility nodes: px4_tf_broadcaster and px4_status_monitor."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock",
    ))

    ld.add_action(Node(
        package="drone_utils",
        executable="px4_tf_broadcaster",
        name="px4_tf_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    ))

    ld.add_action(Node(
        package="drone_utils",
        executable="px4_status_monitor",
        name="px4_status_monitor",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    ))

    return ld
