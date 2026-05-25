"""
Top-level launch file for PX4 + Gazebo Harmonic + ROS 2 drone simulation.

Starts:
  1. robot_state_publisher (URDF for RViz2)
  2. px4_tf_broadcaster   (NED -> ENU TF)
  3. px4_status_monitor   (vehicle status logging)
  4. ros_gz_bridge        (Gazebo <-> ROS 2 topic bridge)
  5. RViz2                (visualization)

PX4 SITL + Gazebo must be started separately in another terminal:
  cd ~/PX4-Autopilot && make px4_sitl gz_x500

Usage:
  ros2 launch drone_bringup simulation.launch.py
  ros2 launch drone_bringup simulation.launch.py use_rviz:=false use_camera:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # ========== Launch Arguments ==========
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation clock",
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2",
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_camera",
        default_value="false",
        description="Bridge camera topics (requires camera-equipped model)",
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_lidar",
        default_value="false",
        description="Bridge lidar topics (requires lidar-equipped model)",
    ))
    ld.add_action(DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("drone_bringup"),
            "config", "rviz_config.rviz"
        ),
        description="Path to RViz2 config file",
    ))

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    # ========== 1. Robot State Publisher (URDF -> TF static + /robot_description) ==========
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("drone_description"),
                    "launch", "description.launch.py",
                ])
            ]),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        )
    )

    # ========== 2. PX4 Utility Nodes (direct Node calls) ==========
    ld.add_action(
        Node(
            package="drone_utils",
            executable="px4_tf_broadcaster",
            name="px4_tf_broadcaster",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )
    ld.add_action(
        Node(
            package="drone_utils",
            executable="px4_status_monitor",
            name="px4_status_monitor",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    # ========== 3. ros_gz_bridge (Gazebo <-> ROS 2) ==========
    # Delayed to allow Gazebo to fully start
    ld.add_action(
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare("drone_bringup"),
                            "launch", "gz_bridge.launch.py",
                        ])
                    ]),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                    }.items(),
                ),
            ],
        )
    )

    # ========== 4. RViz2 ==========
    ld.add_action(
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare("drone_bringup"),
                            "launch", "rviz.launch.py",
                        ])
                    ]),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                    }.items(),
                ),
            ],
        )
    )

    return ld
