"""
gz_bridge.launch.py -- 启动 ros_gz_bridge 节点。

============================================================
ros_gz_bridge 做什么？
============================================================
Gazebo Harmonic 使用 gz-transport 通信 (非 ROS 2 原生)
ros_gz_bridge 把 Gazebo 的话题转换为 ROS 2 话题:

  Gazebo Transport              →  ROS 2
  ───────────────                 ──────
  /clock          (gz.msgs.Clock)     → /clock            (rosgraph_msgs/Clock)
  /camera/image   (gz.msgs.Image)     → /camera/image_raw (sensor_msgs/Image)
  /lidar/scan     (gz.msgs.LaserScan) → /scan             (sensor_msgs/LaserScan)

============================================================
配置文件
============================================================
话题映射关系定义在 config/gz_bridge.yaml 中
ros_gz_bridge 的 parameter_bridge 可执行文件读取此配置

============================================================
为什么需要 us_sim_time?
============================================================
Gazebo 的 /clock 话题提供仿真时间 (而非系统时间)
所有 ROS 2 节点必须使用仿真时间才能与 Gazebo 同步
"""
import os  # 用于路径拼接

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration  # 读取启动参数值
from launch_ros.actions import Node  # 启动 ROS 2 节点


def generate_launch_description():
    # ============================================================
    # 查找配置文件路径
    # ============================================================
    # get_package_share_directory: 获取包安装后的 share 目录
    # 例如: ~/drone_ws/install/drone_bringup/share/drone_bringup/
    pkg_dir = get_package_share_directory("drone_bringup")
    config_path = os.path.join(pkg_dir, "config", "gz_bridge.yaml")

    ld = LaunchDescription()

    # ============================================================
    # 声明参数
    # ============================================================
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock",
    ))

    # ============================================================
    # 启动 ros_gz_bridge 节点
    # ============================================================
    # package="ros_gz_bridge": ROS 2 官方提供的 Gazebo 桥接包
    # executable="parameter_bridge": 通过配置文件定义桥接规则
    # config_file: 指向 gz_bridge.yaml
    ld.add_action(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",  # 参数化桥接器
        name="ros_gz_bridge",           # 节点名
        output="screen",                # 日志输出到终端
        parameters=[{
            "config_file": config_path,  # ★ 桥接配置文件
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        arguments=["--ros-args", "--log-level", "info"],
    ))

    return ld
