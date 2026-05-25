"""
rviz.launch.py -- 启动 RViz2 并加载无人机可视化配置。

============================================================
RViz2 显示什么？
============================================================
1. RobotModel   -- 从 /robot_description 参数加载 X500 URDF 3D 模型
2. TF           -- 显示所有 TF 坐标系 (odom, base_link, arm_FL 等)
3. Grid         -- odom 平面的参考网格
4. Camera 图像  -- /camera/image_raw (需要相机模型)
5. LaserScan    -- /scan (需要激光雷达模型)

============================================================
配置文件
============================================================
config/rviz_config.rviz 预配置了所有显示面板:
  - Fixed Frame: odom (以里程计原点为参考)
  - RobotModel: 从 Topic 加载 (Depth: 5 = Transient Local)
  - TF: 显示坐标轴 + 名称
  - Grid: odom 平面, 20×20 格

============================================================
-d 参数
============================================================
RViz2 的 -d 参数指定 display config 文件:
  rviz2 -d ~/path/to/config.rviz

如果不指定 -d, RViz2 使用上次的配置 (或空白配置)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration  # 读取参数值
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================================
    # 查找 RViz 配置文件路径
    # ============================================================
    pkg_dir = get_package_share_directory("drone_bringup")
    config_path = os.path.join(pkg_dir, "config", "rviz_config.rviz")

    ld = LaunchDescription()

    # ============================================================
    # 声明参数
    # ============================================================
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock",
    ))
    ld.add_action(DeclareLaunchArgument(
        "rviz_config", default_value=config_path,
        description="Path to RViz configuration file",
        # 可在命令行覆盖: rviz_config:=/path/to/custom.rviz
    ))

    # ============================================================
    # 启动 RViz2 节点
    # ============================================================
    ld.add_action(Node(
        package="rviz2",
        executable="rviz2",  # RViz2 图形化主程序
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        # -d: 加载指定的 display config 文件
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    ))

    return ld
