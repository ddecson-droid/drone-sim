"""
description.launch.py -- 加载 X500 URDF 并启动 robot_state_publisher。

============================================================
robot_state_publisher 做什么？
============================================================
1. 读取 /robot_description 参数中的 URDF 字符串
2. 解析 URDF 中的 <joint name="..." type="fixed"> 定义
3. 发布所有固定关节的 TF (如 base_link → arm_FL)

例如:
  <joint name="arm_fr_joint" type="fixed">
    <parent link="base_link"/>
    <child link="arm_fr"/>
    <origin xyz="0.06 -0.06 0.0" rpy="0 0 0"/>
  </joint>

robot_state_publisher 会发布:
  base_link → arm_fr 的静态 TF (x=0.06, y=-0.06, z=0.0)

============================================================
为什么需要这个?
============================================================
RViz2 的 RobotModel 显示需要两样东西:
1. /robot_description 参数 → URDF 字符串 (定义 3D 几何形状)
2. /tf 话题 → 所有关节的 TF (定义各部件之间的位置关系)

robot_state_publisher 负责第2点 (来自 URDF 的静态 TF)
px4_tf_broadcaster 负责动态 TF (odom → base_link, 来自 PX4 数据)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration  # 读取启动参数
from launch_ros.actions import Node  # 启动 ROS 2 节点


def generate_launch_description():
    # ============================================================
    # 读取 URDF 文件内容
    # ============================================================
    # get_package_share_directory: 返回包安装后的 share 路径
    pkg_dir = get_package_share_directory("drone_description")
    urdf_path = os.path.join(pkg_dir, "urdf", "x500.urdf")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()  # 读取整个 URDF XML 内容为字符串

    ld = LaunchDescription()

    # ============================================================
    # 声明参数
    # ============================================================
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    ))

    # ============================================================
    # 启动 robot_state_publisher 节点
    # ============================================================
    # robot_description 参数: 包含完整 URDF 的字符串
    # RViz2 的 RobotModel 插件会读取此参数来渲染 3D 模型
    ld.add_action(Node(
        package="robot_state_publisher",    # 标准 ROS 2 包
        executable="robot_state_publisher", # 可执行文件名
        name="robot_state_publisher",       # 运行时节点名
        output="screen",                    # 终端输出日志
        parameters=[{
            "robot_description": robot_desc,  # ★ URDF 字符串
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    ))

    return ld
