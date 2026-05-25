"""
simulation.launch.py -- 顶层启动文件, 编排所有 ROS 2 节点。

============================================================
启动顺序 (带延迟)
============================================================
t=0.0s  → robot_state_publisher    (URDF → /robot_description 参数)
t=0.5s  → px4_tf_broadcaster        (订阅 /fmu/out/vehicle_odometry, 发布 /tf)
          px4_status_monitor        (订阅 /fmu/out/vehicle_status, 终端打印)
t=5.0s  → ros_gz_bridge            (桥接 Gazebo ↔ ROS 2 话题)
t=8.0s  → RViz2                    (加载 rviz_config.rviz)

============================================================
为什么需要延迟?
============================================================
- ros_gz_bridge 延迟5秒: 等 Gazebo 完全启动
- RViz2 延迟8秒: 等 TF 和 /robot_description 就绪

============================================================
用法 (终端3 启动)
============================================================
  source /opt/ros/jazzy/setup.bash
  source ~/drone_ws/install/setup.bash
  ros2 launch drone_bringup simulation.launch.py
  ros2 launch drone_bringup simulation.launch.py use_rviz:=false
  ros2 launch drone_bringup simulation.launch.py use_camera:=true
"""
import os  # 用于拼接文件路径
from ament_index_python.packages import get_package_share_directory

# ============================================================
# launch 库 API 导入
# ============================================================
from launch import LaunchDescription  # 启动描述: 包含所有要执行的 action
from launch.actions import (
    DeclareLaunchArgument,       # 声明启动参数 (可在命令行用 := 覆盖)
    IncludeLaunchDescription,    # 包含其他 launch 文件
    TimerAction,                 # 延迟执行
)
from launch_ros.actions import Node  # 启动 ROS 2 节点 (替代 IncludeLaunchDescription 避免 libexec 问题)
from launch.conditions import IfCondition  # 条件判断
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 引用外部 .launch.py 文件
from launch.substitutions import (
    LaunchConfiguration,         # 读取启动参数的值
    PathJoinSubstitution,        # 拼接路径
)
from launch_ros.substitutions import FindPackageShare  # 查找包安装路径


def generate_launch_description():
    """
    生成启动描述。

    ROS 2 launch 系统调用此函数来获取要执行的操作列表。
    每个 add_action() 添加一个要在启动时执行的操作。
    """
    ld = LaunchDescription()  # 创建空的启动描述

    # ============================================================
    # 声明启动参数
    # 用户可在命令行用 := 覆盖: ros2 launch ... use_rviz:=false
    # ============================================================
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",                           # 默认启用仿真时间
        description="Use Gazebo simulation clock",      # 显示在 --help 中的说明
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2",
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_camera",
        default_value="false",                           # 默认不桥接相机
        description="Bridge camera topics (requires camera-equipped model)",
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_lidar",
        default_value="false",                           # 默认不桥接激光雷达
        description="Bridge lidar topics (requires lidar-equipped model)",
    ))
    ld.add_action(DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("drone_bringup"),  # 包安装路径
            "config", "rviz_config.rviz"                   # 配置文件相对路径
        ),
        description="Path to RViz2 config file",
    ))

    # 创建启动配置对象, 用于在运行时读取参数值
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    # ============================================================
    # 第1步: robot_state_publisher
    # ============================================================
    # 加载 x500.urdf 到 /robot_description 参数
    # RViz2 通过监听 /robot_description 来获取无人机3D模型
    # 同时发布 base_link → arm/motor/prop 的静态 TF
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("drone_description"),  # 查找 drone_description 包路径
                    "launch", "description.launch.py",       # 子启动文件
                ])
            ]),
            launch_arguments={"use_sim_time": use_sim_time}.items(),  # 传递参数
        )
    )

    # ============================================================
    # 第2步: PX4 工具节点
    # ============================================================
    # 使用 Node 直接启动 (而非 IncludeLaunchDescription)
    # 原因: 避免 ROS 2 Jazzy 的 libexec 目录检查问题
    ld.add_action(
        Node(
            package="drone_utils",                     # 包名
            executable="px4_tf_broadcaster",            # 可执行文件名 (来自 setup.py entry_points)
            name="px4_tf_broadcaster",                  # 运行时节点名
            output="screen",                             # 输出到终端
            parameters=[{"use_sim_time": use_sim_time}], # 参数
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

    # ============================================================
    # 第3步: ros_gz_bridge (延迟5秒)
    # ============================================================
    # 桥接 Gazebo Transport 话题到 ROS 2 话题
    # 延迟: 等待 Gazebo 完全启动并加载模型
    ld.add_action(
        TimerAction(
            period=5.0,  # 延迟秒数
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

    # ============================================================
    # 第4步: RViz2 (延迟8秒)
    # ============================================================
    # 视觉化工具: 显示无人机3D模型和TF树
    # 延迟: 等待 /robot_description 和 /tf 都就绪
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
                    condition=IfCondition(use_rviz),  # 如果 use_rviz=false 就跳过
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                    }.items(),
                ),
            ],
        )
    )

    # 返回构建好的启动描述
    return ld
