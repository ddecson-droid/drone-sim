#!/usr/bin/env python3
"""
PX4 TF Broadcaster -- 将 PX4 里程计 (NED坐标系) 转换为 ROS TF (ENU坐标系)。

============================================================
为什么需要这个节点？
============================================================
PX4 使用 FRD (Front-Right-Down) = NED (North-East-Down) 坐标系
ROS/ RViz2 使用 FLU (Front-Left-Up) = ENU (East-North-Up) 坐标系

没有这个节点做坐标转换, RViz2 中的无人机模型会:
  - 位置偏移 (x/y 轴互换)
  - 姿态翻转 (上下颠倒)

============================================================
坐标转换公式
============================================================
位置: x_enu = y_ned    (北→东)
      y_enu = x_ned    (东→北)
      z_enu = -z_ned   (下→上, 取反)

姿态: FRD→FLU = 绕X轴旋转180°
      四元数乘法: q_enu = q_ned ⊗ (0, 1, 0, 0)

============================================================
话题
============================================================
订阅: /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry)
发布: /tf, /tf_static (tf2_msgs/TFMessage)

TF树: odom ─── base_link
"""
import math  # sqrt 用于四元数归一化
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster  # 广播 /tf 话题
from px4_msgs.msg import VehicleOdometry  # PX4 里程计消息


class PX4TFBroadcaster(Node):
    """
    将 VehicleOdometry (NED) 转换为 TF (ENU) 并广播。

    工作流程:
    1. 订阅 /fmu/out/vehicle_odometry
    2. 收到消息时提取 NED 位置和四元数
    3. 转换为 ENU 坐标
    4. 广播 odom -> base_link 的 TF 变换
    """

    def __init__(self):
        super().__init__("px4_tf_broadcaster")  # 节点名, 可通过 ros2 node list 查看

        # tf2 广播器: 将坐标变换发布到 /tf 话题
        # RViz2 通过监听 /tf 来定位模型在 3D 空间中的位置
        self.tf_broadcaster = TransformBroadcaster(self)

        # ============================================================
        # QoS 配置
        # ============================================================
        # PX4 的 uXRCE-DDS 发布数据使用 BEST_EFFORT 策略
        # 必须匹配, 否则 ROS 2 侧收不到数据
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 最大努力投递 (不保证到达)
            durability=DurabilityPolicy.VOLATILE,        # 新订阅者只收新消息
            depth=10,                                     # 队列深度
        )

        # ============================================================
        # 创建订阅者
        # ============================================================
        # 话题名是 /fmu/out/vehicle_odometry (PX4 固定输出)
        # 消息类型是 VehicleOdometry (定义在 px4_msgs 包中)
        # odometry_callback 在每次收到消息时被调用
        self.odom_sub = self.create_subscription(
            VehicleOdometry,                   # 消息类型
            "/fmu/out/vehicle_odometry",       # 话题名
            self.odometry_callback,            # 回调函数
            qos,                               # QoS 配置
        )

        # 打印启动信息
        self.get_logger().info("PX4 TF Broadcaster started. Waiting for /fmu/out/vehicle_odometry...")

    def _ned_to_enu_quaternion(self, q_ned):
        """
        将 NED (FRD) 四元数转换为 ENU (FLU) 四元数。

        原理:
        - PX4 四元数 [w, x, y, z] 表示 NED世界 → FRD机体的旋转
        - ROS 需要四元数表示 ENU世界 → FLU机体的旋转
        - FRD → FLU = 绕X轴旋转180° (pi弧度)
        - 通过在右侧乘以旋转四元数 (0, 1, 0, 0) 实现

        哈密尔顿乘积 (q ⊗ q_flip):
        q = (qw, qx, qy, qz)
        q_flip = (0, 1, 0, 0)  # cos(pi/2)=0, sin(pi/2)=1

        (w1 + x1i + y1j + z1k) * (w2 + x2i + y2j + z2k)
        = (w1w2 - x1x2 - y1y2 - z1z2)
        + (w1x2 + x1w2 + y1z2 - z1y2)i
        + (w1y2 - x1z2 + y1w2 + z1x2)j
        + (w1z2 + x1y2 - y1x2 + z1w2)k

        代入 q_flip = (0, 1, 0, 0):
        w = -qx   从 0 - qx*1 - 0 - 0 = -qx
        x = qw    从 0 + qw*1 + 0 - 0 = qw
        y = -qz   从 0 - 0 + 0 + (-qz)*0 = -qz  (修正: 实际是 -qz)
        z = qy    从 0 + qx*0 - qy*1 + qz*0 = qy (修正: 实际是 qy)
        """
        qw, qx, qy, qz = q_ned  # 从 PX4 消息中提取四元数分量

        # 哈密尔顿乘积: q ⊗ (0, 1, 0, 0)
        # 手动展开四元数乘法公式
        w = -qx   # w1*x2 项, 其余为0
        x = qw    # x1*w2 项, 其余为0
        y = -qz   # -z1*y2 项 (交叉项)
        z = qy    # y1*x2 项

        # 归一化: 确保四元数模长为1, 防止累积误差
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm > 1e-9:  # 避免除零
            w, x, y, z = w/norm, x/norm, y/norm, z/norm

        # 返回 ROS geometry_msgs/Quaternion 格式 (x, y, z, w)
        return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))

    def odometry_callback(self, msg: VehicleOdometry):
        """
        处理 PX4 里程计消息。

        每次收到 /fmu/out/vehicle_odometry 时调用。
        将从 PX4 收到的 NED 数据转换为 ENU 并发布 TF。
        """
        # ============================================================
        # 步骤1: 提取 NED 位置
        # ============================================================
        # msg.position 是 [x_north, y_east, z_down] 三元素列表
        x_ned = msg.position[0]  # 北向 (North)
        y_ned = msg.position[1]  # 东向 (East)
        z_ned = msg.position[2]  # 下向 (Down), 正值=高度降低

        # ============================================================
        # 步骤2: NED → ENU 坐标转换
        # ============================================================
        # NED (北-东-下) → ENU (东-北-上)
        x_enu = y_ned   # NED东向 → ENU的X轴 (东)
        y_enu = x_ned   # NED北向 → ENU的Y轴 (北)
        z_enu = -z_ned  # NED下向取反 → ENU的Z轴 (上)

        # ============================================================
        # 步骤3: 四元数 NED → ENU
        # ============================================================
        q_ned = msg.q  # PX4 四元数 [w, x, y, z], NED→FRD旋转
        q_enu = self._ned_to_enu_quaternion(q_ned)  # 转换后 ENU→FLU旋转

        # ============================================================
        # 步骤4: 构建 TF 消息
        # ============================================================
        t = TransformStamped()  # 带时间戳的坐标变换

        # 头部信息: 时间戳 + 参考帧
        t.header.stamp = self.get_clock().now().to_msg()  # 当前 ROS 时间
        t.header.frame_id = "odom"       # 父坐标系: 里程计原点
        t.child_frame_id = "base_link"   # 子坐标系: 无人机机体中心

        # 平移: ENU 位置
        t.transform.translation.x = x_enu
        t.transform.translation.y = y_enu
        t.transform.translation.z = z_enu

        # 旋转: ENU 四元数
        t.transform.rotation = q_enu

        # ============================================================
        # 步骤5: 广播 TF
        # ============================================================
        # sendTransform 将变换发布到 /tf 话题
        # RViz2 监听 /tf 来更新 RobotModel 的位置
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """
    主函数: ROS 2 节点入口。

    标准 ROS 2 启动流程:
    1. rclpy.init() - 初始化客户端库
    2. 创建节点实例
    3. rclpy.spin() - 进入事件循环 (处理回调)
    4. 退出时 destroy_node() + shutdown()
    """
    rclpy.init(args=args)  # 初始化 ROS 2 Python 客户端
    node = PX4TFBroadcaster()  # 创建节点
    try:
        rclpy.spin(node)  # 阻塞等待回调 (直到 Ctrl+C)
    except KeyboardInterrupt:
        pass  # Ctrl+C 正常退出, 不打印异常
    finally:
        node.destroy_node()  # 清理资源
        rclpy.shutdown()     # 关闭 ROS 2


# Python 入口点
if __name__ == "__main__":
    main()
