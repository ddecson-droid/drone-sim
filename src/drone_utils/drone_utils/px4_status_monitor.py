#!/usr/bin/env python3
"""
PX4 Vehicle Status Monitor -- 在终端打印无人机状态。

============================================================
为什么需要这个节点？
============================================================
PX4 内部状态变化没有可视化界面, 这个节点在终端实时打印:
  - 解锁状态 (INIT → STANDBY → ARMED)
  - 飞行模式 (MANUAL / OFFBOARD / AUTO_MISSION 等)
  - 预检状态 (通过/失败)

在仿真调试时非常有用: 一眼就能看到为什么不能解锁,
或者当前处于什么飞行模式。

============================================================
话题
============================================================
订阅: /fmu/out/vehicle_status (px4_msgs/VehicleStatus)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus  # PX4 飞行状态消息


# ============================================================
# PX4 nav_state 编号 → 飞行模式名称映射
# 来自 PX4 源码: src/modules/commander/state_machine_helper.h
# ============================================================
NAV_STATES = {
    0: "MANUAL",              # 手动模式
    1: "ALTCTL",              # 高度保持
    2: "POSCTL",              # 位置保持
    3: "AUTO_MISSION",        # 自动任务
    4: "AUTO_LOITER",         # 自动盘旋
    5: "AUTO_RTL",            # 自动返航
    6: "AUTO_LAND",           # 自动降落
    7: "AUTO_RTGS",           # 自动返回地面站
    8: "AUTO_READY",          # 任务就绪
    9: "AUTO_TAKEOFF",        # 自动起飞
    10: "AUTO_FOLLOW_TARGET", # 目标跟随
    11: "AUTO_VTOL_TAKEOFF",  # VTOL 起飞
    12: "AUTO_PRECLAND",      # 精准降落
    14: "OFFBOARD",            # ★ 外部控制模式 (我们用的)
    17: "AUTO_LANDING",       # 降落中
    18: "AUTO_GOTO",          # 飞到指定位置
}

# ============================================================
# PX4 arming_state 编号 → 解锁状态名称映射
# ============================================================
ARMING_STATES = {
    1: "INIT",           # 初始化阶段
    2: "STANDBY",        # 待命 (可以解锁)
    3: "ARMED",          # 已解锁 (电机可以旋转)
    4: "STANDBY_ERROR",  # 待命错误 (检查失败)
}


class PX4StatusMonitor(Node):
    """
    监听 /fmu/out/vehicle_status, 状态变化时打印到终端。

    只在状态变化时打印 (减少日志刷屏):
    - 如果飞行模式从 MANUAL 变为 OFFBOARD, 打印一行
    - 如果解锁状态从 STANDBY 变为 ARMED, 打印一行
    """

    def __init__(self):
        super().__init__("px4_status_monitor")  # 节点名

        # QoS 配置: 必须匹配 PX4 发布侧
        # PX4 uXRCE-DDS 用 BEST_EFFORT (不重传丢失的消息)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 尽力投递
            durability=DurabilityPolicy.VOLATILE,        # 不保留历史消息
            depth=10,                                     # 最多缓存10条
        )

        # 创建订阅者: 监听飞行状态
        self.status_sub = self.create_subscription(
            VehicleStatus,                 # 消息类型
            "/fmu/out/vehicle_status",     # 话题名 (PX4 固定)
            self.status_callback,          # 回调函数
            qos,                           # QoS
        )

        # 记录上一次的状态值, 用于检测状态变化
        self.last_nav_state = -1     # 上次飞行模式编号
        self.last_arming_state = -1  # 上次解锁状态编号

        self.get_logger().info(
            "PX4 Status Monitor started. Waiting for /fmu/out/vehicle_status..."
        )

    def status_callback(self, msg: VehicleStatus):
        """
        处理状态消息。

        只在状态改变时才打印, 避免日志被重复消息刷屏。
        """
        nav_state = msg.nav_state          # 当前飞行模式编号
        arming_state = msg.arming_state    # 当前解锁状态编号

        # 检测状态是否变化 (任一变化都打印)
        if nav_state != self.last_nav_state or arming_state != self.last_arming_state:
            # 编号 → 可读名称 (查不到就用 UNKNOWN)
            nav_name = NAV_STATES.get(nav_state, f"UNKNOWN({nav_state})")
            arm_name = ARMING_STATES.get(arming_state, f"UNKNOWN({arming_state})")

            # 格式化输出到终端
            self.get_logger().info(
                f"PX4 Status -- Arming: {arm_name} | "
                f"Mode: {nav_name} | "
                f"Preflight: {'OK' if msg.pre_flight_checks_pass else 'FAIL'}"
                # pre_flight_checks_pass: True=通过, False=有失败项
            )

            # 更新记录, 下次相同状态不再打印
            self.last_nav_state = nav_state
            self.last_arming_state = arming_state


def main(args=None):
    """
    主函数: ROS 2 节点入口。

    流程: init → 创建节点 → spin (事件循环) → destroy → shutdown
    """
    rclpy.init(args=args)  # 初始化客户端库
    node = PX4StatusMonitor()  # 创建节点实例
    try:
        rclpy.spin(node)  # 进入事件循环, 等待回调
    except KeyboardInterrupt:
        pass  # 用户按 Ctrl+C
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
