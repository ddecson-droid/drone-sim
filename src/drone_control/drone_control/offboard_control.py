#!/usr/bin/env python3
"""
PX4 Offboard Control -- 自动起飞-悬停-降落状态机。

============================================================
状态机流程
============================================================
IDLE → ARMING → TAKEOFF → HOVER → LANDING → DISARMING → DONE

每个状态说明:
  IDLE:      等待开始 (auto_arm 为 True 则自动跳转)
  ARMING:    发送解锁命令, 等待 arming_state 变为 ARMED
  TAKEOFF:   切换到 OFFBOARD 模式, 逐步爬升到目标高度
  HOVER:     在目标高度悬停指定时长
  LANDING:   切换到 LAND 模式, 等待降落
  DISARMING: 发送上锁命令, 等待 disarmed
  DONE:      完成, 不再操作

============================================================
PX4 Offboard 控制的 3 个必要条件
============================================================
1. 持续发送 OffboardControlMode 心跳 (≥ 2Hz)
2. 持续发送 TrajectorySetpoint (当前位置或目标位置)
3. 发送 VehicleCommand 切换到 OFFBOARD 模式

缺失任何一个都会让 PX4 退出 Offboard 模式 (触发 failsafe)

============================================================
坐标系统
============================================================
所有位置使用 NED 坐标系:
  x = 北 (正 = 前进)
  y = 东 (正 = 左移)
  z = 下 (负 = 上升)

所以 takeoff_height = -5.0 表示上升到 5 米高度

============================================================
话题
============================================================
发布: /fmu/in/offboard_control_mode  (OffboardControlMode 心跳)
      /fmu/in/trajectory_setpoint     (位置设定点)
      /fmu/in/vehicle_command         (解锁/模式切换命令)

订阅: /fmu/out/vehicle_status         (反馈当前状态)
      /fmu/out/vehicle_odometry       (反馈当前位置)
"""
import rclpy
from enum import IntEnum  # 用于定义状态枚举
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,   # 外部控制模式心跳消息
    TrajectorySetpoint,    # 轨迹设定点 (位置/速度/加速度)
    VehicleCommand,        # 车辆命令 (解锁/切换模式)
    VehicleStatus,         # 车辆状态反馈
    VehicleOdometry,       # 里程计反馈 (位置+姿态)
)


# ============================================================
# 状态机枚举
# ============================================================
class State(IntEnum):
    IDLE = 0       # 空闲: 等待开始
    ARMING = 1     # 解锁中: 发送 ARM 命令, 等待确认
    TAKEOFF = 2    # 起飞中: 切换到 OFFBOARD 模式, 爬升
    HOVER = 3      # 悬停中: 保持高度等待
    LANDING = 4    # 降落中: 切换到 LAND 模式
    DISARMING = 5  # 上锁中: 发送 DISARM 命令
    DONE = 6       # 完成: 不再发送任何命令


# ============================================================
# PX4 VehicleCommand 命令常量
# 定义在 PX4 源码: msg/vehicle_command.msg
# ============================================================
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400  # 解锁/上锁命令
                                        # param1=1.0 → 解锁
                                        # param1=0.0 → 上锁

VEHICLE_CMD_DO_SET_MODE = 176           # 切换飞行模式命令
                                        # param1=1.0, param2=6.0 → OFFBOARD 模式
                                        # param3=5.0 → LAND 模式


class OffboardControl(Node):
    """
    外部控制节点: 自动执行起飞→悬停→降落序列。

    通过 2 个定时器驱动:
    - offboard_heartbeat: 10Hz, 发送 OffboardControlMode 心跳
    - control_loop: 20Hz, 运行状态机逻辑
    """

    def __init__(self):
        super().__init__("offboard_control")  # 节点名

        # ============================================================
        # 可调参数 (通过 launch 文件或命令行覆盖)
        # ============================================================
        self.declare_parameter("takeoff_height", -5.0)  # 目标高度 (NED, 负=上升)
        self.declare_parameter("hover_time", 10.0)       # 悬停时间 (秒)
        self.declare_parameter("auto_arm", True)         # 是否自动解锁

        self.takeoff_height = self.get_parameter("takeoff_height").value
        self.hover_time = self.get_parameter("hover_time").value
        self.auto_arm = self.get_parameter("auto_arm").value

        # ============================================================
        # QoS 配置
        # ============================================================
        # 关键: 必须用 BEST_EFFORT 匹配 PX4 uXRCE-DDS 发布侧
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 尽力投递
            durability=DurabilityPolicy.VOLATILE,        # 不保留历史
            depth=10,                                     # 队列深度
        )

        # ============================================================
        # 订阅: 读取 PX4 反馈
        # ============================================================
        self.status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.status_callback, qos
        )
        # 状态回调: 更新 nav_state (飞行模式) 和 arming_state (解锁状态)

        self.odom_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.odom_callback, qos
        )
        # 里程计回调: 更新 current_z (当前高度)

        # ============================================================
        # 发布: 向 PX4 发送命令
        # ============================================================
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        # Offboard 控制模式心跳: 告诉 PX4 "我还在控制, 别切回手动"

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        # 轨迹设定点: 告诉 PX4 "飞到这个位置"

        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )
        # 车辆命令: 解锁/上锁/切换模式

        # ============================================================
        # 状态变量
        # ============================================================
        self.state = State.IDLE                     # 当前状态机状态
        self.nav_state = 0                           # 当前飞行模式编号
        self.arming_state = 1                        # 当前解锁状态 (1=INIT)
        self.current_z = 0.0                         # 当前高度 (NED)
        self.takeoff_setpoint_sent = False           # 是否已发送过 OFFBOARD 模式切换

        # ============================================================
        # 定时器
        # ============================================================
        self.control_timer = self.create_timer(0.05, self.control_loop)
        # 控制循环: 20Hz (每 50ms 运行一次状态机)

        self.offboard_timer = self.create_timer(0.1, self.offboard_heartbeat)
        # 心跳: 10Hz (每 100ms 发送一次 OffboardControlMode)
        # PX4 要求 ≥2Hz, 我们用 10Hz 保证余量

        # 时间记录 (用于计算持续时间)
        self.start_time = None        # 起飞开始时间
        self.hover_start_time = None  # 悬停开始时间

        # 打印启动信息
        self.get_logger().info(
            f"Offboard Control started. "
            f"Takeoff height: {-self.takeoff_height:.1f}m, "   # 显示为正值
            f"Hover time: {self.hover_time:.1f}s, "
            f"Auto-arm: {self.auto_arm}"
        )

    def status_callback(self, msg: VehicleStatus):
        """
        收到 /fmu/out/vehicle_status 时更新状态。

        PX4 以 ~10Hz 发布此话题, 包含:
        - nav_state: 飞行模式 (0=MANUAL, 14=OFFBOARD)
        - arming_state: 解锁状态 (2=STANDBY, 3=ARMED)
        - pre_flight_checks_pass: 预检是否通过
        """
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def odom_callback(self, msg: VehicleOdometry):
        """
        收到 /fmu/out/vehicle_odometry 时更新高度。

        msg.position[2] 是 NED 的 z 分量: 负值=上升, 正值=下降
        """
        self.current_z = msg.position[2]

    def offboard_heartbeat(self):
        """
        发送 Offboard 控制模式心跳 (10Hz)。

        为什么需要心跳?
        PX4 在 Offboard 模式下, 如果超过 500ms 没收到心跳消息,
        会自动触发 failsafe (切换到 Hold 模式或降落)。
        所以必须持续以 ≥2Hz 的频率发送。

        我们用位置控制模式: position=True, 其余=False
        """
        if self.state < State.DONE:  # 完成后不再发送
            mode_msg = OffboardControlMode()
            mode_msg.position = True      # ★ 位置控制
            mode_msg.velocity = False     # 不使用速度控制
            mode_msg.acceleration = False # 不使用加速度控制
            mode_msg.attitude = False     # 不使用姿态控制
            mode_msg.body_rate = False    # 不使用角速率控制
            mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            # timestamp 单位是微秒 (PX4 要求)
            self.offboard_mode_pub.publish(mode_msg)

    def arm(self):
        """
        发送解锁命令。

        VehicleCommand:
        - command=400 (COMPONENT_ARM_DISARM)
        - param1=1.0 → 解锁
        - target_system=1 (PX4 SITL 默认系统ID)
        """
        self.get_logger().info("Sending ARM command...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM  # 400
        cmd.param1 = 1.0          # 1.0=解锁, 0.0=上锁
        cmd.target_system = 1     # PX4 系统 ID (SITL 默认为1)
        cmd.target_component = 1  # 组件 ID (autopilot=1)
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def disarm(self):
        """发送上锁命令 (param1=0.0)"""
        self.get_logger().info("Sending DISARM command...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM  # 400
        cmd.param1 = 0.0          # 0.0=上锁
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def set_offboard_mode(self):
        """
        切换到 Offboard 飞行模式。

        VehicleCommand:
        - command=176 (DO_SET_MODE)
        - param1=1.0 (main mode: custom)
        - param2=6.0 (sub-mode: offboard)

        切换前必须已经:
        1. 持续发送 OffboardControlMode 心跳
        2. 持续发送 TrajectorySetpoint
        """
        self.get_logger().info("Switching to OFFBOARD mode...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_DO_SET_MODE  # 176
        cmd.param1 = 1.0  # 主模式: custom (自定义)
        cmd.param2 = 6.0  # 子模式: offboard (外部控制)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def set_land_mode(self):
        """
        切换到 Land 模式, 让 PX4 自动降落。
        param3=5.0 或使用 NAV_LAND 命令均可触发降落。
        """
        self.get_logger().info("Switching to LAND mode...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0
        cmd.param3 = 5.0  # 子模式: LAND (降落)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def publish_position_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """
        发布位置设定点 (NED 坐标系)。

        参数:
        - x: 北向位移 (米)
        - y: 东向位移 (米)
        - z: 下向位移 (米, 负值=上升)
        - yaw: 偏航角 (弧度)

        设定点字段说明:
        - position[3]: x, y, z (NED)
        - velocity[3]: NaN (不使用速度控制)
        - yaw: 目标偏航角 (NaN=不控制)
        - yawspeed: 目标偏航角速度 (NaN=不控制)
        """
        sp = TrajectorySetpoint()
        sp.position = [float(x), float(y), float(z)]  # 位置设定 (NED)
        sp.yaw = float(yaw)                            # 偏航角
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(sp)

    def control_loop(self):
        """
        状态机主循环 (20Hz)。

        根据当前状态执行对应逻辑, 并在条件满足时切换到下一个状态。

        关键时序:
        - TAKEOFF 阶段: 先发设定点 → 等1秒 → 切 OFFBOARD → 爬升 → 到达后切 HOVER
        - HOVER 阶段: 保持高度 → 计时达标 → 切 LANDING
        - ARMING/DISARMING: 等待 PX4 反馈确认状态变化
        """
        now = self.get_clock().now().nanoseconds / 1e9  # 当前时间 (秒)

        # ---- IDLE: 等待开始 ----
        if self.state == State.IDLE:
            if self.auto_arm:
                self.state = State.ARMING  # 直接跳转到解锁

        # ---- ARMING: 解锁无人机 ----
        elif self.state == State.ARMING:
            if self.arming_state == 2:  # STANDBY: 待命状态, 可以解锁
                self.arm()               # 发送解锁命令
                self.get_logger().info("Waiting for arm...")
            elif self.arming_state == 3:  # ARMED: 解锁成功
                self.get_logger().info("Vehicle armed. Taking off...")
                self.state = State.TAKEOFF
                self.start_time = now     # 记录起飞开始时间

        # ---- TAKEOFF: 起飞并爬升到目标高度 ----
        elif self.state == State.TAKEOFF:
            if not self.takeoff_setpoint_sent:
                # 阶段A: 等待1秒后切换到 OFFBOARD 模式
                # 同时发送当前位置附近设定点 (满足 PX4 的 setpoint 预检查)
                if now - (self.start_time or now) > 1.0:
                    self.set_offboard_mode()
                    self.takeoff_setpoint_sent = True
                # 发布略高于当前高度的设定点
                self.publish_position_setpoint(z=self.current_z + 0.5)
            else:
                # 阶段B: 渐进爬升到目标高度 (2秒内完成过渡)
                elapsed = now - self.start_time - 1.0  # 扣除等待的1秒
                # 线性插值: 从当前高度到目标高度
                t = min(elapsed / 2.0, 1.0)  # 插值参数 0→1
                target_z = self.current_z + (self.takeoff_height - self.current_z) * t
                self.publish_position_setpoint(z=target_z)

                # 判断是否到达: 高度误差 < 0.5米 且 经过 > 3秒
                if abs(self.current_z - self.takeoff_height) < 0.5 and elapsed > 3.0:
                    self.get_logger().info(
                        f"Reached takeoff height: {-self.current_z:.1f}m. Hovering..."
                    )
                    self.state = State.HOVER
                    self.hover_start_time = now

        # ---- HOVER: 在目标高度悬停 ----
        elif self.state == State.HOVER:
            self.publish_position_setpoint(z=self.takeoff_height)  # 保持高度
            # 检查悬停时间是否达标
            if self.hover_start_time and (now - self.hover_start_time) > self.hover_time:
                self.get_logger().info("Hover complete. Landing...")
                self.state = State.LANDING

        # ---- LANDING: 切换到 LAND 模式降落 ----
        elif self.state == State.LANDING:
            self.set_land_mode()  # 让 PX4 接管降落
            # 检测是否已接地 (NED: z > -0.5 表示接近地面)
            if self.current_z > -0.5:
                self.get_logger().info("On ground. Disarming...")
                self.state = State.DISARMING
            else:
                self.publish_position_setpoint(z=0.0)  # 保持对地面高度的设定

        # ---- DISARMING: 上锁 ----
        elif self.state == State.DISARMING:
            self.disarm()
            if self.arming_state == 2:  # STANDBY (已上锁)
                self.get_logger().info("Disarmed. Mission complete!")
                self.state = State.DONE

        # ---- DONE: 完成 ----
        elif self.state == State.DONE:
            pass  # 不再做任何操作


def main(args=None):
    """
    主函数: ROS 2 节点标准入口。
    """
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)  # 事件循环
    except KeyboardInterrupt:
        node.get_logger().info("Offboard control interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
