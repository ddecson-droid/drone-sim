#!/usr/bin/env python3
"""
Keyboard Teleop -- 键盘遥控无人机 (速度控制模式)。

============================================================
操作方式
============================================================
  w/s     前进/后退   (NED: vx ±1.0 m/s)
  a/d     左移/右移   (NED: vy ±1.0 m/s)
  r/f     上升/下降   (NED: vz ∓1.5 m/s, 负值=上升)
  q/e     偏航左/右   (yaw_rate ±0.5 rad/s)
  空格    悬停        (零速度)
  t       解锁 + 切换 OFFBOARD 模式
  l       降落
  Esc     退出

============================================================
与 offboard_control.py 的区别
============================================================
offboard_control 是"位置控制" → 发位置设定点
keyboard_teleop 是"速度控制"  → 发速度设定点

速度控制更适合遥控: 按住 w 就持续前进, 松手按空格就悬停

============================================================
非阻塞键盘读取原理
============================================================
select.select() 检查 stdin 是否有数据可读 (超时=0, 即时返回):
- 有按键 → 读取1字节并处理
- 无按键 → 立即返回, 不阻塞

tty.setraw() 将终端设为 raw 模式: 按键无需回车, 即时响应
termios.tcsetattr() 退出时恢复终端设置

============================================================
话题
============================================================
发布: /fmu/in/offboard_control_mode  (速度模式心跳)
      /fmu/in/trajectory_setpoint     (速度设定点)
      /fmu/in/vehicle_command         (解锁/模式切换)

订阅: /fmu/out/vehicle_status         (反馈当前状态)
"""
import sys           # stdin 文件描述符
import select        # 非阻塞 I/O 多路复用
import termios       # 终端属性操作
import tty           # 终端模式设置
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,  # 控制模式心跳
    TrajectorySetpoint,   # 轨迹设定点
    VehicleCommand,       # 车辆命令
    VehicleStatus,        # 状态反馈
)


# ============================================================
# PX4 命令常量
# ============================================================
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400  # 解锁/上锁
VEHICLE_CMD_DO_SET_MODE = 176           # 切换模式

# ============================================================
# 按键 → 速度 映射表 (NED 坐标系)
# ============================================================
# 格式: (vx_north, vy_east, vz_down, yaw_rate)
# vx: 北向速度 (m/s)
# vy: 东向速度 (m/s)
# vz: 下向速度 (m/s, 负值=上升)
# yaw_rate: 偏航角速度 (rad/s)
# ============================================================
KEY_MAP = {
    "w": (1.0, 0.0, 0.0, 0.0),      # 前进: 向北加速
    "s": (-1.0, 0.0, 0.0, 0.0),     # 后退: 向南加速
    "a": (0.0, 1.0, 0.0, 0.0),      # 左移: 向东加速
    "d": (0.0, -1.0, 0.0, 0.0),     # 右移: 向西加速
    "r": (0.0, 0.0, -1.5, 0.0),     # 上升: 向上加速 (NED z负=上升)
    "f": (0.0, 0.0, 1.5, 0.0),      # 下降: 向下加速
    "q": (0.0, 0.0, 0.0, 0.5),      # 偏航左: 逆时针旋转
    "e": (0.0, 0.0, 0.0, -0.5),     # 偏航右: 顺时针旋转
}


class KeyboardTeleop(Node):
    """
    键盘遥控节点: 读取键盘输入, 转换为速度指令发给 PX4。
    """

    def __init__(self):
        super().__init__("keyboard_teleop")  # 节点名

        # ============================================================
        # QoS 配置 (与 PX4 匹配)
        # ============================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ============================================================
        # 订阅: 读取 PX4 状态
        # ============================================================
        self.status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.status_callback, qos
        )

        # ============================================================
        # 发布: 向 PX4 发送命令
        # ============================================================
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        # 控制模式心跳: velocity=True (速度控制, 非位置控制)

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        # 速度设定点: 发布当前期望的速度值

        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )
        # 车辆命令: 解锁/切换模式

        # ============================================================
        # 状态变量
        # ============================================================
        self.nav_state = 0          # 飞行模式
        self.arming_state = 1       # 解锁状态
        self.vx = 0.0               # 当前北向速度指令
        self.vy = 0.0               # 当前东向速度指令
        self.vz = 0.0               # 当前下向速度指令
        self.yaw_rate = 0.0         # 当前偏航角速度指令
        self.armed = False          # 是否已解锁

        # ============================================================
        # 定时器
        # ============================================================
        self.offboard_timer = self.create_timer(0.1, self.offboard_heartbeat)
        # 控制模式心跳: 10Hz (必须 ≥ 2Hz)

        self.control_timer = self.create_timer(0.05, self.control_loop)
        # 控制循环: 20Hz (发布当前速度指令)

        self.input_timer = self.create_timer(0.1, self.read_keyboard)
        # 键盘读取: 10Hz (每100ms检查一次按键)

        # 打印操作说明
        self.get_logger().info(
            "Keyboard Teleop started.\n"
            "  w/s: forward/back  a/d: left/right  r/f: up/down\n"
            "  q/e: yaw L/R       space: hover\n"
            "  t: arm+takeoff     l: land          esc: exit"
        )

        # ============================================================
        # 保存终端设置 (退出时恢复)
        # ============================================================
        self.old_settings = termios.tcgetattr(sys.stdin)

    def status_callback(self, msg: VehicleStatus):
        """
        接收 PX4 状态更新。
        """
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def offboard_heartbeat(self):
        """
        发送速度控制模式心跳 (10Hz)。

        velocity=True: 告诉 PX4 "我使用速度控制"
        position=False: 位置设定点无效

        必须在切换到 OFFBOARD 模式之前就开始发送心跳,
        否则 PX4 会拒绝模式切换。
        """
        mode_msg = OffboardControlMode()
        mode_msg.position = False     # 不使用位置控制
        mode_msg.velocity = True      # ★ 使用速度控制
        mode_msg.acceleration = False # 不使用加速度控制
        mode_msg.attitude = False     # 不使用姿态控制
        mode_msg.body_rate = False    # 不使用角速率控制
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(mode_msg)

    def arm(self):
        """发送解锁命令 (param1=1.0)"""
        self.get_logger().info("Arming...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM  # 400
        cmd.param1 = 1.0  # 解锁
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def disarm(self):
        """发送上锁命令 (param1=0.0)"""
        self.get_logger().info("Disarming...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM  # 400
        cmd.param1 = 0.0  # 上锁
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def set_offboard_mode(self):
        """
        切换到 OFFBOARD 飞行模式 (param2=6)

        注意: 切换前必须已经持续发送 OffboardControlMode 和 TrajectorySetpoint
        至少 1 秒, 否则 PX4 会拒绝切换。
        """
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_DO_SET_MODE  # 176
        cmd.param1 = 1.0  # 主模式: custom
        cmd.param2 = 6.0  # 子模式: offboard
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        """
        控制循环 (20Hz): 发布当前速度指令。

        position 设为 NaN: 表示"我不控制位置, 我控制的是速度"
        velocity 设为当前 vx,vy,vz: 目标速度
        yawspeed 设为当前 yaw_rate: 目标偏航角速度

        为什么用 NaN?
        PX4 内部根据 NaN 来判断哪些字段有效:
        - 有效字段: 正常数值
        - 无效字段: NaN (PX4 会忽略)
        """
        sp = TrajectorySetpoint()
        sp.position = [float("nan"), float("nan"), float("nan")]
        # 位置 = NaN → PX4 知道不是位置控制模式
        sp.velocity = [float(self.vx), float(self.vy), float(self.vz)]
        # 速度目标 (NED)
        sp.yawspeed = float(self.yaw_rate)
        # 偏航角速度目标
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(sp)

    def read_keyboard(self):
        """
        非阻塞读取键盘输入。

        原理:
        1. tty.setraw() 将终端设为 raw 模式 (按键即时响应, 无需回车)
        2. select.select() 非阻塞检查 stdin 是否有数据
        3. 读取单个字符
        4. 根据字符设置速度指令
        5. finally 块恢复终端设置 (防止异常导致终端状态异常)
        """
        try:
            tty.setraw(sys.stdin.fileno())  # 切换到 raw 模式
            if select.select([sys.stdin], [], [], 0)[0]:
                # select 返回非空 → 有按键等待读取
                key = sys.stdin.read(1)  # 读取1个字符

                if key == "\x1b":  # ESC 键 (ASCII 27)
                    self.get_logger().info("Exiting...")
                    self.cleanup()
                    rclpy.shutdown()
                    sys.exit(0)

                elif key == " ":  # 空格键 → 悬停
                    self.vx = self.vy = self.vz = self.yaw_rate = 0.0
                    self.get_logger().info("Hover (zero velocity)")

                elif key == "t":  # t 键 → 解锁 + 切换 Offboard
                    self.arm()
                    self.set_offboard_mode()

                elif key == "l":  # l 键 → 降落
                    self.vx = self.vy = self.yaw_rate = 0.0
                    self.vz = 1.0  # 正向下降 (NED: z正=向下)
                    self.get_logger().info("Landing...")

                elif key in KEY_MAP:
                    # 从映射表读取速度值
                    vx, vy, vz, yr = KEY_MAP[key]
                    self.vx = vx
                    self.vy = vy
                    self.vz = vz
                    self.yaw_rate = yr
                    self.get_logger().info(
                        f"Velocity NED: vx={self.vx:.1f} vy={self.vy:.1f} "
                        f"vz={self.vz:.1f} yaw_rate={self.yaw_rate:.1f}"
                    )
        except Exception:
            pass  # 忽略任何读取异常
        finally:
            # ★ 关键: 恢复终端设置
            # 无论前面是否出错, 都必须恢复否则终端会处于异常状态
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def cleanup(self):
        """
        恢复终端设置。

        必须在退出前调用, 否则终端处于 raw 模式:
        - 所有按键即时响应, 无法正常使用 bash
        - 需要通过 reset 命令恢复
        """
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except Exception:
            pass

    def destroy_node(self):
        """
        销毁节点前恢复终端设置。

        覆盖基类方法: 确保 Ctrl+C 退出时也能恢复终端。
        """
        self.cleanup()
        super().destroy_node()


def main(args=None):
    """
    主函数: ROS 2 节点标准入口。
    """
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()      # 恢复终端
        node.destroy_node()  # 销毁节点
        rclpy.shutdown()     # 关闭 ROS 2


if __name__ == "__main__":
    main()
