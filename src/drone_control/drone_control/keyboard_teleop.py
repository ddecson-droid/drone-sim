#!/usr/bin/env python3
"""
Keyboard teleoperation for drone velocity control in offboard mode.

Controls (press keys):
  w/s    : forward/backward  (NED: x +/-)
  a/d    : left/right        (NED: y +/-)
  q/e    : yaw left/right
  r/f    : up/down           (NED: z +/-)
  space  : hover (zero velocity)
  t      : arm and takeoff mode
  l      : land
  esc    : exit

Publishes:
  /fmu/in/offboard_control_mode  (OffboardControlMode)
  /fmu/in/trajectory_setpoint     (TrajectorySetpoint - velocity mode)
  /fmu/in/vehicle_command         (VehicleCommand)
"""
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)


# PX4 command constants
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_DO_SET_MODE = 176

# Key-to-velocity mapping (NED frame)
KEY_MAP = {
    "w": (1.0, 0.0, 0.0, 0.0),     # forward  (+x north)
    "s": (-1.0, 0.0, 0.0, 0.0),    # backward (-x south)
    "a": (0.0, 1.0, 0.0, 0.0),     # left     (+y east)
    "d": (0.0, -1.0, 0.0, 0.0),    # right    (-y west)
    "r": (0.0, 0.0, -1.5, 0.0),    # up       (-z down, NED: negative z = up)
    "f": (0.0, 0.0, 1.5, 0.0),     # down     (+z down)
    "q": (0.0, 0.0, 0.0, 0.5),     # yaw left
    "e": (0.0, 0.0, 0.0, -0.5),    # yaw right
}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.status_callback, qos
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        self.nav_state = 0
        self.arming_state = 1
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        self.armed = False

        # Timers
        self.offboard_timer = self.create_timer(0.1, self.offboard_heartbeat)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.input_timer = self.create_timer(0.1, self.read_keyboard)

        self.get_logger().info(
            "Keyboard Teleop started.\n"
            "  w/s: forward/back  a/d: left/right  r/f: up/down\n"
            "  q/e: yaw L/R       space: hover\n"
            "  t: arm+takeoff     l: land          esc: exit"
        )

        # Save terminal settings for keyboard input
        self.old_settings = termios.tcgetattr(sys.stdin)

    def status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def offboard_heartbeat(self):
        mode_msg = OffboardControlMode()
        mode_msg.position = False
        mode_msg.velocity = True
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = False
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(mode_msg)

    def arm(self):
        self.get_logger().info("Arming...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def disarm(self):
        self.get_logger().info("Disarming...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 0.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def set_offboard_mode(self):
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        sp = TrajectorySetpoint()
        sp.position = [float("nan"), float("nan"), float("nan")]
        sp.velocity = [float(self.vx), float(self.vy), float(self.vz)]
        sp.yawspeed = float(self.yaw_rate)
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(sp)

    def read_keyboard(self):
        """Non-blocking keyboard read."""
        try:
            tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)

                if key == "\x1b":  # ESC
                    self.get_logger().info("Exiting...")
                    self.cleanup()
                    rclpy.shutdown()
                    sys.exit(0)
                elif key == " ":
                    self.vx = self.vy = self.vz = self.yaw_rate = 0.0
                    self.get_logger().info("Hover (zero velocity)")
                elif key == "t":
                    self.arm()
                    self.set_offboard_mode()
                elif key == "l":
                    self.vx = self.vy = self.yaw_rate = 0.0
                    self.vz = 1.0  # positive z = descend in NED
                    self.get_logger().info("Landing...")
                elif key in KEY_MAP:
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
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def cleanup(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except Exception:
            pass

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
