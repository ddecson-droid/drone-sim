#!/usr/bin/env python3
"""
PX4 Offboard Control - Takeoff, hover, land sequence using position setpoints.

State machine: IDLE -> ARMING -> TAKEOFF -> HOVER -> LANDING -> DISARMING -> DONE

Publishes:
  /fmu/in/offboard_control_mode  (OffboardControlMode)
  /fmu/in/trajectory_setpoint     (TrajectorySetpoint)
  /fmu/in/vehicle_command         (VehicleCommand)

Subscribes:
  /fmu/out/vehicle_status         (VehicleStatus)
  /fmu/out/vehicle_odometry       (VehicleOdometry)
"""
import rclpy
import time
from enum import IntEnum
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleOdometry,
)


class State(IntEnum):
    IDLE = 0
    ARMING = 1
    TAKEOFF = 2
    HOVER = 3
    LANDING = 4
    DISARMING = 5
    DONE = 6


# PX4 vehicle command constants
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_NAV_TAKEOFF = 22


class OffboardControl(Node):
    def __init__(self):
        super().__init__("offboard_control")

        # Parameters
        self.declare_parameter("takeoff_height", -5.0)  # NED: negative = up
        self.declare_parameter("hover_time", 10.0)       # seconds
        self.declare_parameter("auto_arm", True)

        self.takeoff_height = self.get_parameter("takeoff_height").value
        self.hover_time = self.get_parameter("hover_time").value
        self.auto_arm = self.get_parameter("auto_arm").value

        # QoS for PX4
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.status_callback, qos
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.odom_callback, qos
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        # State
        self.state = State.IDLE
        self.nav_state = 0
        self.arming_state = 1  # INIT
        self.current_z = 0.0
        self.takeoff_setpoint_sent = False

        # Timers
        self.control_timer = self.create_timer(0.05, self.control_loop)   # 20 Hz
        self.offboard_timer = self.create_timer(0.1, self.offboard_heartbeat)  # 10 Hz

        self.start_time = None
        self.hover_start_time = None

        self.get_logger().info(
            f"Offboard Control started. Takeoff height: {-self.takeoff_height:.1f}m, "
            f"Hover time: {self.hover_time:.1f}s, Auto-arm: {self.auto_arm}"
        )

    def status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def odom_callback(self, msg: VehicleOdometry):
        self.current_z = msg.position[2]  # NED: negative up

    def offboard_heartbeat(self):
        """Publish offboard control mode heartbeat (required at >= 2 Hz)."""
        if self.state < State.DONE:
            mode_msg = OffboardControlMode()
            mode_msg.position = True
            mode_msg.velocity = False
            mode_msg.acceleration = False
            mode_msg.attitude = False
            mode_msg.body_rate = False
            mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_mode_pub.publish(mode_msg)

    def arm(self):
        """Send arm command."""
        self.get_logger().info("Sending ARM command...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0  # ARM
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def disarm(self):
        self.get_logger().info("Sending DISARM command...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 0.0  # DISARM
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def set_offboard_mode(self):
        """Switch to offboard flight mode."""
        self.get_logger().info("Switching to OFFBOARD mode...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0  # Main mode: custom
        cmd.param2 = 6.0  # Sub-mode: offboard
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def set_land_mode(self):
        self.get_logger().info("Switching to LAND mode...")
        cmd = VehicleCommand()
        cmd.command = VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0
        cmd.param3 = 5.0  # LAND
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def publish_position_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Publish position setpoint in NED frame (z negative = up)."""
        sp = TrajectorySetpoint()
        sp.position = [float(x), float(y), float(z)]
        sp.yaw = float(yaw)
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(sp)

    def control_loop(self):
        """Main state machine control loop (20 Hz)."""
        now = self.get_clock().now().nanoseconds / 1e9

        if self.state == State.IDLE:
            if self.auto_arm:
                self.state = State.ARMING

        elif self.state == State.ARMING:
            # Wait for vehicle to be ready, then arm
            if self.arming_state == 2:  # STANDBY
                self.arm()
                self.get_logger().info("Waiting for arm...")
            elif self.arming_state == 3:  # ARMED
                self.get_logger().info("Vehicle armed. Taking off...")
                self.state = State.TAKEOFF
                self.start_time = now

        elif self.state == State.TAKEOFF:
            # Send position setpoints slightly above current position
            # First, switch to offboard mode after a brief moment
            if not self.takeoff_setpoint_sent:
                if now - (self.start_time or now) > 1.0:
                    self.set_offboard_mode()
                    self.takeoff_setpoint_sent = True
                # Publish at current position first to satisfy offboard precheck
                self.publish_position_setpoint(z=self.current_z + 0.5)
            else:
                # Ramp up to takeoff height over 2 seconds
                elapsed = now - self.start_time - 1.0
                target_z = self.current_z + (self.takeoff_height - self.current_z) * min(elapsed / 2.0, 1.0)
                self.publish_position_setpoint(z=target_z)

                if abs(self.current_z - self.takeoff_height) < 0.5 and elapsed > 3.0:
                    self.get_logger().info(
                        f"Reached takeoff height: {-self.current_z:.1f}m. Hovering..."
                    )
                    self.state = State.HOVER
                    self.hover_start_time = now

        elif self.state == State.HOVER:
            self.publish_position_setpoint(z=self.takeoff_height)
            if self.hover_start_time and (now - self.hover_start_time) > self.hover_time:
                self.get_logger().info("Hover complete. Landing...")
                self.state = State.LANDING

        elif self.state == State.LANDING:
            # Use PX4 land mode
            self.set_land_mode()
            # Wait for descent
            if self.current_z > -0.5:  # near ground (NED down is positive)
                self.get_logger().info("On ground. Disarming...")
                self.state = State.DISARMING
            else:
                self.publish_position_setpoint(z=0.0)  # command ground level

        elif self.state == State.DISARMING:
            self.disarm()
            if self.arming_state == 2:  # STANDBY
                self.get_logger().info("Disarmed. Mission complete!")
                self.state = State.DONE

        elif self.state == State.DONE:
            pass  # Done


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Offboard control interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
