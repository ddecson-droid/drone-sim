#!/usr/bin/env python3
"""
PX4 Vehicle Status Monitor.

Subscribes to /fmu/out/vehicle_status and logs arming state,
flight mode, and system health at a low rate.

Subscribes: /fmu/out/vehicle_status (px4_msgs/VehicleStatus)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus


# PX4 nav_state (flight mode) mapping
NAV_STATES = {
    0: "MANUAL",
    1: "ALTCTL",
    2: "POSCTL",
    3: "AUTO_MISSION",
    4: "AUTO_LOITER",
    5: "AUTO_RTL",
    6: "AUTO_LAND",
    7: "AUTO_RTGS",
    8: "AUTO_READY",
    9: "AUTO_TAKEOFF",
    10: "AUTO_FOLLOW_TARGET",
    11: "AUTO_VTOL_TAKEOFF",
    12: "AUTO_PRECLAND",
    14: "OFFBOARD",
    17: "AUTO_LANDING",
    18: "AUTO_GOTO",
}

ARMING_STATES = {
    1: "INIT",
    2: "STANDBY",
    3: "ARMED",
    4: "STANDBY_ERROR",
}


class PX4StatusMonitor(Node):
    def __init__(self):
        super().__init__("px4_status_monitor")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.status_callback,
            qos,
        )

        self.last_nav_state = -1
        self.last_arming_state = -1
        self.get_logger().info("PX4 Status Monitor started. Waiting for /fmu/out/vehicle_status...")

    def status_callback(self, msg: VehicleStatus):
        nav_state = msg.nav_state
        arming_state = msg.arming_state

        if nav_state != self.last_nav_state or arming_state != self.last_arming_state:
            nav_name = NAV_STATES.get(nav_state, f"UNKNOWN({nav_state})")
            arm_name = ARMING_STATES.get(arming_state, f"UNKNOWN({arming_state})")

            self.get_logger().info(
                f"PX4 Status -- Arming: {arm_name} | "
                f"Mode: {nav_name} | "
                f"Preflight: {'OK' if msg.pre_flight_checks_pass else 'FAIL'}"
            )

            self.last_nav_state = nav_state
            self.last_arming_state = arming_state


def main(args=None):
    rclpy.init(args=args)
    node = PX4StatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
