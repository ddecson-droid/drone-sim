#!/usr/bin/env python3
"""
PX4 TF Broadcaster - Converts PX4 VehicleOdometry (NED frame) to ROS TF (ENU frame).

PX4 uses NED (North-East-Down) / FRD (Front-Right-Down)
ROS uses ENU (East-North-Up) / FLU (Front-Left-Up)

Transform:
  x_enu =  y_ned
  y_enu =  x_ned
  z_enu = -z_ned
  q_enu = NED quaternion rotated 180 degrees about X axis

Subscribes: /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry)
Publishes:  /tf (tf2_msgs/TFMessage)
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import VehicleOdometry


class PX4TFBroadcaster(Node):
    def __init__(self):
        super().__init__("px4_tf_broadcaster")
        self.tf_broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odometry_callback,
            qos,
        )

        self.get_logger().info("PX4 TF Broadcaster started. Waiting for /fmu/out/vehicle_odometry...")

    def _ned_to_enu_quaternion(self, q_ned):
        """
        Convert NED (FRD) quaternion to ENU (FLU) quaternion.

        PX4 quaternion [w, x, y, z] rotates from NED to body-FRD.
        We need a quaternion that rotates from ENU to body-FLU.

        FRD -> FLU is a 180-degree rotation about the X axis.
        """
        qw, qx, qy, qz = q_ned

        # Rotation from FRD to FLU: 180 deg about X axis
        # Equivalently: multiply q_ned by (0, 1, 0, 0) on the right
        # q_flu = q_ned * q_flip, where q_flip = (cos(pi/2), sin(pi/2), 0, 0) = (0, 1, 0, 0)
        # Using Hamilton product:
        w = -qx
        x = qw
        y = -qz
        z = qy

        # Normalize
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm > 1e-9:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm

        return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))

    def odometry_callback(self, msg: VehicleOdometry):
        # PX4 position in NED [x_north, y_east, z_down]
        x_ned, y_ned, z_ned = msg.position[0], msg.position[1], msg.position[2]

        # NED -> ENU
        x_enu = y_ned
        y_enu = x_ned
        z_enu = -z_ned

        q_ned = msg.q  # [w, x, y, z]
        q_enu = self._ned_to_enu_quaternion(q_ned)

        # Publish odom -> base_link transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x_enu
        t.transform.translation.y = y_enu
        t.transform.translation.z = z_enu
        t.transform.rotation = q_enu

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PX4TFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
