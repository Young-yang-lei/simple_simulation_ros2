#!/usr/bin/env python3
"""Publish roll/pitch/yaw in degrees from odometry and IMU quaternions."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Joy
from geometry_msgs.msg import Vector3Stamped

class EulerNode(Node):
    def __init__(self):
        super().__init__('odom_to_euler')
        self.euler = self.create_publisher(Vector3Stamped, 'euler', 10)
        self.imu_euler = self.create_publisher(Vector3Stamped, 'imueuler', 10)
        self.ctrl = self.create_publisher(Vector3Stamped, 'ctrlout', 10)
        self.create_subscription(Odometry, 'odom', lambda m: self.convert(m.header, m.pose.pose.orientation, self.euler), qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu', lambda m: self.convert(m.header, m.orientation, self.imu_euler), qos_profile_sensor_data)
        self.create_subscription(Joy, 'ctrlin', self.joy, qos_profile_sensor_data)

    def convert(self, header, q, pub):
        norm = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
        if norm < 1e-9:
            return
        w, x, y, z = (v/norm for v in (q.w, q.x, q.y, q.z))
        msg = Vector3Stamped(header=header)
        msg.vector.x = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
        msg.vector.y = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))))
        msg.vector.z = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
        pub.publish(msg)

    def joy(self, joy):
        if len(joy.axes) < 4:
            return
        msg = Vector3Stamped(header=joy.header)
        msg.vector.x, msg.vector.y, msg.vector.z = joy.axes[0], joy.axes[1], -joy.axes[3]
        self.ctrl.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    node = EulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
