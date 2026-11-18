#!/usr/bin/env python3
"""Publish a configurable stationary odometry stream for interface debugging."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSender(Node):
    def __init__(self):
        super().__init__('odom_sender')
        self.msg = Odometry()
        self.msg.header.frame_id = self.declare_parameter('world_frame', 'world').value
        self.msg.child_frame_id = self.declare_parameter('body_frame', 'base_link').value
        self.msg.pose.pose.orientation.w = 1.0
        for axis in ('x', 'y', 'z'):
            setattr(self.msg.pose.pose.position, axis, self.declare_parameter(axis, 0.0).value)
        rate = self.declare_parameter('rate', 10.0).value
        if rate <= 0: raise ValueError('rate must be positive')
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_timer(1.0/rate, self.tick)

    def tick(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rclpy.init()
    node = OdomSender()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
