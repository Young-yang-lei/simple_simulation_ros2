#!/usr/bin/env python3
"""Report receive rate and serialized traffic for a configured ROS2 topic."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

class TopicStatistics(Node):
    def __init__(self):
        super().__init__('topic_statistics')
        topic = self.declare_parameter('topic', 'odometry').value
        typename = self.declare_parameter('message_type', 'nav_msgs/msg/Odometry').value
        self.count = self.size = 0
        self.start = time.monotonic()
        self.create_subscription(get_message(typename), topic, self.receive, qos_profile_sensor_data, raw=True)
        self.create_timer(2.0, self.report)

    def receive(self, data):
        self.count += 1
        self.size += len(data)

    def report(self):
        end = time.monotonic()
        elapsed = end-self.start
        self.get_logger().info(f'{self.count/elapsed:.1f} Hz, {self.size/elapsed/1024:.1f} KiB/s')
        self.count = self.size = 0
        self.start = end

if __name__ == '__main__':
    rclpy.init()
    node = TopicStatistics()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
