#!/usr/bin/env python3
"""Broadcast odometry TF and a bounded path. Frame aliases must be coincident."""
import copy
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster

class TfAssist(Node):
    def __init__(self):
        super().__init__('tf_assist')
        self.frame = self.declare_parameter('world_frame', '').value
        self.body = self.declare_parameter('body_frame', '').value
        self.broadcast = self.declare_parameter('broadcast_tf', True).value
        maximum = self.declare_parameter('path_length', 3000).value
        if maximum <= 0: raise ValueError('path_length must be positive')
        self.path = deque(maxlen=maximum)
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(Path, 'path', 10)
        self.odom_pub = self.create_publisher(Odometry, 'out_odom', 10)
        self.create_subscription(Odometry, 'odom', self.receive, qos_profile_sensor_data)
        self.create_timer(0.5, self.publish_path)

    def receive(self, msg):
        output = copy.deepcopy(msg)
        output.header.frame_id = self.frame or msg.header.frame_id
        output.child_frame_id = self.body or msg.child_frame_id or 'base_link'
        self.odom_pub.publish(output)
        if self.broadcast:
            tf = TransformStamped(header=output.header, child_frame_id=output.child_frame_id)
            tf.transform.translation.x = output.pose.pose.position.x
            tf.transform.translation.y = output.pose.pose.position.y
            tf.transform.translation.z = output.pose.pose.position.z
            tf.transform.rotation = output.pose.pose.orientation
            self.br.sendTransform(tf)
        self.path.append(PoseStamped(header=output.header, pose=output.pose.pose))

    def publish_path(self):
        if self.path: self.pub.publish(Path(header=self.path[-1].header, poses=list(self.path)))

if __name__ == '__main__':
    rclpy.init()
    node = TfAssist()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
