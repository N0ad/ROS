import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.declare_parameter('radius')
        self.declare_parameter('direction_of_rotation')
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        seconds, _ = self.get_clock().now().seconds_nanoseconds()
        
        radius = self.get_parameter('radius').get_parameter_value().double_value
        direction_of_rotation = self.get_parameter('direction_of_rotation').get_parameter_value().integer_value

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'
        t.transform.translation.x = radius * math.cos(seconds * direction_of_rotation)
        t.transform.translation.y = radius * math.sin(seconds * direction_of_rotation)
        #t.transform.translation.z = 0.0
        #t.transform.rotation.x = 0.0
        #t.transform.rotation.y = 0.0
        #t.transform.rotation.z = 0.0
        #t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
