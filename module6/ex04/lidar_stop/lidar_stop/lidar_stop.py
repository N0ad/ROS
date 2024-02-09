import sys
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point, Quaternion
from tf_transformations import euler_from_quaternion


class Turtle(Node):
    def __init__(self):
        super().__init__("lidar_stop")
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.pose_sub = self.create_subscription(LaserScan, '/robot/scan', self.pose_callback, 1)
        self.timer = self.create_timer(0.2, self.go_forward)
        self.msg = LaserScan()

    def pose_callback(self, data):
        self.msg = data

    def go_forward(self):
        message = Twist()
        check = 1
   
        for i in range(len(self.msg.ranges)):
            if(self.msg.ranges[i] < self.msg.range_min and self.msg.ranges[i] > self.msg.range_max):
                continue

            if(self.msg.ranges[i] < 1.0):
                check = 0
                break

        if(check):
            message.linear.x = 0.1
        else:
            message.linear.x = 0.0
            
        self.publisher_.publish(message)             

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Turtle()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()