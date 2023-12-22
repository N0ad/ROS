import rclpy
from rclpy.node import Node
import sys
from math import pi
from geometry_msgs.msg import Twist
import time

class Circle(Node):

    def __init__(self):
        super().__init__('circle')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.radius = self.declare_parameter('radius', 0.1).get_parameter_value().double_value
        self.d = 0
        self.timer = self.create_timer(1, self.move_to_goal)
    def move_to_goal(self):
        radius = self.radius
        tw = Twist()
        tw.linear.x = (radius + self.d) * pi * 0.5
        tw.angular.z = 0.5 * pi
        self.d = self.d + 0.2
        self.publisher.publish(tw)
        

def main():
    rclpy.init()
    circle= Circle()
    rclpy.spin(circle)
    circle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
