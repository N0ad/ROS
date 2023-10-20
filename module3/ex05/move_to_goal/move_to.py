import time
import sys
import math as m

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class MoveToGoal(Node):

    def __init__(self):
        super().__init__("move_to_goal_node")

        self._cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self._pose_sub    = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 0)
        self.pose_data    = Pose(x=5.54, y=5.54, theta=0.)
        
    def pose_callback(self, msg):
        self.pose_data = msg

    def __call__(self, x, y, theta):
        self._move_to_goal(x, y, theta)

    def _is_moving(self): 
        self.get_logger().info(f"{self._get_turtle_pos()}")
        while(self._get_turtle_pos().linear_velocity or self._get_turtle_pos().angular_velocity):
            time.sleep(0.1)
        time.sleep(3)
        
    def _get_turtle_pos(self):
        return self.pose_data
    
    def _send_turtle_msg(self, x_speed : float = 0., angle_speed : float = 0.):
        msg = Twist()
        msg.linear.x = x_speed
        msg.angular.z = angle_speed

        self._cmd_vel_pub.publish(msg)
        time.sleep(0.5)

    def _move_to_goal(self, x, y, theta):
        start_position = self._get_turtle_pos()
        self.get_logger().info(f"Start: {start_position.x} {start_position.y} {start_position.theta}")
        self.get_logger().info(f"Goal: {x} {y} {theta}")

        x_d = x - start_position.x
        y_d = y - start_position.y

        ang_target = theta - start_position.theta

        dst = m.sqrt(x_d**2 + y_d**2)
        ra = m.sqrt(dst**2 / (2 * (1 - m.cos(ang_target))))
        dst_target = ra * ang_target

        self.get_logger().info(f"\t{dst_target} {ang_target}")

        self._send_turtle_msg(dst_target, angle_speed=ang_target)
        
def main():
    rclpy.init()

    mvg = MoveToGoal()
    mvg(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))

    rclpy.spin(mvg)

    mvg.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()