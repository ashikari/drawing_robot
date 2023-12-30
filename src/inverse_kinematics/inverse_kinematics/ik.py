#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__("inverse_kinematics")
        self.counter = 0
        self.get_logger().info("Inverse Kinematics(IK) Node has been started")
        self.create_timer(0.1, self.timer_callback)

        self.publisher_ = self.create_publisher(Twist, "robot_cmd_vel", 10)
        

    def timer_callback(self):
        self.get_logger().info("HERE WE ARE: " + str(self.counter))
        self.publish_cmd()
        self.counter += 1

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 100.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 100.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()