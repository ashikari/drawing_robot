#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        self.get_logger().info("Inverse Kinematics Node has been started")


def main(args=None):
    rclpy.init(args=args)
    inverse_kinematics = InverseKinematics()
    rclpy.spin(inverse_kinematics)
    rclpy.shutdown()

if __name__ == '__main__':
    main()