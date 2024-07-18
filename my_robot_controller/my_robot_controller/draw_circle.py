#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher()
        self.get_logger().info("Draw Circle Node has been started")

def main(args=None):
    rclpy.init(args=args)
    rclpy.shutdown()