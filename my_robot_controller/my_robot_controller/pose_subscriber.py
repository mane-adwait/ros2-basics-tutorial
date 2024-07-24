#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber_ = self.create_subscription(
             Pose, "/turtle1/pose", self.pose_callback, 10) # Create a subscriber to the /turtle1/pose topic.
        
    def pose_callback(self, msg: Pose):
        self.get_logger().info(str(msg))

def main(args=None):
        rclpy.init(args=args) # Initialize ROS2 communications.
        node = PoseSubscriberNode() # Create an instance of our class.
        rclpy.spin(node) # Keep the node running until it is shutdown.
        rclpy.shutdown() # Shutdown ROS2 communications.