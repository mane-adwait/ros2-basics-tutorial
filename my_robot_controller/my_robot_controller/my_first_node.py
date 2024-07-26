#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node): # MyNode inherits from Node in rclpy.node.

    def __init__(self): # Create a constructor.
        super().__init__("first_node")
        # super() refers to the parent class i.e. the Node class in rclpy.node.
        # super().__init__() is the constructor of the parent class.
        # It has the node name as an input argument.
        # Node name: "first_node". This is used to run the node.
        # Note that the node name is different from the file name.

        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback) # (timer_period, callback_function)

    def timer_callback(self):
        self.get_logger().info("Anduril " + str(self.counter_) )
        self.counter_ += 1


def main(args=None):  # To install with ROS2.
    rclpy.init(args=args) # Initialize ROS2 communications.
    
    # Node is defined here.
    node = MyNode()
    rclpy.spin(node) # Keep the node alive. 

    rclpy.shutdown() # Shutdown ROS2 communications.
 
if __name__ == '__main__':  # To execute directly from the terminal.
    main()