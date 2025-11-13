#!/usr/bin/env python3
"""
Simple test script to check TurtleBot controller without full simulation.
This script tests the controller logic and publishes commands directly.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import time

class SimpleTestController(Node):
    def __init__(self):
        super().__init__('simple_test_controller')
        
        # Create publisher for cmd_vel
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_callback)  # 10 Hz
        
        self.start_time = time.time()
        self.get_logger().info("Simple test controller started")
        
    def control_callback(self):
        """Simple control callback that publishes basic movement commands."""
        t = time.time() - self.start_time
        
        msg = Twist()
        
        # Simple trajectory: move forward for 5 seconds, then turn
        if t < 5.0:
            msg.linear.x = 0.2  # Move forward at 0.2 m/s
            msg.angular.z = 0.0
            self.get_logger().info(f"Moving forward: t={t:.1f}s")
        elif t < 8.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # Turn at 0.5 rad/s
            self.get_logger().info(f"Turning: t={t:.1f}s")
        elif t < 13.0:
            msg.linear.x = 0.2  # Move forward again
            msg.angular.z = 0.0
            self.get_logger().info(f"Moving forward again: t={t:.1f}s")
        else:
            msg.linear.x = 0.0  # Stop
            msg.angular.z = 0.0
            self.get_logger().info(f"Stopped: t={t:.1f}s")
            
        self.pub.publish(msg)

def main():
    rclpy.init()
    
    controller = SimpleTestController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()