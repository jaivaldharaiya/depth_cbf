#!/usr/bin/env python3
"""
Quick diagnostic script to test if the controller is working properly.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('diagnostic_node')
        
        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Timer for diagnostics
        self.timer = self.create_timer(1.0, self.diagnostic_callback)
        
        self.scan_received = False
        self.scan_count = 0
        
        self.get_logger().info("Diagnostic node started - checking TurtleBot3 connectivity")
    
    def scan_callback(self, msg):
        """Check if we're receiving LiDAR data."""
        self.scan_received = True
        self.scan_count += 1
        
        # Log basic scan info
        valid_ranges = [r for r in msg.ranges if not np.isnan(r) and not np.isinf(r)]
        if valid_ranges:
            min_range = min(valid_ranges)
            self.get_logger().info(f"Scan #{self.scan_count}: {len(valid_ranges)} valid points, min_range={float(min_range):.2f}m")
    
    def diagnostic_callback(self):
        """Send diagnostic information and test commands."""
        self.get_logger().info(f"=== DIAGNOSTIC REPORT ===")
        self.get_logger().info(f"LiDAR data received: {self.scan_received}")
        self.get_logger().info(f"Scan messages: {self.scan_count}")
        
        # Send a simple test command
        msg = Twist()
        msg.linear.x = 0.1  # Move forward slowly
        msg.angular.z = 0.0
        
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published cmd_vel: linear.x={float(msg.linear.x)}, angular.z={float(msg.angular.z)}")
        
        # Check topics
        topic_names = self.get_topic_names_and_types()
        cmd_vel_exists = any('/cmd_vel' in name for name, _ in topic_names)
        scan_exists = any('/scan' in name for name, _ in topic_names)
        
        self.get_logger().info(f"/cmd_vel topic exists: {cmd_vel_exists}")
        self.get_logger().info(f"/scan topic exists: {scan_exists}")

def main():
    rclpy.init()
    
    node = DiagnosticNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()