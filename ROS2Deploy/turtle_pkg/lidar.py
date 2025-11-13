"""
LiDAR sensor interface module for TurtleBot3 using ROS 2.

This module provides an interface to the TurtleBot3's LiDAR sensor,
processing laser scan data and converting it to point clouds.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class Lidar:
    """LiDAR sensor interface class."""
    
    def __init__(self, node):
        """
        Initialize the LiDAR sensor interface.
        
        Args:
            node: ROS 2 node instance
        """
        self.node = node
        self._sensor_topic = "/scan"
        self._sensor_sub = self.node.create_subscription(
            LaserScan, 
            self._sensor_topic, 
            self.SensorCallback, 
            1
        )
        self._range_max = 5.0
        self._range_min = 0.15
        self._ptcloudSize = 375
        
        # Initialize with valid pointcloud data (obstacles at max range in all directions)
        self._pointcloud = np.ones((3, self._ptcloudSize)) * self._range_max
        self._pointcloud[2, :] = 0.0  # Set z-coordinates to 0 for 2D navigation
        
        # Set first few points in a circle around the robot for safety
        angles = np.linspace(0, 2*np.pi, self._ptcloudSize)
        for i in range(self._ptcloudSize):
            self._pointcloud[0, i] = self._range_max * np.cos(angles[i])
            self._pointcloud[1, i] = self._range_max * np.sin(angles[i])
            self._pointcloud[2, i] = 0.0
        
        self.node.get_logger().info(f"LiDAR sensor initialized, subscribing to {self._sensor_topic}")
        self.node.get_logger().info(f"Initial pointcloud shape: {self._pointcloud.shape}")

    def SensorCallback(self, msg):
        """
        Callback to process sensor measurements.
        
        Args:
            msg: LaserScan message
        """
        # Initialize pointcloud with max range values to ensure we always have valid data
        angles = np.linspace(0, 2*np.pi, self._ptcloudSize)
        for i in range(self._ptcloudSize):
            self._pointcloud[0, i] = self._range_max * np.cos(angles[i])
            self._pointcloud[1, i] = self._range_max * np.sin(angles[i])
            self._pointcloud[2, i] = 0.0
        
        # Process actual LaserScan data if available and valid
        if len(msg.ranges) > 0:
            # Loop over all ranges in the LaserScan
            for idx, r in enumerate(msg.ranges):
                # Skip if too far or too close or invalid
                if np.isnan(r) or np.isinf(r) or r > self._range_max or r < self._range_min:
                    continue
                    
                # Get angle of this ray
                angle = msg.angle_min + idx * msg.angle_increment
                
                # Convert to Cartesian coordinates
                if idx < self._ptcloudSize:
                    self._pointcloud[0][idx] = r * np.cos(angle)
                    self._pointcloud[1][idx] = r * np.sin(angle)
                    self._pointcloud[2][idx] = 0.0

    def get_pointcloud(self):
        """
        Get the current point cloud data.
        
        Returns:
            dict: Dictionary with "ptcloud" and "stateVec" keys
        """
        return {
            "ptcloud": self._pointcloud,
            "stateVec": np.zeros((3, 1))  # Placeholder state vector
        }