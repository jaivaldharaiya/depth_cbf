"""
State estimation module for TurtleBot3 using ROS 2.

This module provides state observers for getting position, orientation, 
and velocity information from the TurtleBot3's odometry system.
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from tf_transformations import euler_from_quaternion
import numpy as np

class StateObserver:
    """Base class for state observers."""
    
    def __init__(self, node):
        """
        Init function for state observer
        """
        self.node = node
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self.node)
  
    def get_state(self):
        """Get the current state - to be implemented by subclasses."""
        raise NotImplementedError

    def get_position(self):
        """
        Use the odometry data on the Turtlebot to return the current position
        """
        state_not_found = True
        while state_not_found and rclpy.ok():
            try:
                trans = self.tfBuffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
                x, y = trans.transform.translation.x, trans.transform.translation.y
                state_not_found = False
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                # self.node.get_logger().debug(f"Transform lookup failed: {str(e)}")
                pass
        return x, y

class EgoTurtlebotObserver:
    """State observer for a single TurtleBot3."""
    
    def __init__(self, node):
        """
        Init function for a state observer for a single turtlebot within a system of N turtlebots
        Args:
            node: ROS 2 node instance
        """
        self.node = node
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self.node)
        self.singleInputDimn = 2
        self.singleStateDimn = 3
    
    def get_state(self):
        """
        Returns a potentially noisy observation of the system state
        Returns:
            numpy.ndarray: 3x1 array containing [x, y, yaw]
        """
        try:
            trans = self.tfBuffer.lookup_transform("odom", "base_footprint", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            x, y = trans.transform.translation.x, trans.transform.translation.y
            (roll, pitch, yaw) = euler_from_quaternion([
                trans.transform.rotation.x, 
                trans.transform.rotation.y,
                trans.transform.rotation.z, 
                trans.transform.rotation.w
            ])
            return np.array([x, y, yaw]).reshape((3, 1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.node.get_logger().warn(f"Transform lookup failed: {str(e)}, returning zero state")
            return np.array([0.0, 0.0, 0.0]).reshape((3, 1))
    
    def get_orient(self):
        """Get the current orientation (yaw angle)."""
        return self.get_state()[2, 0]
    
    def get_pos(self):
        """Get the current position as 3D vector (z=0)."""
        return np.vstack((self.get_state()[0:2].reshape((2, 1)), 0))
    
    def get_vel(self, previous_u_input=None):
        """
        Returns a potentially noisy measurement of the derivative of the state vector
        
        Args:
            previous_u_input: 2x1 numpy array, previous control input applied
            
        Returns:
            3x1 numpy array, observed derivative of the state vector
        """
        # Default to zero input if not provided
        if previous_u_input is None:
            previous_u_input = np.zeros((2, 1))
        
        # Ensure proper shape
        if previous_u_input.size == 0:
            previous_u_input = np.zeros((2, 1))
        elif previous_u_input.shape != (2, 1):
            previous_u_input = previous_u_input.reshape((2, 1))
        
        PHI = self.get_state()[2, 0]
        xDot = np.array([[np.cos(PHI), 0], [np.sin(PHI), 0], [0, 1]]) @ previous_u_input
        return xDot.reshape((3, 1))