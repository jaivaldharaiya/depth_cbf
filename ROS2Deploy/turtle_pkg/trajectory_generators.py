"""
Trajectory generation module for TurtleBot3.

This module provides trajectory generation capabilities including
waypoint navigation and smooth path planning.
"""

import numpy as np
import rclpy
from rclpy.node import Node

class WaypointTrajectory:
    """
    Waypoint-based trajectory generator for TurtleBot3.
    Supports linear interpolation between waypoints.
    """
    
    def __init__(self, waypoints, total_time, node=None):
        """
        Initialize waypoint trajectory.
        
        Args:
            waypoints (list): List of [x, y, theta] waypoints
            total_time (float): Total time to complete trajectory
            node: ROS2 node for logging
        """
        self.waypoints = np.array(waypoints)
        self.total_time = total_time
        self.node = node
        self.n_waypoints = len(waypoints)
        
        if self.n_waypoints < 2:
            raise ValueError("Need at least 2 waypoints")
        
        # Calculate time intervals
        self.segment_time = total_time / (self.n_waypoints - 1)
        
    def pos(self, t):
        """
        Get desired position at time t.
        
        Args:
            t (float): time
            
        Returns:
            numpy.ndarray: 3x1 position vector [x, y, 0]
        """
        # Clamp time to trajectory duration
        t = max(0, min(t, self.total_time))
        
        # Find current segment
        segment_idx = min(int(t / self.segment_time), self.n_waypoints - 2)
        local_t = (t - segment_idx * self.segment_time) / self.segment_time
        
        # Linear interpolation between waypoints
        start_wp = self.waypoints[segment_idx]
        end_wp = self.waypoints[segment_idx + 1]
        
        current_pos = start_wp + local_t * (end_wp - start_wp)
        
        # Return as 3D vector (TurtleBot moves in 2D but we use 3D for consistency)
        return np.array([[current_pos[0]], [current_pos[1]], [0]])
    
    def vel(self, t):
        """
        Get desired velocity at time t.
        
        Args:
            t (float): time
            
        Returns:
            numpy.ndarray: 3x1 velocity vector [vx, vy, 0]
        """
        # Clamp time to trajectory duration
        t = max(0, min(t, self.total_time))
        
        if t >= self.total_time:
            return np.zeros((3, 1))
        
        # Find current segment
        segment_idx = min(int(t / self.segment_time), self.n_waypoints - 2)
        
        # Calculate velocity as difference between waypoints divided by segment time
        start_wp = self.waypoints[segment_idx]
        end_wp = self.waypoints[segment_idx + 1]
        
        vel_2d = (end_wp - start_wp) / self.segment_time
        
        # Return as 3D vector
        return np.array([[vel_2d[0]], [vel_2d[1]], [0]])
    
    def accel(self, t):
        """
        Get desired acceleration at time t.
        For linear segments, acceleration is zero except at waypoints.
        
        Args:
            t (float): time
            
        Returns:
            numpy.ndarray: 3x1 acceleration vector [ax, ay, 0]
        """
        # For linear interpolation, acceleration is typically zero
        return np.zeros((3, 1))
    
    def get_state(self, t):
        """
        Get complete state (position, velocity, acceleration) at time t.
        
        Args:
            t (float): time
            
        Returns:
            tuple: (position, velocity, acceleration) as 3x1 numpy arrays
        """
        return self.pos(t), self.vel(t), self.accel(t)


class CircularTrajectory:
    """
    Circular trajectory generator for TurtleBot3.
    """
    
    def __init__(self, center, radius, angular_velocity, node=None):
        """
        Initialize circular trajectory.
        
        Args:
            center (list): [x, y] center of circle
            radius (float): radius of circle
            angular_velocity (float): angular velocity (rad/s)
            node: ROS2 node for logging
        """
        self.center = np.array(center)
        self.radius = radius
        self.omega = angular_velocity
        self.node = node
        
    def pos(self, t):
        """
        Get desired position at time t.
        
        Args:
            t (float): time
            
        Returns:
            numpy.ndarray: 3x1 position vector [x, y, 0]
        """
        x = self.center[0] + self.radius * np.cos(self.omega * t)
        y = self.center[1] + self.radius * np.sin(self.omega * t)
        
        return np.array([[x], [y], [0]])
    
    def vel(self, t):
        """
        Get desired velocity at time t.
        
        Args:
            t (float): time
            
        Returns:
            numpy.ndarray: 3x1 velocity vector [vx, vy, 0]
        """
        vx = -self.radius * self.omega * np.sin(self.omega * t)
        vy = self.radius * self.omega * np.cos(self.omega * t)
        
        return np.array([[vx], [vy], [0]])
    
    def accel(self, t):
        """
        Get desired acceleration at time t.
        
        Args:
            t (float): time
            
        Returns:
            numpy.ndarray: 3x1 acceleration vector [ax, ay, 0]
        """
        ax = -self.radius * self.omega**2 * np.cos(self.omega * t)
        ay = -self.radius * self.omega**2 * np.sin(self.omega * t)
        
        return np.array([[ax], [ay], [0]])
    
    def get_state(self, t):
        """
        Get complete state (position, velocity, acceleration) at time t.
        
        Args:
            t (float): time
            
        Returns:
            tuple: (position, velocity, acceleration) as 3x1 numpy arrays
        """
        return self.pos(t), self.vel(t), self.accel(t)


class FigureEightTrajectory:
    """
    Figure-eight trajectory generator for TurtleBot3.
    """
    
    def __init__(self, center, scale, period, node=None):
        """
        Initialize figure-eight trajectory.
        
        Args:
            center (list): [x, y] center of figure-eight
            scale (float): size scaling factor
            period (float): time for one complete cycle
            node: ROS2 node for logging
        """
        self.center = np.array(center)
        self.scale = scale
        self.period = period
        self.omega = 2 * np.pi / period
        self.node = node
        
    def pos(self, t):
        """
        Get desired position at time t using parametric figure-eight equations.
        
        Args:
            t (float): time
            
        Returns:
            numpy.ndarray: 3x1 position vector [x, y, 0]
        """
        theta = self.omega * t
        
        # Parametric figure-eight (lemniscate of Gerono)
        x = self.center[0] + self.scale * np.cos(theta)
        y = self.center[1] + self.scale * np.sin(2 * theta) / 2
        
        return np.array([[x], [y], [0]])
    
    def vel(self, t):
        """
        Get desired velocity at time t.
        
        Args:
            t (float): time
            
        Returns:
            numpy.ndarray: 3x1 velocity vector [vx, vy, 0]
        """
        theta = self.omega * t
        
        vx = -self.scale * self.omega * np.sin(theta)
        vy = self.scale * self.omega * np.cos(2 * theta)
        
        return np.array([[vx], [vy], [0]])
    
    def accel(self, t):
        """
        Get desired acceleration at time t.
        
        Args:
            t (float): time
            
        Returns:
            numpy.ndarray: 3x1 acceleration vector [ax, ay, 0]
        """
        theta = self.omega * t
        
        ax = -self.scale * self.omega**2 * np.cos(theta)
        ay = -2 * self.scale * self.omega**2 * np.sin(2 * theta)
        
        return np.array([[ax], [ay], [0]])
    
    def get_state(self, t):
        """
        Get complete state (position, velocity, acceleration) at time t.
        
        Args:
            t (float): time
            
        Returns:
            tuple: (position, velocity, acceleration) as 3x1 numpy arrays
        """
        return self.pos(t), self.vel(t), self.accel(t)


def create_trajectory(trajectory_type, **kwargs):
    """
    Factory function to create different trajectory types.
    
    Args:
        trajectory_type (str): Type of trajectory ('waypoint', 'circular', 'figure_eight')
        **kwargs: Parameters for the specific trajectory type
        
    Returns:
        Trajectory object of the specified type
    """
    if trajectory_type == 'waypoint':
        return WaypointTrajectory(kwargs['waypoints'], kwargs['total_time'], kwargs.get('node'))
    elif trajectory_type == 'circular':
        return CircularTrajectory(kwargs['center'], kwargs['radius'], kwargs['angular_velocity'], kwargs.get('node'))
    elif trajectory_type == 'figure_eight':
        return FigureEightTrajectory(kwargs['center'], kwargs['scale'], kwargs['period'], kwargs.get('node'))
    else:
        raise ValueError(f"Unknown trajectory type: {trajectory_type}")