#!/usr/bin/env python3
"""
TurtleBot3 Control Barrier Function Controller - Main Node

This module implements the main controller node for safe navigation of TurtleBot3
using Control Barrier Functions (CBF) with point cloud-based obstacle avoidance.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import os
import sys

# Import package modules using relative imports
from .state_estimation import EgoTurtlebotObserver
from .trajectory import Trajectory
from .controller import TurtlebotFBLin, TurtlebotCBFR1, TurtlebotCBFAdvanced
from .lidar import Lidar
from .pointcloud import PointcloudTurtlebot

def task_controller():
    """
    Controls a turtlebot whose position is denoted by turtlebot_frame,
    to go to a position denoted by target_frame
    """

    # Initialize ROS 2
    rclpy.init()
    node = rclpy.create_node('turtlebot_controller')
    
    node.get_logger().info("Starting TurtleBot CBF Controller...")

    # Define a list containing the control frequencies
    freqList = []

    try:
        # Initialization Time
        start_time = node.get_clock().now().nanoseconds / 1e9
        frequency = 300
        
        # Observer
        observer = EgoTurtlebotObserver(node)

        # Trajectory
        start_position = np.array([[0, 0, 0]]).T
        end_position = np.array([[3, 0, 0]]).T
        time_duration = 1
        trajectory = Trajectory(start_position, end_position, time_duration)

        # Lidar
        lidar = Lidar(node)

        # Pointcloud -> initialize with an empty dictionary
        ptcloudDict = {}
        ptcloudDict["ptcloud"] = lidar.get_pointcloud()
        ptcloudDict["stateVec"] = np.zeros((3, 1))
        pointcloud = PointcloudTurtlebot(ptcloudDict)

        # Set to true to apply CBF-QP control
        useCBFQP = True 
        useAdvancedCBF = True  # Set to True to use the advanced CBF implementation
        
        if not useCBFQP:
            controller = TurtlebotFBLin(observer, trajectory, frequency, node)
            node.get_logger().info("Using Feedback Linearization controller")
        elif useAdvancedCBF:
            # Use the advanced CBF controller with full optimization
            controller = TurtlebotCBFAdvanced(observer, pointcloud, trajectory, lidar, node, 
                                            DELTA=0.3, alpha0=1.0, alpha1=1.0)
            node.get_logger().info("Using Advanced CBF-QP controller with optimization")
        else:
            # Use the simplified CBF controller
            controller = TurtlebotCBFR1(observer, pointcloud, trajectory, lidar, node)
            node.get_logger().info("Using Simplified CBF controller")

        node.get_logger().info("Controller initialized. Starting control loop...")

        # Loop until the node is killed with Ctrl-C
        while rclpy.ok():
            t = node.get_clock().now().nanoseconds / 1e9 - start_time
            t1 = node.get_clock().now().nanoseconds / 1e9
            
            # Update the pointcloud dictionary and pass it into the pointcloud object
            ptcloudDict["stateVec"] = observer.get_state()
            ptcloudDict["ptcloud"] = lidar.get_pointcloud()
            pointcloud.update_pointcloud(ptcloudDict)

            # Evaluate and apply the control input
            controller.eval_input(t, save=True)
            controller.apply_input()
            t2 = node.get_clock().now().nanoseconds / 1e9

            # Update the frequency list
            freqList.append(1/(t2 - t1))
            rclpy.spin_once(node, timeout_sec=1.0/frequency)

    except Exception as e:
        node.get_logger().error(f"Error in controller: {str(e)}")
    finally:
        # After shutdown, save the timing test
        node.get_logger().info("Shutting down controller...")
        node.destroy_node()
        rclpy.shutdown()
        
        # Save the timing data
        save = True
        if save and freqList:
            try:
                # Create data directory path relative to package
                data_dir = os.path.join(current_dir, "..", "data")
                os.makedirs(data_dir, exist_ok=True)
                npy_path = os.path.join(data_dir, "timingtest.npy")
                np.save(npy_path, np.array(freqList))
                print(f"Saved timing data to {npy_path}")
            except Exception as e:
                print(f"Failed to save timing data: {str(e)}")

def main(args=None):
    """Main entry point for the ROS 2 node."""
    try:
        task_controller()
    except KeyboardInterrupt:
        print("Controller interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {str(e)}")

if __name__ == '__main__':
    main()