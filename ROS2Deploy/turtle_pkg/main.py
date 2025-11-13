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

from std_srvs.srv import SetBool

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
        time_duration = 10  # Increased to 10 seconds for slower movement
        trajectory = Trajectory(start_position, end_position, time_duration)

        # Lidar
        lidar = Lidar(node)

        # Pointcloud -> initialize with proper structure
        lidar_data = lidar.get_pointcloud()
        ptcloudDict = {}
        ptcloudDict["ptcloud"] = lidar_data["ptcloud"]
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

        # Enable motor power
        motor_client = node.create_client(SetBool, '/motor_power')
        if motor_client.wait_for_service(timeout_sec=5.0):
            motor_request = SetBool.Request()
            motor_request.data = True
            future = motor_client.call_async(motor_request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            if future.result() and future.result().success:
                node.get_logger().info("✅ Motor power enabled successfully")
            else:
                node.get_logger().warn("⚠️  Failed to enable motor power, but continuing...")
        else:
            node.get_logger().warn("⚠️  Motor power service not available, but continuing...")

        # Loop until the node is killed with Ctrl-C
        control_count = 0
        while rclpy.ok():
            t = node.get_clock().now().nanoseconds / 1e9 - start_time
            t1 = node.get_clock().now().nanoseconds / 1e9
            
            # Debug: Print trajectory progress every 100 iterations
            if control_count % 100 == 0:
                current_pos = observer.get_state()
                target_pos = trajectory.pos(t)
                node.get_logger().info(f"Time: {t:.2f}s, Current: [{current_pos[0,0]:.2f}, {current_pos[1,0]:.2f}], Target: [{target_pos[0,0]:.2f}, {target_pos[1,0]:.2f}]")
            
            # Update the pointcloud dictionary and pass it into the pointcloud object
            ptcloudDict["stateVec"] = observer.get_state()
            lidar_data = lidar.get_pointcloud()
            ptcloudDict["ptcloud"] = lidar_data["ptcloud"]
            pointcloud.update_pointcloud(ptcloudDict)

            # Evaluate and apply the control input
            u = controller.eval_input(t, save=True)
            controller.apply_input()
            
            # Debug: Print control input every 100 iterations
            if control_count % 100 == 0:
                node.get_logger().info(f"Control input: linear.x={u[0,0]:.3f}, angular.z={u[1,0]:.3f}")
            
            t2 = node.get_clock().now().nanoseconds / 1e9

            # Update the frequency list
            freqList.append(1/(t2 - t1))
            control_count += 1
            
            # Reset trajectory when complete to keep robot moving
            if t > time_duration:
                node.get_logger().info("Trajectory complete, restarting...")
                start_time = node.get_clock().now().nanoseconds / 1e9  # Reset start time
                control_count = 0
            
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