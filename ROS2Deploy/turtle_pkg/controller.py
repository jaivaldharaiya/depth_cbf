"""
Control system module for TurtleBot3 using ROS 2.

This module provides various controllers including feedback linearization
and Control Barrier Function (CBF) based controllers for safe navigation.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, Point
import sys
import time

try:
    from scipy import sparse
    import osqp
    ADVANCED_CBF_AVAILABLE = True
except ImportError:
    print("Warning: scipy/osqp not available, using simplified CBF controller")
    sparse = None
    ADVANCED_CBF_AVAILABLE = False

# Import the advanced CBF controllers
try:
    from .cbf_controllers import TurtleBotCBFController, CBFQPR1, CBFQPR2
    from .depth_proc import DepthProc
    CBF_AVAILABLE = True
except ImportError:
    print("Warning: Advanced CBF controllers not available, using basic controller only")
    CBF_AVAILABLE = False

class Controller:
    """Base controller class for feedback controllers."""
    
    def __init__(self, observer, trajectory=None, node=None):
        """
        Skeleton class for feedback controllers
        
        Args:
            observer: state observer object
            trajectory: trajectory for the controller to track
            node: ROS 2 node instance
        """
        self.observer = observer
        self.trajectory = trajectory
        self.node = node
        if self.node:
            self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.previous_u_input = np.zeros((2, 1))
    
    def eval_input(self, t):
        """
        Solve for and return control input
        
        Args:
            t (float): time in simulation
            
        Returns:
            numpy.ndarray: input vector, as determined by controller
        """
        if self.trajectory:
            self._u = self.trajectory.vel(t)
        else:
            self._u = np.zeros((2, 1))
        return self._u
    
    def get_input(self):
        """
        Retrieves input stored in class parameter
        
        Returns:
            numpy.ndarray: most recent input stored in class parameter
        """
        return self._u
    
    def apply_input(self):
        """Apply the computed control input by publishing to cmd_vel topic."""
        if not self.node:
            return
            
        msg = Twist()
        msg.linear.x = float(self._u[0, 0])
        msg.angular.z = float(self._u[1, 0])
        self.previous_u_input = self._u
        self.pub.publish(msg)

class TurtlebotFBLin(Controller):
    """Feedback linearizing controller for TurtleBot3."""
    
    def __init__(self, observer, trajectory, frequency, node):
        """
        Class for a feedback linearizing controller for a single turtlebot
        
        Args:
            observer: state observer object
            trajectory: trajectory object
            frequency: control frequency
            node: ROS 2 node instance
        """
        super().__init__(observer, trajectory, node)
        
        # Store the time step dt for integration
        self.dt = 1.0 / frequency
        
        # Store feedback gains
        self.k1 = 4.0
        self.k2 = 4.0
        
        # Store value for integral
        self.vDotInt = 0.0
        
        # Previous control input
        self.previous_u_input = np.array([0., 0.]).reshape((2, 1))
        if self.node:
            self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
    
    def eval_z_input(self, t):
        """
        Solve for the input z to the feedback linearized system.
        Use linear tracking control techniques to accomplish this.
        """
        # Get the state of turtlebot
        q = self.observer.get_state()
        
        # Get the derivative of q
        qDot = self.observer.get_vel(self.previous_u_input)
        
        # Get the trajectory states
        xD, vD, aD = self.trajectory.get_state(t)
        
        # Find a control input z to the augmented system
        e = np.array([[xD[0, 0], xD[1, 0]]]).T - np.array([[q[0, 0], q[1, 0]]]).T
        eDot = np.array([[vD[0, 0], vD[1, 0]]]).T - np.array([[qDot[0, 0], qDot[1, 0]]]).T
        z = aD[0:2].reshape((2, 1)) + self.k2 * eDot + self.k1 * e
        
        return z
    
    def eval_w_input(self, t, z):
        """
        Solve for the w input to the system
        
        Args:
            t (float): current time in the system
            z (numpy.ndarray): z input to the system
        """
        # Get the current phi
        phi = self.observer.get_state()[2, 0]
        
        # Get the (xdot, ydot) velocity
        qDot = self.observer.get_vel(self.previous_u_input)[0:2]
        v = np.linalg.norm(qDot)
        
        # Eval A(q)
        Aq = np.array([[np.cos(phi), -v * np.sin(phi)], 
                       [np.sin(phi), v * np.cos(phi)]])
        
        # Invert to get the w input - use pseudoinverse to avoid problems
        w = np.linalg.pinv(Aq) @ z
        
        return w

    def eval_input(self, t):
        """
        Solves for the control input using a feedback linearizing controller.
        
        Args:
            t (float): current time in simulation
        """
        # Get the z input to the system
        z = self.eval_z_input(t)
        
        # Get the w input to the system
        w = self.eval_w_input(t, z)
        
        # Integrate the w1 term to get v
        self.vDotInt += w[0, 0] * self.dt
        
        # Return the [v, omega] input
        self._u = np.array([[self.vDotInt, w[1, 0]]]).T
        return self._u

# Comprehensive CBF controller using advanced implementation
class TurtlebotCBFAdvanced(Controller):
    """
    Advanced CBF-based controller for TurtleBot3 using full CBF optimization.
    This uses the integrated CBF controllers from the qrotor_sim_examples.
    """
    
    def __init__(self, observer, pointcloud, trajectory, lidar, node, DELTA=0.3, alpha0=1.0, alpha1=1.0):
        """
        Initialize the advanced CBF controller.
        
        Args:
            observer: state observer object
            pointcloud: pointcloud object
            trajectory: trajectory object
            lidar: LiDAR sensor object
            node: ROS 2 node instance
            DELTA (float): safety buffer distance
            alpha0 (float): CBF parameter alpha for h
            alpha1 (float): CBF parameter alpha for hDot
        """
        super().__init__(observer, trajectory, node)
        
        self.pointcloud = pointcloud
        self.lidar = lidar
        
        if CBF_AVAILABLE and ADVANCED_CBF_AVAILABLE:
            # Use the advanced CBF controller
            self.cbf_controller = TurtleBotCBFController(
                observer, trajectory, lidar, node, DELTA, alpha0, alpha1
            )
            self.use_advanced = True
            if node:
                node.get_logger().info("Using advanced CBF controller with optimization")
        else:
            # Fallback to simplified CBF
            self.cbf_controller = TurtlebotCBFR1(observer, pointcloud, trajectory, lidar, node)
            self.use_advanced = False
            if node:
                node.get_logger().warn("Using simplified CBF controller (missing dependencies)")
        
        # Store publisher
        self.previous_u_input = np.array([0., 0.]).reshape((2, 1))
        if self.node:
            self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def eval_input(self, t, save=False):
        """
        Evaluate the input using the CBF controller.
        
        Args:
            t (float): current time in simulation
            save (bool): save the CBF value for analysis
            
        Returns:
            numpy.ndarray: safe velocity for TurtleBot navigation
        """
        try:
            # Use the appropriate CBF controller
            if self.use_advanced:
                self._u = self.cbf_controller.eval_input(t, save)
            else:
                self._u = self.cbf_controller.eval_input(t, save)
                
            return self._u
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in CBF controller evaluation: {str(e)}")
            # Return zero input in case of error
            self._u = np.zeros((2, 1))
            return self._u
    
    def apply_input(self):
        """Apply the computed control input."""
        if not self.node:
            return
            
        msg = Twist()
        msg.linear.x = float(self._u[0, 0])
        msg.angular.z = float(self._u[1, 0])
        self.previous_u_input = self._u
        self.pub.publish(msg)

# Simplified CBF controller for when external libraries are not available
class TurtlebotCBFR1(Controller):
    """
    Simplified CBF-based controller for TurtleBot3.
    This is a fallback controller when external CBF libraries are not available.
    """
    
    def __init__(self, observer, pointcloud, trajectory, lidar, node):
        """
        Initialize the CBF controller.
        
        Args:
            observer: state observer object
            pointcloud: pointcloud object
            trajectory: trajectory object
            lidar: LiDAR sensor object
            node: ROS 2 node instance
        """
        super().__init__(observer, trajectory, node)
        
        self.pointcloud = pointcloud
        self.lidar = lidar
        
        # Store state feedback gain
        self.kX = 1.0
        
        # Set delta for CBF
        self.DELTA = 0.125 + 0.15
        
        # Create a fallback controller
        self.velControl = TurtlebotVelTrack(observer, None, trajectory, lidar, node)
        
        # Store publisher
        self.previous_u_input = np.array([0., 0.]).reshape((2, 1))
        if self.node:
            self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
            self.pub_data = self.node.create_publisher(Twist, '/sys_data', 10)

    def nominal_eval(self, t):
        """Nominal control evaluation."""
        x = self.observer.get_pos()
        if self.trajectory:
            xD = self.trajectory.pos(t)
            return self.kX * (xD - x)
        return np.zeros((3, 1))

    def h(self, q, qC):
        """
        Computes the control barrier function
        
        Args:
            q: current position vector of the system
            qC: closest point in the pointcloud to the robot
            
        Returns:
            float: value of barrier function
        """
        return (q - qC).T @ (q - qC) - self.DELTA**2

    def eval_input(self, t, save=False):
        """
        Evaluate the input of the simplified CBF controller.
        
        Args:
            t (float): current time in simulation
            save (bool): save the CBF value
            
        Returns:
            numpy.ndarray: safe velocity that allows for position tracking
        """
        try:
            # Get the position of the turtlebot
            q = self.observer.get_pos()
            
            # Initialize default values
            h_val = np.array([[1.0]])  # Safe default CBF value
            
            # Simple obstacle avoidance based on minimum distance
            ptcloud = self.pointcloud.get_ptcloud_s()
            if ptcloud is not None and ptcloud.ndim == 2 and ptcloud.shape[1] > 0:
                # Find closest point
                distances = np.linalg.norm(ptcloud - q, axis=0)
                if distances.size > 0:  # Check if distances array is not empty
                    min_dist = np.min(distances)
                    min_idx = np.argmin(distances)
                    qC = ptcloud[:, min_idx].reshape((3, 1))
                    
                    # Compute barrier function
                    h_val = self.h(q, qC)
                    
                    if min_dist < self.DELTA:
                        # Emergency stop if too close
                        vD = np.zeros((3, 1))
                        if self.node:
                            self.node.get_logger().warn(f"Obstacle too close! Distance: {float(min_dist):.3f}")
                    else:
                        # Use nominal controller
                        vD = self.nominal_eval(t)
                else:
                    # Empty distances array, use nominal controller
                    vD = self.nominal_eval(t)
            else:
                # No obstacles detected or invalid pointcloud, use nominal controller
                vD = self.nominal_eval(t)
            
            # Convert to turtlebot input
            self._u = self.velControl.eval_input(t, vD)
            
            # Ensure control input is valid
            if self._u is None or self._u.size == 0:
                self._u = np.zeros((2, 1))
            
            if save and self.node:
                # Publish system data
                msg = Twist()
                msg.linear.x = float(h_val[0, 0]) if isinstance(h_val, np.ndarray) else float(h_val)
                msg.linear.y = float(self._u[0, 0])
                msg.linear.z = float(self._u[1, 0])
                msg.angular.x = float(q[0, 0])
                msg.angular.y = float(q[1, 0])
                msg.angular.z = t
                self.pub_data.publish(msg)
            
            return self._u
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in CBF controller: {str(e)}")
            # Return zero input in case of error
            self._u = np.zeros((2, 1))
            return self._u
    
    def apply_input(self):
        """Apply the computed control input."""
        if not self.node:
            return
            
        msg = Twist()
        msg.linear.x = float(self._u[0, 0])
        msg.angular.z = float(self._u[1, 0])
        self.previous_u_input = self._u
        self.pub.publish(msg)

class TurtlebotVelTrack(Controller):
    """Velocity tracking controller for TurtleBot3."""
    
    def __init__(self, observer, lyapunovBarrierList, trajectory, depthCam, node):
        """
        Init function for velocity tracking controller for turtlebot
        """
        super().__init__(observer, trajectory, node)
        
        # Store controller gain
        self.kTheta = 1.0
        self.e3 = np.array([0, 0, 1])

    def eval_phi_error(self, vD):
        """
        Function to evaluate angular error.
        
        Args:
            vD: desired velocity of system
        """
        # Get current angle from observer
        phi = self.observer.get_orient()
        
        # Form phi vector, vector of current vel heading
        vHat = np.array([np.cos(phi), np.sin(phi), 0])
        
        # Get velocity vector
        vDHat = vD.reshape((3, ))
        
        # Compute desired angle
        return np.arctan2(np.dot(np.cross(vHat, vDHat), self.e3), np.dot(vHat, vDHat))

    def eval_input(self, t, vD=None):
        """
        Compute the input to the system
        
        Args:
            t: time
            vD: desired velocity (optional)
        """
        # Get the desired velocity
        if vD is None:
            # Use the trajectory desired velocity
            if self.trajectory:
                vD = self.trajectory.vel(t)
            else:
                vD = np.zeros((3, 1))
        
        # Get phi error
        ePhi = self.eval_phi_error(vD)
        
        # Assemble inputs
        if np.linalg.norm(vD) != 0:
            # Get signed norm of v based on ePhi
            if abs(ePhi) > np.pi / 2:
                # Negate sign of velocity
                v = -np.linalg.norm(vD)
                # Update ePhi using the negative velocity vector
                ePhi = self.eval_phi_error(-vD)
            else:
                v = np.linalg.norm(vD)
        else:
            # If zero divide, then ||vD|| = 0
            v = 0.0
        
        # Compute omega based on the updated ePhi
        omega = self.kTheta * ePhi
        
        # Return full input vector
        self._u = np.array([[v, omega]]).T
        return self._u