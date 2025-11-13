"""
CBF controller implementations for TurtleBot3.

This file contains a set of CBF controller implementations adapted from
the qrotor_sim_examples for use with TurtleBot3 in ROS2.
"""

import numpy as np
from scipy import sparse
import rclpy
from rclpy.node import Node

try:
    import osqp
    OSQP_AVAILABLE = True
except ImportError:
    OSQP_AVAILABLE = False
    print("Warning: OSQP not available, CBF optimization will be disabled")

from .pointcloud import PointcloudTurtlebot
from .depth_proc import DepthProc

class CBFQPR1:
    """
    Implements a generic relative degree 1 CBF-QP controller.
    This does not interface directly with a dynamics instance.
    """
    def __init__(self, nominalCtrlEval, alpha, POSQP):
        """
        Init function for a CBF-QP Controller. This controller works over the 
        relative degree 1 dynamics of the system and outputs a velocity
        vector for safe navigation.

        Args:
            nominalCtrlEval (Function): Evaluation function from Nominal Controller object
            alpha (float): constant on h in CBF constraint
            POSQP (Sparse matrix): weight matrix in cost function
        """
        # Store CBF parameters
        self.alpha = alpha

        # Store OSQP cost weight matrix -> convert to scipy sparse csc matrix
        self.POSQP = POSQP

        # Create a nominal controller
        self.nominalCtrlEval = nominalCtrlEval

        # Store a variable for the OSQP problem instance
        self.prob = None

    def solve_opti(self, h, Lfh, Lgh, t):
        """
        Set up the CBF-QP optimization problem using OSQP.
        Note: This function generates a new OSQP instance each time it is called
        Strange issues with updating the parameters causing zeroing out happened otherwise.
        
        Args:
            h (float): Barrier function value
            Lfh (numpy.ndarray): Lie derivative Lfh
            Lgh (numpy.ndarray): Lie derivative Lgh
            t (float): current time in simulation
            
        Returns:
            numpy.ndarray: Optimized control input
        """
        if not OSQP_AVAILABLE:
            # Fallback to nominal controller if OSQP not available
            return self.nominalCtrlEval(t)
        
        # Assemble a constraint matrix and vector of the correct shape
        Anp = -Lgh
        A = sparse.csc_matrix(Anp.tolist())
        b = self.alpha * h + Lfh
        q = -self.nominalCtrlEval(t)  # q = -kX
    
        # Create an OSQP object and store in self.prob
        self.prob = osqp.OSQP()

        # Setup workspace and change alpha parameter
        self.prob.setup(P=self.POSQP, q=q, A=A, u=b, verbose=False, alpha=1.0)

        # Solve problem
        res = self.prob.solve().x

        # Return optimization output
        return res.reshape((res.size, 1))
    
    def eval_cbf_input(self, h, Lfh, Lgh, t):
        """
        Evaluate the input of the CBF-QP controller.
        
        Args:
            h (float): Barrier function value
            Lfh (numpy.ndarray): Lie derivative Lfh  
            Lgh (numpy.ndarray): Lie derivative Lgh
            t (float): current time in simulation
            
        Returns:
            numpy.ndarray: safe velocity that allows for position tracking
        """
        # Return the input
        return self.solve_opti(h, Lfh, Lgh, t)


class CBFQPR2:
    """
    Implements a generic relative degree 2 CBF-QP controller.
    This does not interface directly with a dynamics instance.
    """
    def __init__(self, nominalCtrlEval, alpha0, alpha1, POSQP):
        """
        Init function for a CBF-QP Controller. This controller works over the 
        relative degree 2 dynamics of the system.

        Args:
            nominalCtrlEval (Function): Evaluation function from Nominal Controller object
            alpha0 (float): constant on h in CBF constraint
            alpha1 (float): constant on hDot
            POSQP (Sparse matrix): weight matrix in cost function
        """
        # Store CBF parameters
        self.alpha0 = alpha0  # alpha for h
        self.alpha1 = alpha1  # alpha for hDot

        # Store OSQP cost weight matrix -> convert to scipy sparse csc matrix
        self.POSQP = POSQP

        # Create a nominal controller
        self.nominalCtrlEval = nominalCtrlEval

        # Store a variable for the OSQP problem instance
        self.prob = None

    def solve_opti(self, h, Lfh, Lgh, Lf2h, LgLfh, t):
        """
        Set up the CBF-QP optimization problem using OSQP.
        
        Args:
            h (float): Barrier function value
            Lfh (numpy.ndarray): First Lie derivative
            Lgh (numpy.ndarray): Control Lie derivative
            Lf2h (numpy.ndarray): Second Lie derivative
            LgLfh (numpy.ndarray): Mixed Lie derivative
            t (float): current time in simulation
            
        Returns:
            numpy.ndarray: Optimized control input
        """
        if not OSQP_AVAILABLE:
            # Fallback to nominal controller if OSQP not available
            return self.nominalCtrlEval(t)
            
        # Assemble a constraint matrix and vector of the correct shape
        Anp = -LgLfh
        A = sparse.csc_matrix(Anp.tolist())
        b = self.alpha0 * h + self.alpha1 * Lfh + Lf2h
        q = -self.nominalCtrlEval(t)  # q = -kX
    
        # Create an OSQP object and store in self.prob
        self.prob = osqp.OSQP()

        # Setup workspace and change alpha parameter
        self.prob.setup(P=self.POSQP, q=q, A=A, u=b, verbose=False, alpha=1.0)

        # Solve problem
        res = self.prob.solve().x

        # Return optimization output
        return res.reshape((res.size, 1))
    
    def eval_cbf_input(self, h, Lfh, Lgh, Lf2h, LgLfh, t):
        """
        Evaluate the input of the CBF-QP controller.
        
        Args:
            h (float): Barrier function value
            Lfh (numpy.ndarray): First Lie derivative
            Lgh (numpy.ndarray): Control Lie derivative  
            Lf2h (numpy.ndarray): Second Lie derivative
            LgLfh (numpy.ndarray): Mixed Lie derivative
            t (float): current time in simulation
            
        Returns:
            numpy.ndarray: safe control input
        """
        # Return the input
        return self.solve_opti(h, Lfh, Lgh, Lf2h, LgLfh, t)


class TurtleBotCBFController:
    """
    TurtleBot CBF Controller using depth processing and optimization.
    This is adapted from the 3D quadrotor controller for TurtleBot use.
    """
    def __init__(self, observer, trajectory, lidar, node, DELTA=0.3, alpha0=1.0, alpha1=1.0):
        """
        Init function for a CBF-QP Controller for TurtleBot3.

        Args:
            observer: state observer object
            trajectory: trajectory for the controller to track
            lidar: LiDAR sensor object
            node: ROS2 node for logging
            DELTA (float): buffer distance for safety
            alpha0 (float): CBF parameter alpha for h
            alpha1 (float): CBF parameter alpha for hDot
        """
        self.observer = observer
        self.trajectory = trajectory
        self.lidar = lidar
        self.node = node
        
        # Initialize pointcloud with dummy data
        dummy_ptcloud = {"ptcloud": np.zeros((3, 1)), "stateVec": np.zeros((3, 1))}
        self.pointcloud = PointcloudTurtlebot(dummy_ptcloud)
        self.depthProc = DepthProc(self.pointcloud)
        
        # Store CBF parameters
        self.DELTA = DELTA  # buffer distance
        self.alpha0 = alpha0  # alpha for h
        self.alpha1 = alpha1  # alpha for hDot

        # Store OSQP cost weight matrix -> convert to scipy sparse csc matrix
        if OSQP_AVAILABLE:
            self.POSQP = sparse.csc_matrix(np.eye(2))  # 2D for TurtleBot [v, omega]
        else:
            self.POSQP = np.eye(2)

        # Store nominal controller
        self.kX = 1.0  # feedback gain

        # Store a CBF-QP implementation
        if OSQP_AVAILABLE:
            self.CBFQP = CBFQPR1(self.nominal_eval, self.alpha0, self.POSQP)

        # Store control input
        self._u = np.zeros((2, 1))

    def nominal_eval(self, t):
        """
        Nominal control evaluation for tracking.
        
        Args:
            t (float): current time
            
        Returns:
            numpy.ndarray: nominal control input [linear_vel, angular_vel]
        """
        x = self.observer.get_pos()
        if self.trajectory:
            xD = self.trajectory.pos(t)
            # Simple proportional control for position tracking
            error = xD - x
            
            # Convert position error to TurtleBot control commands
            # Linear velocity proportional to forward error
            linear_vel = self.kX * error[0, 0]  # x-direction error
            
            # Angular velocity proportional to lateral error (simplified)
            angular_vel = self.kX * error[1, 0]  # y-direction error
            
            # Limit velocities for safety
            linear_vel = np.clip(linear_vel, -0.5, 0.5)   # max 0.5 m/s
            angular_vel = np.clip(angular_vel, -1.0, 1.0)  # max 1.0 rad/s
            
            return np.array([[linear_vel], [angular_vel]])
        return np.zeros((2, 1))

    def h(self, q, qC):
        """
        Computes the control barrier function for TurtleBot.
        
        Args:
            q (numpy.ndarray): 3x1 current position vector of the system
            qC (numpy.ndarray): 3x1 closest point in the pointcloud to the robot
            
        Returns:
            float: value of barrier function
        """
        # Add on a buffer length for safety margin
        return (q - qC).T @ (q - qC) - self.DELTA**2
    
    def compute_lie_derivatives(self, q, qDot, A, b, c):
        """
        Computes approximations of the Lie derivatives of the barrier function
        for TurtleBot kinematics.
        
        Args:
            q (numpy.ndarray): 3x1 current position vector
            qDot (numpy.ndarray): 3x1 current velocity vector
            A (numpy.ndarray): 3x3 quadratic weight matrix
            b (numpy.ndarray): 3x1 linear weight vector
            c (numpy.ndarray): 1x1 affine weight
            
        Returns:
            tuple: (Lfh, Lgh) Lie derivatives
        """
        # For TurtleBot kinematics: [x, y, theta], control input [v, omega]
        # State dynamics: xdot = v*cos(theta), ydot = v*sin(theta), thetadot = omega
        
        # Current orientation
        theta = q[2, 0]
        
        # Gradient of quadratic approximation
        grad_h = 2 * A @ q + b
        
        # Lie derivative Lfh (drift dynamics)
        # For TurtleBot, drift is typically zero for position
        Lfh = 0.0
        
        # Lie derivative Lgh (control dynamics)
        # g(x) = [cos(theta), sin(theta), 0; 0, 0, 1] for controls [v, omega]
        g = np.array([[np.cos(theta), 0],
                      [np.sin(theta), 0], 
                      [0, 1]])
        
        Lgh = grad_h.T @ g  # 1x2 matrix
        
        return Lfh, Lgh

    def eval_input(self, t, save=False):
        """
        Evaluate the input of the CBF controller.
        
        Args:
            t (float): current time in simulation
            save (bool): flag to save data
            
        Returns:
            numpy.ndarray: safe velocity that allows for position tracking
        """
        try:
            # Get the position and velocity of the turtlebot
            q = self.observer.get_pos()
            qDot = self.observer.get_vel()

            # Update the pointcloud with the latest LiDAR reading
            lidar_data = self.lidar.get_pointcloud()
            if lidar_data is not None and lidar_data["ptcloud"].shape[1] > 0:
                self.pointcloud.update_pointcloud(lidar_data)

                # Compute the approximation of the CBF using 2D fit
                try:
                    A, b, c = self.depthProc.get_cbf_quad_fit_2D(q, self.h)
                except Exception as e:
                    if self.node:
                        self.node.get_logger().warn(f"CBF fit failed: {str(e)}, using fallback")
                    # Use identity for fallback
                    A = np.eye(3)
                    b = np.zeros((3, 1))
                    c = np.array([[1.0]])

                # Compute the Lie derivatives
                Lfh, Lgh = self.compute_lie_derivatives(q, qDot, A, b, c)

                # Compute closest point and barrier function
                qC, _ = self.depthProc.get_closest_point(q)
                h_val = self.h(q, qC)
                
                if OSQP_AVAILABLE:
                    # Set up optimization problem and solve
                    self._u = self.CBFQP.eval_cbf_input(h_val, Lfh, Lgh, t)
                else:
                    # Fallback controller when OSQP not available
                    if h_val < 0:
                        # Emergency stop if barrier violated
                        self._u = np.zeros((2, 1))
                        if self.node:
                            self.node.get_logger().warn("Barrier violated! Emergency stop.")
                    else:
                        # Use nominal controller
                        self._u = self.nominal_eval(t)
                        
                if save and self.node:
                    # Log CBF data for analysis
                    self.node.get_logger().info(f"CBF value: {h_val:.3f}, Control: [{self._u[0,0]:.3f}, {self._u[1,0]:.3f}]")
                    
            else:
                # No LiDAR data available, use nominal controller
                self._u = self.nominal_eval(t)
                if self.node and save:
                    self.node.get_logger().info(f"No LiDAR data - using nominal control: [{self._u[0,0]:.3f}, {self._u[1,0]:.3f}]")

            return self._u
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"CBF Controller Error: {str(e)}")
                self.node.get_logger().error(f"Using emergency fallback - nominal controller")
            # Use nominal controller as fallback
            self._u = self.nominal_eval(t)
            return self._u

    def get_input(self):
        """
        Retrieves input stored in class parameter.
        
        Returns:
            numpy.ndarray: most recent input stored in class parameter
        """
        return self._u