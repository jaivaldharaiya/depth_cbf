"""
Point cloud processing module for TurtleBot3.

This module provides point cloud data structures and utilities for 
spatial coordinate transformations and obstacle detection.
"""

import numpy as np
from scipy.spatial import KDTree

class Pointcloud:
    """
    Master class for pointclouds. Interfaces with sensor data.
    Includes utilities for computing pointcloud in the spatial frame,
    storing and updating pointclouds and the states at which they 
    were taken.
    """
    def __init__(self, ptcloudDict):
        """
        Init function for pointclouds.
        
        Args:
            ptcloudDict (dict): Dictionary containing "ptcloud" and "stateVec" keys
        """
        # Store pointcloud dictionary
        self._ptcloudDict = None
        self.ptcloudQ = None
        self.q = None
        self.ptcloudS = None
        self.kdtree = None

        # Update the class params with the pointcloud
        self.update_pointcloud(ptcloudDict)
    
    def get_ptcloud_q(self):
        """Returns the pointcloud in the robot frame."""
        return self.ptcloudQ
    
    def get_state(self):
        """Returns the state of the robot when the pointcloud was taken."""
        return self.q
    
    def get_ptcloud_s(self):
        """Retrieves the spatial pointcloud."""
        return self.ptcloudS
    
    def update_statevec(self, q):
        """Updates the state vector attribute."""
        self.q = q
    
    def update_ptcloud_q(self, ptcloud_q):
        """Updates the pointcloud in robot frame."""
        self.ptcloudQ = ptcloud_q
    
    def update_ptcloudDict(self, ptcloudDict):
        """
        Update the pointcloud dictionary with a new pointcloud dictionary.
        Also updates ptcloud_q and statevec attributes.
        """
        self._ptcloudDict = ptcloudDict
        self.update_ptcloud_q(ptcloudDict["ptcloud"])
        self.update_statevec(ptcloudDict["stateVec"])

    def compute_rotation(self, theta):
        """
        Compute the rotation matrix from the robot frame to the world frame.
        
        Args:
            theta: angle of rotation
            
        Returns:
            numpy.ndarray: rotation matrix between spatial and robot frames
        """
        Rsq = np.array([[1, 0, 0], 
                        [0, np.cos(theta), -np.sin(theta)], 
                        [0, np.sin(theta), np.cos(theta)]])
        return Rsq
    
    def get_pos_orient_photo(self):
        """Get the 3D position vector and orientation of the system."""
        # Extract XYZ position
        qPhoto = self.q[0:3].reshape((3, 1))
        # Extract orientation angle when photo was taken
        thetaPhoto = self.q[4, 0]
        # Compute and return the rotation
        return qPhoto, self.compute_rotation(thetaPhoto)
    
    def calc_ptcloud_s(self):
        """
        Calculates the pointcloud in the spatial frame. Updates the class attribute.
        """
        # Check if we have valid pointcloud data
        if self.ptcloudQ is None:
            # Initialize with a default safe pointcloud (single point far away)
            self.ptcloudS = np.array([[10.0], [0.0], [0.0]])
            return self.ptcloudS
            
        # Check if ptcloudQ is a numpy array and has valid size
        if isinstance(self.ptcloudQ, np.ndarray):
            if self.ptcloudQ.size == 0:
                self.ptcloudS = np.array([[10.0], [0.0], [0.0]])
                return self.ptcloudS
        elif hasattr(self.ptcloudQ, '__len__'):
            if len(self.ptcloudQ) == 0:
                self.ptcloudS = np.array([[10.0], [0.0], [0.0]])
                return self.ptcloudS
        else:
            # Unknown type, use safe default
            self.ptcloudS = np.array([[10.0], [0.0], [0.0]])
            return self.ptcloudS
            
        # Calculate position and orientation at time of photo
        qPhoto, RPhoto = self.get_pos_orient_photo()
        
        # Check dimensions before matrix multiplication
        if (not isinstance(self.ptcloudQ, np.ndarray) or 
            self.ptcloudQ.ndim < 2 or 
            self.ptcloudQ.shape[1] == 0):
            # Initialize with a default safe pointcloud
            self.ptcloudS = np.array([[10.0], [0.0], [0.0]])
            return self.ptcloudS
            
        # Convert the ptcloud into the spatial frame and store in class attribute
        self.ptcloudS = RPhoto @ self.ptcloudQ + qPhoto
        return self.ptcloudS
    
    def update_pointcloud(self, ptcloudDict):
        """
        Master update function. Updates the dictionary and attributes
        and computes the pointcloud in the spatial frame.
        """
        # Check if we have valid pointcloud data
        if ptcloudDict is None or "ptcloud" not in ptcloudDict:
            return
            
        # Validate pointcloud dimensions
        ptcloud = ptcloudDict["ptcloud"]
        if ptcloud is None:
            return
            
        # Check if ptcloud is a numpy array and has valid size
        if isinstance(ptcloud, np.ndarray):
            if ptcloud.size == 0:
                return
        elif hasattr(ptcloud, '__len__'):
            if len(ptcloud) == 0:
                return
        else:
            # If it's neither array nor has length, skip
            return
            
        # Update the dictionary and robot frame attributes
        self.update_ptcloudDict(ptcloudDict)
        # Update the world frame pointcloud
        self.calc_ptcloud_s()
        # Update the KD tree only if we have valid spatial pointcloud
        if (self.ptcloudS is not None and 
            isinstance(self.ptcloudS, np.ndarray) and 
            self.ptcloudS.ndim >= 2 and 
            self.ptcloudS.shape[1] > 0):
            self.kdtree = KDTree(self.get_ptcloud_s().T)

class PointcloudQrotor(Pointcloud):
    """
    Quadrotor pointcloud class. Interfaces with a depth camera.
    Includes utilities for computing pointcloud in the spatial frame,
    storing and updating pointclouds and the states at which they 
    were taken.
    """
    def __init__(self, ptcloudDict):
        """
        Init function for pointclouds.
        
        Args:
            ptcloudDict (dict): Dictionary containing "ptcloud" and "statevec" keys
        """
        # Call the super init function
        super().__init__(ptcloudDict)

    def compute_rotation(self, theta):
        """
        Compute the rotation matrix from the quadrotor frame to the world frame.
        
        Args:
            theta: angle of rotation about the x-axis
            
        Returns:
            numpy.ndarray: rotation matrix between spatial and quadrotor frames
        """
        Rsq = np.array([[1, 0, 0], 
                        [0, np.cos(theta), -np.sin(theta)], 
                        [0, np.sin(theta), np.cos(theta)]])
        return Rsq
    
    def get_pos_orient_photo(self):
        """
        Get the 3D position vector and orientation of the system.
        
        Returns:
            tuple: (position, rotation_matrix)
        """
        # Extract XYZ position of quadrotor when photo was taken
        qPhoto = self.q[0:3].reshape((3, 1))
        # Extract orientation angle of quadrotor when photo was taken
        thetaPhoto = self.q[3, 0]
        return qPhoto, self.compute_rotation(thetaPhoto)
    
class PointcloudQrotor3D(Pointcloud):
    """
    3D Quadrotor pointcloud class
    """
    def __init__(self, ptcloudDict):
        """
        Init function for pointclouds.
        
        Args:
            ptcloudDict (dict): Dictionary containing "ptcloud" and "statevec" keys
        """
        # Call the super init function
        super().__init__(ptcloudDict)
    
    def get_pos_orient_photo(self):
        """
        Get the 3D position vector and orientation of the system.
        
        Returns:
            tuple: (position, rotation_matrix)
        """
        # Extract XYZ position of quadrotor when photo was taken
        qPhoto = self.q[0:3].reshape((3, 1))
        # Extract rotation matrix of quadrotor when photo was taken
        Rphoto = self.q[3:12].reshape((3, 3))
        return qPhoto, Rphoto

    def update_ptcloudDict(self, ptcloudDict):
        """
        Override the pointcloud dict to only use the N closest points.
        """
        N = 1000  # number of columns to pick
        self._ptcloudDict = ptcloudDict
        indexList = np.argsort(np.linalg.norm(ptcloudDict["ptcloud"], axis=0))
        sortedPtcloud = ptcloudDict["ptcloud"][:, indexList]
        self.update_ptcloud_q(sortedPtcloud[:, 0:N])
        self.update_statevec(ptcloudDict["stateVec"])

    def update_pointcloud(self, ptcloudDict):
        """
        Override update function for 3D processing.
        """
        # Print the pointcloud size
        super().update_pointcloud(ptcloudDict)

class PointcloudTurtlebot(Pointcloud):
    """
    TurtleBot pointcloud class. Interfaces with LiDAR sensor data.
    Includes utilities for computing pointcloud in the spatial frame,
    storing and updating pointclouds and the states at which they 
    were taken.
    """
    def __init__(self, ptcloudDict):
        """
        Init function for TurtleBot pointclouds.
        
        Args:
            ptcloudDict (dict): Dictionary containing "ptcloud" and "stateVec" keys
        """
        # Call the super init function
        super().__init__(ptcloudDict)

    def compute_rotation(self, phi):
        """
        Compute the rotation matrix from the turtlebot frame to the world frame.
        
        Args:
            phi: angle of rotation about the z-axis
            
        Returns:
            numpy.ndarray: rotation matrix between spatial and turtlebot frames
        """
        Rsq = np.array([[np.cos(phi), -np.sin(phi), 0], 
                        [np.sin(phi), np.cos(phi), 0], 
                        [0, 0, 1]])
        return Rsq
    
    def get_pos_orient_photo(self):
        """
        Get the 3D position vector and orientation of the TurtleBot.
        
        Returns:
            tuple: (position, rotation_matrix)
        """
        # Extract XYZ position of turtlebot when scan was taken
        qPhoto = np.vstack((self.q[0:2].reshape((2, 1)), 0))
        # Extract orientation angle of turtlebot when scan was taken
        thetaPhoto = self.q[2, 0]
        return qPhoto, self.compute_rotation(thetaPhoto)