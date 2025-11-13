"""
Depth processing module for TurtleBot3 CBF implementation.

This file contains utilities for processing depth data and implementing
Control Barrier Function (CBF) quadratic approximations for safe navigation.
"""
import numpy as np

class DepthProc:
    """
    Master class for depth image processing and CBF computation.
    """
    def __init__(self, pointcloud):
        """
        Init function for depth processing.
        
        Args:
            pointcloud (Pointcloud): Pointcloud object
        """
        # Store pointcloud and barrier objects
        self.pointcloud = pointcloud

        # Store a mesh centered at the origin for fast computation
        self.mesh0 = self.get_3d_mesh_data(np.zeros((3, 1)))

    def get_closest_point(self, qS):
        """
        Returns closest point to qS.
        
        Args:
            qS (numpy.ndarray): 3x1 position vector
            
        Returns:
            tuple: (closest_point, index)
        """
        # Find closest point using the kd tree
        dist, index = self.pointcloud.kdtree.query(qS[:, 0], k=1)

        # Extract the point at iMin and return it
        return self.pointcloud.get_ptcloud_s()[:, index].reshape((3, 1)), index
    
    def get_closest_points(self, qSMatrix):
        """
        Get the *set* of closest points to the columns of qSMatrix.
        
        Args:
            qSMatrix (numpy.ndarray): 3xN matrix of position vectors
            
        Returns:
            numpy.ndarray: 3xN matrix of closest points
        """
        # Find closest points using the kd tree
        dist, index = self.pointcloud.kdtree.query(qSMatrix.T)

        # Get all of the points at each index and return
        index = index.tolist()
        ptcloudS = self.pointcloud.get_ptcloud_s()
        closestPts = np.zeros((3, len(index)))
        for i in range(len(index)):
            # Extract the point at i and add it to the matrix
            closestPts[:, i] = ptcloudS[:, index[i]]
        return closestPts

    def get_neighbors(self, q, r):
        """
        Calculate the neighbors of a point q in the pointcloud within a ball of radius r.
        
        Args:
            q (numpy.ndarray): 3x1 position vector
            r (float): radius for neighbor search
            
        Returns:
            numpy.ndarray: 3xN matrix of neighboring points
        """
        # Find indices of all points within distance r of point q
        indexList = self.pointcloud.kdtree.query_ball_point(q[0:3, 0], r=r)

        # Form a matrix using the index list
        neighbors = np.zeros((3, len(indexList)))
        for i in range(len(indexList)):
            # Get the ith column of the pointcloud
            neighbors[:, i] = self.pointcloud.get_ptcloud_s()[:, indexList[i]]
        
        # Return nearest neighbors
        return neighbors

    def get_normal_vec(self, q, A, b, c):
        """
        Get the normal vector to the surface q^TAq + b^Tq +c at a point q = [x, y].
        Returns outward unit normal vector in R3.
        
        Args:
            q (numpy.ndarray): 2x1 vector [x, y]
            A, b, c: Quadratic fit parameters
            
        Returns:
            numpy.ndarray: 3x1 unit normal pointing outwards from the surface
        """
        gradQ = (A + A.T) @ q + b
        surfGrad = np.vstack((-gradQ, 1))  # flip sign to get outward vector
        nHat = surfGrad / np.linalg.norm(surfGrad)
        return nHat
    
    def fit_quad_form(self, X, Y):
        """
        Fit a quadratic form x^T A x + b^Tx + c to a data matrix X = [x1; x2; ...; xN] in RN.
        Each column in X is an input point. Each row represents an element of the input vector.
        
        Args:
            X (numpy.ndarray): N x M array, m columns of points in Rn 
            Y (numpy.ndarray): M x 1 array, vector of output data values 
            
        Returns:
            tuple: (A, b, c) quadratic fit parameters
        """
        # Get the dimension of X
        N = X.shape[0]

        # Get number of data points
        M = Y.size

        # Form a data matrix D that has N^2 columns and m rows (for m the number of data points)
        D = np.zeros((M, N**2))
        index = 0
        for i in range(N):
            for j in range(N):
                # Compute combination XIXJ. Get the all ith entries of X
                XI = X[i, :]
                XJ = X[j, :]

                # Compute element wise product and store in D
                D[:, index] = np.multiply(XI, XJ)

                # Increment the index in D
                index += 1
        
        # Form the rest of the data matrix
        for i in range(N):
            D = np.hstack((D, X[i, :].reshape((M, 1))))
        D = np.hstack((D, np.ones((M, 1))))

        # Find the regression weights via pseudoinverse
        w = np.linalg.pinv(D) @ Y

        # Unpack the weights
        A = w[0:N**2].reshape((N, N))  # reshape Aij into a matrix
        b = w[N**2:-1].reshape((N, 1))
        c = w[-1].reshape((1, 1))
        
        # Return the quadratic terms
        return A, b, c
    
    def get_3d_mesh_data(self, q, delta=0.005, gridSize=3):
        """
        Function to turn a set of axes [X; Y; Z] into a 3D grid. Then, returns the points in the grid as a matrix.
        
        Args:
            q (numpy.ndarray): 3x1 center point for mesh generation
            delta (float): grid sampling size
            gridSize (int): size of grid used to generate data
            
        Returns:
            numpy.ndarray: 3xM array containing points in the 3D grid as columns
        """
        # Generate the corners of the mesh for fitting
        q0 = q - gridSize * delta
        q1 = q + gridSize * delta
        
        # Generate the mesh for fitting (NOTE: linspace returns a 3D array that must be transposed)
        meshPts = np.linspace(q0, q1, 2*gridSize)[:, :, 0].T

        # Check the shape of the data
        if meshPts.shape[0] == 3:
            # Generate a meshgrid from the axes
            X, Y, Z = np.meshgrid(meshPts[0, :], meshPts[1, :], meshPts[2, :])

            # Return data
            return np.array([X.ravel(), Y.ravel(), Z.ravel()])
        elif meshPts.shape[0] == 2:
            # Generate a meshgrid from the axes
            Y, Z = np.meshgrid(meshPts[0, :], meshPts[1, :])

            # Return data with zeros filled in for X
            return np.array([Y.ravel() * 0, Y.ravel(), Z.ravel()])
    
    def eval_cbf_mesh(self, data, h):
        """
        Evaluate the CBF on a dataset D = [x; y; z]. Each column is a point to evaluate the CBF at.
        Note: this function uses matrix computations directly on the closest point matrices
        and then extracts the diagonal of the matrix to get the CBF values. This gives an enormous (~10x)
        speed boost compared to computing iteratively over columns.
        
        Args:
            data (numpy.ndarray): 3xN array [X;Y;Z] matrix to evaluate the CBF at
            h (function): h(q, qC), the CBF function of the robot position q and the closest point qC
            
        Returns:
            numpy.ndarray: Nx1 array of CBF values computed for each column in the data matrix
        """
        # Evaluate the closest points to each column in the data matrix
        QC = self.get_closest_points(data)

        # Call the function directly on the matrices and extract the diagonal, which contains the CBF values
        return np.diagonal(h(data, QC)).reshape((data.shape[1], 1))
    
    def get_cbf_quad_fit_2D(self, q, h, delta=0.005, gridSize=5, returnMesh=False, useXYZ=[False, True, True]):
        """
        Perform a quadratic fit x^T A x + b^Tx + c of a CBF function from XYZ coordinates.
        NOTE: This is the 2D function, NOT the 3D function. This fit will simply be performed
        over YZ coordinates, and X will be ignored. This is a useful function for visualization
        and for planar systems such as the turtlebot or planar quadrotor, but should not be
        used for full 3-dimensional systems.

        Args:
            q (numpy.ndarray): 3x1 current position of the system. CBF will be fit in a grid around this position.
            h (function): CBF function h(q)
            delta (float): grid sampling size for CBF fit
            gridSize (int): size of grid used to generate data
            returnMesh (bool): return the mesh of points used to generate the data
            useXYZ (list): select which coordinates to generate the mesh in
            
        Returns:
            tuple: A, b, c CBF fit parameters (3x3), (3x1), (1x1) arrays
        """
        # Generate meshgrid data
        if useXYZ[1] and useXYZ[2]:
            # Generate meshgrid with YZ
            arr1 = [q[1, 0] - i*delta for i in range(-gridSize, gridSize+1)]
            arr2 = [q[2, 0] - i*delta for i in range(-gridSize, gridSize+1)]
            data = self.get_3d_mesh_data(q[1:, 0].reshape((2, 1)), delta=delta, gridSize=gridSize)
        elif useXYZ[0] and useXYZ[1]:
            # Generate meshgrid with XY
            arr1 = [q[0, 0] - i*delta for i in range(-gridSize, gridSize+1)]
            arr2 = [q[1, 0] - i*delta for i in range(-gridSize, gridSize+1)]
            data = self.get_3d_mesh_data(q[0:2, 0].reshape((2, 1)), delta=delta, gridSize=gridSize)

        # Evaluate the CBF data
        hData = self.eval_cbf_mesh(data, h)
        
        if not returnMesh:
            return self.fit_quad_form(data, hData)
        else:
            # Get the grid
            arr1Mesh, arr2Mesh = np.meshgrid(arr1, arr2)
            A, b, c = self.fit_quad_form(data, hData)
            return A, b, c, arr1Mesh, arr2Mesh
        
    def get_cbf_quad_fit_3D(self, q, h):
        """
        Perform a quadratic fit x^T A x + b^Tx + c of a CBF function from XYZ coordinates.
        NOTE: This function performs a fit of the CBF over XYZ, not just YZ.

        Args:
            q (numpy.ndarray): 3x1 current position of the system. CBF will be fit in a grid around this position.
            h (function): CBF function h(q)
            
        Returns:
            tuple: A, b, c CBF fit parameters (3x3), (3x1), (1x1) arrays
        """
        # Generate the data mesh by shifting the precomputed mesh around the origin
        data = self.mesh0 + q

        # Evaluate the CBF over the data matrices
        hData = self.eval_cbf_mesh(data, h)

        # Perform a fit over the data
        return self.fit_quad_form(data, hData)