"""
Kabsch algorithim for computing optimal rotation and translation between two
sets of paired coordinates
"""

import numpy as np
from math import sqrt, cos, sin

def kabsch(A:np.ndarray, B:np.ndarray):
    """
    Use Kabsch algorithim to compute rotation & translation from `B` to `A`.
    Matrices `A` and `B` should have identical shapes of (n, d), where n is
    the number of points and d is the number of dimensions (i.e. d=3 corresonds
    to points having x, y, z coordinates). Modified from
    https://gist.github.com/oshea00/dfb7d657feca009bf4d095d4cb8ea4be

    The transfomation represents the form:
        a = R*b + t

    where
        a = (d,1) vector (ex. [x, y, z]^T)
        b = (d,1) vector (ex. [x, y, z]^T)
        R = (d,d) rotation matrix
        t = (d,1) translation vector

    Args:
        A (np.ndarray): (n,2) or (n,3) array of final points (after transformation)
        B (np.ndarray): (n,2) or (n,3) array of initial points 
    
    Returns:
        R (np.ndarray): rotation matrix
        t (np.ndarray): translation vector
        H (np.ndarray): homogeneous matrix
    """
    assert len(A) == len(B), "Point set A and B must contain same number of points"
    assert A.shape[1] == B.shape[1], "Point set A and B must have same dimension"
    assert A.shape[1] in [2,3], "Point sets must be (n,2) or (n,3) shape"

    # Total number of points
    n = len(A)
    # Dimesionality of data
    d = A.shape[1]

    # Centroids of point sets for centering
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # Translate point sets to center by subtracting centroids
    AA = A - np.tile(centroid_A, (n, 1))
    BB = B - np.tile(centroid_B, (n, 1))

    # SVD for rotation matrix
    H = np.matmul(np.transpose(BB),AA)
    U, S, Vt = np.linalg.svd(H)
    R = np.matmul(Vt.T ,U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        Vt[-1,:] *= -1
        R = np.matmul(Vt.T ,U.T)
    
    # Compute translation
    t = np.matmul(-R, centroid_B.reshape(d, 1)) + centroid_A.reshape(d, 1)

    # Construct homogenous matrix
    HH = np.concatenate((R,t), axis=1)
    H = np.concatenate((HH, np.zeros((1, d+1))), axis=0)
    H[-1,-1] = 1

    return R, t, H

def rigid_transform_vector(R:np.ndarray, t:np.ndarray, x:np.ndarray):
    """
    Compute rigid transformation of vector with shape (d,1)

    Evaluates y = R*x + t where:
        x = (d,1) vector (ex. [x, y, z]^T)
        R = (d,d) rotation matrix
        t = (d,1) translation vector

    Args:
        R (np.ndarray): (d,d) rotation matrix
        t (np.ndarray): (d,1) translation vector
        x (np.ndarray): (d,1) vector to transform
    """
    assert R.shape[1] == x.shape[0]
    assert t.shape[0] == x.shape[0]
    return np.matmul(R,x) + t

def rigid_transform_matrix(R:np.ndarray, t:np.ndarray, X:np.ndarray):
    """
    Compute rigid transformation of vector with shape (d,1)

    Evaluates y = R*X^T + t where:
        X = (n,d) matrix (like [[x1, y1, z1]
                                [x2, y2, z2]])
        R = (d,d) rotation matrix
        t = (d,1) translation vector

    Args:
        R (np.ndarray): (d,d) rotation matrix
        t (np.ndarray): (d,1) translation vector
        x (np.ndarray): (n,d) matrix to transform
    """
    assert R.shape[1] == X.shape[1]
    assert t.shape[0] == X.shape[1]
    return (np.matmul(R,X.T) + t).T


if __name__ == "__main__":
    th = np.pi/2 #theta
    c = cos(th)
    s = sin(th)
    R = np.matrix([[c, -s],
                   [s,  c]])
    t = np.matrix([[.65],
                   [9.87]])
    B = np.matrix([[5.46,9.4232],
                   [10.57,8.65],
                   [0.34,6]])
    A = (np.matmul(R,B.T) + t).T
    Rhat, that, Hhat = kabsch(A, B)
    A2 = (np.matmul(Rhat,B.T) + that).T
    assert np.all(rigid_transform_matrix(Rhat, that, B) == A2)
    err = A - A2
    err = np.multiply(err, err)
    err = np.sum(err)
    rmse = sqrt(err / len(A))
    print("Delta R:\n",R-Rhat)
    print("\nDelta t:\n",t-that)
    print("\nDelta A:\n", A-A2)
    print("\nRMSE: ",rmse)

    import timeit
    SETUP_CODE = '''
import numpy as np
from math import cos, sin
from src.tracking.kabsch import kabsch
th = np.pi/2 #theta
c = cos(th)
s = sin(th)
R = np.matrix([[c, -s],
                [s,  c]])
t = np.matrix([[.65],
                [9.87]])
B = np.matrix([[5.46,9.4232],
                [10.57,8.65],
                [0.34,6]])
A = (np.matmul(R,B.T) + t).T
'''
    MAIN_CODE = '''
kabsch(A, B)
'''
    print(timeit.timeit(setup=SETUP_CODE,stmt=MAIN_CODE, number =1000))