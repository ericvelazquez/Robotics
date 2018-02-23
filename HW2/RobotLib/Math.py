import numpy as np
import math

def deg2rad(theta):
    return theta*np.pi/180.

def rad2deg(theta):
    return theta*180./np.pi

def rot(theta):
    """ Creates a 2x2 rotation matrix
        Arguments:
            theta: angle of rotation -- positive theta is counter-clockwise rotation
        Returns:
            2x2 rotation matrix
    """
    s = np.sin(theta)
    c = np.cos(theta)
    return np.matrix([[c,-s],[s,c]])

def transform(x,y,theta):
    """ Creates a 3x3 transformation matrix which
        transforms points from the child frame to the parent frame
        Arguments:
            x: x coordinate of child frame origin in parent frame
            y: y coordinate of child frame origin in parent frame
            theta: orientation of child frame in parent frame
        Returns:
            3x3 transformation matrix
    """
    s = np.sin(theta)
    c = np.cos(theta)
    return np.matrix([[c,-s,x],[s,c,y],[0,0,1]])

def invert(T):
    """ Inverts a 3x3 transformation matrix
        Arguments:
            T: 3x3 transformation matrix
        Returns:
            inverse of T
    """
    R = T[0:2,0:2]
    c = T[0:2,2]
    Rinv = np.transpose(R)
    cinv = -Rinv*c
    return np.matrix([[Rinv[0,0],Rinv[0,1],cinv[0]],[Rinv[1,0],Rinv[1,1],cinv[1]],[0,0,1]])

def vec(x,y):
    """ Creates a 2D column vector
        Arguments:
            x: x coordinate
            y: y coordinate
        Returns:
            2D column vector (x,y)
    """
    return np.matrix([[x],[y]])

def unproject(v):
    """ Un-projects a non-homogeneous vector to homogeneous,
        i.e., adds a 1 one on the end
        Arguments:
            v: 2D non-homogeneous vector
        Returns:
            3D homogenous version of v
    """
    return np.matrix([[v[0,0]],[v[1,0]],[1]])

def project(v):
    """ Projects a homogeneous vector to non-homogeneous,
        i.e., divides by third coordinate
        Arguments:
            v: 3D homogeneous vector
        Returns:
            2D non-homogeneous version of v
    """
    return np.matrix([[v[0,0]/v[2,0]],[v[1,0]/v[2,0]]])

def mul(T,v):
    """ Multiply transformation matrix by 2D non-homoegneous vector
        Arguments:
            T: 3D transformation matrix
            v: 2D non-homogeneous vector
        Returns:
            v transformed by T
    """
    return project(T*unproject(v))

def angle(T):
    """ Get angle from transformation matrix
        Arguments:
            T: 2x2 rotation matrix or 3x3 transformation matrix
        Returns:
            theta angle in radians
    """
    return math.atan2(T[1,0],T[0,0])

if __name__ == '__main__':
    # unit tests
    A = transform(np.random.rand(),np.random.rand(),np.random.rand())
    Ainv = invert(A)
    print(A)
    print(Ainv)
    print(A*Ainv)
