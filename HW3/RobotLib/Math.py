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

def meshgrid(width,height,T=None):
    """ Returns grids of x and y coordinates with an optional transformation applied.
        
        By default, this function will return two matrices of shape (height,width)
        containing the x and y coordinates of the center of each cell in the grid.
        
        If a transformation (T) is specified, the x and y coordinates will be
        transformed by T before being returned.

        Arguments:
            width: grid width
            height: grid height
            T: optional transformation to apply to points
        Returns:
            two matrices containing x and y coordinates
    """

    # make x and y ranges
    x_range = np.linspace(0.5,width-0.5,width)
    y_range = np.linspace(0.5,height-0.5,width)

    # get matrices of x and y coordinates
    x,y = np.meshgrid(x_range,y_range, indexing='xy')

    # if no transformation is supplied, return x and y
    if T is None:
        return x, y
    
    # reshape x and y into column vectors
    x = np.reshape(x,(1,width*height))
    y = np.reshape(y,(1,width*height))
    
    # concatenate into a 3xn matrix 
    o = np.ones((1,width*height))
    pt = np.concatenate([x,y,o],axis=0)
    
    # apply transformation matrix
    pt_transformed = T*pt
    
    # extract transformed x and y
    x_transformed = pt_transformed[0,:]
    y_transformed = pt_transformed[1,:]

    # reshape back into grids
    x_transformed = np.reshape(x_transformed,(height,width))
    y_transformed = np.reshape(y_transformed,(height,width))

    # return transformed x and y
    return x_transformed, y_transformed

if __name__ == '__main__':
    # unit tests
    A = transform(np.random.rand(),np.random.rand(),np.random.rand())
    Ainv = invert(A)
    print(A)
    print(Ainv)
    print(A*Ainv)
