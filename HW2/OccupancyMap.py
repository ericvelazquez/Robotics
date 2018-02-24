import pygame
import math
import numpy as np
from RobotLib.Math import *
from cv2 import imread, imwrite
import cv2
# to install cv2 module: pip install opencv-python

class OccupancyMap:
    """
    Maintains an occupancy map.
    
    The map contains values in the range 0 (free) to 1 (occupied).
    
    The map is stored as a Numpy matrix (self.grid) with shape (height,width).
    To access the value at row i, column j, use self.grid[i,j]
    """
    def __init__(self,path=None,width=None,height=None):
        """ Creates an occupancy map.
            Arguments:
                path: optional path to a grayscale image representing the map
                width: optional width of the map (if path is not provided)
                height: optional height of the map (if path is not provided)
        """
        # create a map with the width of the robot added to the obstacles
        # so we avoid collisions with the object

        if path is not None:
            # read map from image
            self.grid = imread(path,0).astype('float32')/255.
            print('Loaded occupancy map image: width %d, height %d'%(self.grid.shape[1],self.grid.shape[0]))

            num_occupied = np.count_nonzero(self.grid)
            num_free = np.prod(self.grid.shape) - num_occupied
            print('free/occupied: %d / %d'%(num_free,num_occupied))

            self.width = self.grid.shape[1]
            self.height = self.grid.shape[0]

            self.gridDilate = cv2.dilate(self.grid, np.ones((15,15),np.uint8), iterations = 1)

        elif width is not None and height is not None:
            # create empty map
            self.width = width
            self.height = height
            self.grid = np.zeros((height,width),dtype='float32')
        else:
            raise ValueError('must provide path or width + height')
    
    def draw(self,surface):
        """ Draws the occupancy map onto the surface. """
        omap_array = ((1.-self.grid.transpose())*255.).astype('int')
        omap_array = np.tile(np.expand_dims(omap_array,axis=-1),[1,1,3])
        pygame.surfarray.blit_array(surface,omap_array)
    
    def get_first_hit(self,T_sonar_map):
        """ Calculates distance that sonar would report given current sonar-to-map transformation.
            Arguments:
                T_sonar_map: sonar-to-map transformation matrix
            Returns:
                First-hit distance or zero if no hit.
        """
        # max distance set to 250 to fix slow simulation
        max_distance = 250
        #one centimeter increment
        for d in range(1,max_distance):
            point = mul(T_sonar_map,vec(d,0))
            if point[0]<512 and point[0]>=0 and point[1]<512 and \
                point[0]>=0 and self.gridDilate[int(point[0])][int(point[1])] == 1:
                return d
        return 0
    
if __name__ == '__main__':
    # create a map
    grid = np.zeros((512,512))
    
    # obstacle
    grid[300:400,300:400] = 1

    imwrite('map.png',(grid*255.).astype('uint8'))


