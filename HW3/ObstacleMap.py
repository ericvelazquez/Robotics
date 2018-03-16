import pygame
import math
import numpy as np
from RobotLib.Math import *
from cv2 import imread, imwrite
# to install cv2 module: pip install opencv-python

class ObstacleMap:
    """
    Maintains an obstacle map.
    
    The map contains binary values: 0 (free) or 1 (occupied).
    
    The map is stored as a matrix with shape (height,width).
    
    The map can be used to simulate rangefinder readings.
    """
    def __init__(self,path,max_dist=80):
        """ Creates an obstacle map.
            Arguments:
                path: path to a grayscale image representing the map
                max_dist: maximum rangefinder reading (cm)
        """
        self.max_dist = max_dist

        # read map from image
        self.grid = imread(path,0).astype('float32')/255.
        
        self.height = self.grid.shape[0]
        self.width = self.grid.shape[1]
        
    def draw(self,surface):
        """ Draws the obstacle map onto the surface. """
        # transpose grid and convert to 0-255 range
        omap_array = ((1.-self.grid.transpose())*255.).astype('int')
        # replicate across RGB channels
        omap_array = np.tile(np.expand_dims(omap_array,axis=-1),[1,1,3])
        # draw grid on the surface
        pygame.surfarray.blit_array(surface,omap_array)
    
    def get_first_hit(self,T_sonar_map):
        """ Calculates distance that sonar would report given current pose.
            Arguments:
                T_sonar_map: sonar-to-map transformation matrix
            Returns:
                First-hit distance or zero if no hit.
        """
        # iterate over possible range of distances
        for i in range(self.max_dist):
            # get point in sonar reference frame
            pt_sonar = vec(i,0)

            # transform to map reference frame
            pt_map = mul(T_sonar_map,pt_sonar)

            # get integer location in map
            r = int(pt_map[1])
            c = int(pt_map[0])

            # test for location outside map
            if r < 0 or r >= self.grid.shape[0]:
                continue
            if c < 0 or c >= self.grid.shape[1]:
                continue

            # test if cell is occupied
            if self.grid[r,c] > 0:
                # return rangefinder measurement
                return i

        # return 0 for no hit
        return 0.

if __name__ == '__main__':
    # run this script to make an example map

    # create a map
    grid = np.zeros((128,128))
    
    # border
    grid[:,0] = 1
    grid[:,127] = 1
    grid[0,:] = 1
    grid[127,:] = 1
    
    # obstacle
    grid[75:100,75:100] = 1
    
    imwrite('map.png',(grid*255.).astype('uint8'))

