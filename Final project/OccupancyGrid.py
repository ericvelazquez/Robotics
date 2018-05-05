import pygame
import math
import numpy as np
from RobotLib.Math import *
from cv2 import imread, imwrite
# to install cv2 module: pip install opencv-python

class OccupancyGrid:
    """
    Maintains an occupancy grid.
    
    The grid contains the log-odds of occupancy of each cell. 

    The log-odds of a cell being occupied is log( p(occupied) / p(free) ).
    
    The grid is stored as a matrix with shape (height,width).
    """
    def __init__(self,width,height):
        """ Creates an occupancy grid.
            Arguments:
                width: width of the grid
                height: height of the grid
        """
        self.width = width
        self.height = height

        # make empty occupancy grid
        self.grid = np.zeros((height,width),dtype='float32')
    
    def get_probabilities(self):
        """ Returns a grid containing the probability of occupancy at each cell. """
        return 1. - 1./(1+np.exp(self.grid))
    
    def draw(self,surface):
        """ Draws the occupancy grid onto the surface. """
        probs = self.get_probabilities()
        omap_array = ((1.-probs.transpose())*255.).astype('int')
        omap_array = np.tile(np.expand_dims(omap_array,axis=-1),[1,1,3])
        pygame.surfarray.blit_array(surface,omap_array)
    
