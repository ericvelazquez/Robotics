import pygame
import math
import numpy as np
from RobotLib.Math import *
from cv2 import imread, imwrite


# to install cv2 module: pip install opencv-python

class OccupancyGrid:
    """

    """

    def __init__(self):
        self.log_odds = np.zeros((128,128))
        self.cell_prob_occupancy()

    def cell_prob_occupancy(self):
        """
        Computes the probability of occupancy of each cell
        :return: porbability of occupancy
        """
        self.probability_occupancy = 1-1/(1 + np.exp(self.log_odds))
        print self.log_odds

    def draw(self, surface):
        """ Draws the porbability of occupancy onto the surface. """
        # transpose grid and convert to 0-255 range
        ogrid_array = ((1. - self.probability_occupancy.transpose()) * 255.).astype('int')
        # replicate across RGB channels
        ogrid_array = np.tile(np.expand_dims(ogrid_array, axis=-1), [1, 1, 3])
        # draw grid on the surface
        pygame.surfarray.blit_array(surface, ogrid_array)

if __name__ == '__main__':
    o = OccupancyGrid()
