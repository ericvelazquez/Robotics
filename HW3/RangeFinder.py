import pygame
import math
import numpy as np
from RobotLib.Math import *
from cv2 import imread, imwrite
from OccupancyGrid import OccupancyGrid


# to install cv2 module: pip install opencv-python
# Default values:
l_occ = 5
l_free = -5
l_0 = 0
beta = 20/180*np.pi
alpha = 20/180*np.pi

class RangeFinder:
    """

    """

    def __init__(self, ogrid):
        self.cone_width = 0 #radians
        self.obstacle_width = 0 #centimeters
        self.ogrid = ogrid

    def inverse_sensor_model(self, rfvalue):
        """
        Calculates the log odds for a partical cell m for particular rangefinder value
        :param mi: particular cell
        :param rfvalue: Rangefinder value
        :return:
        """
        # Convert to polar coordinates
        x_transformed, y_transformed = meshgrid(128,128)
        # To get the centers of each cell
        x_transformed += 0.5
        y_transformed += 0.5
        r = np.sqrt(np.square(x_transformed) + np.square(y_transformed))
        theta = np.arctan2(y_transformed,x_transformed)

        # Compute the log odds for each cell in the grid
        a = np.bitwise_and(np.bitwise_or(-beta/2 <= theta, theta <= beta/2), np.bitwise_or(rfvalue - alpha/2 <= r, r <= rfvalue + alpha/2))
        self.ogrid.log_odds[a] += l_occ

        b = np.bitwise_and(np.bitwise_or(-beta/2 <= theta, theta <= beta/2), np.bitwise_or(0 <= r, r <= rfvalue - alpha/2))
        self.ogrid.log_odds[b] += l_free

        c = np.bitwise_not(np.bitwise_or(a,b))
        self.ogrid.log_odds[c] += l_0

if __name__ == '__main__':
    pass