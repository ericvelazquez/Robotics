import math
import numpy as np
from RobotLib.Math import *

class Rangefinder:
    """
    Implements a rangefinder sensor model.
    """
    def __init__(self,cone_width,obstacle_width):
        """ Creates a rangefinder model.
            Arguments:
                cone_width: width of rangefinder measurement cone (rad)
                obstacle_width: width of obstacle border (cm)
        """
        self.cone_width = cone_width
        self.obstacle_width = obstacle_width
        
        # epsilon for small probability value
        eps = 1e-5
        
        # log-odds value to use for free space
        self.L_free = np.log(eps) - np.log(1.-eps)

        # log-odds value to use for occupied space
        self.L_occ = -self.L_free
    
    def get_L_meas(self,x,y,dist):
        """ Gets the log-odds of occupancy given a measurement.
            Arguments:
                x: x coordinates (in sensor frame)
                y: y coordinates (in sensor frame)
                dist: distance measured (cm)
        """

        # get radius in polar coordinates
        r = np.sqrt(np.square(x) + np.square(y))

        # get angle in polar coordinates
        theta = np.arctan2(y,x)

        # initialize log-odds to zero (prior)
        L_meas = np.zeros(r.shape,dtype='float32')

        # get area inside cone
        in_cone = np.bitwise_and(
                x > 0,
                np.abs(theta) < self.cone_width*0.5 )

        # set obstacle boundary to L_occ
        on_boundary = np.bitwise_and(
                np.abs(r-dist) < self.obstacle_width*0.5,
                in_cone )
        L_meas[on_boundary] = self.L_occ

        # set free space to L_free
        in_free_space = np.bitwise_and(
                r < (dist - self.obstacle_width*0.5),
                in_cone )
        L_meas[in_free_space] = self.L_free
        return L_meas

    def integrate_measurement_parallel(self,ogrid,T_map_sonar,dist):
        """ Integrates a measurement from the sonar rangefinder.
            Arguments:
                ogrid: occupancy grid
                T_map_sonar: map-to-sonar transformation matrix
                dist: distance measured (cm)
        """
        # get x and y coordinates for all grid cells, transformed to sonar frame
        x_sonar, y_sonar = meshgrid(ogrid.width, ogrid.height, T_map_sonar)
        
        # add measurement log-odds to current log-odds (i.e., calculate posterior)
        ogrid.grid += self.get_L_meas(x_sonar,y_sonar,dist)

