import pygame
import math
import numpy as np
from RobotLib.Math import *

class ObstacleMap:
    """
    Maintains an obstacle map consisting of lines.
    
    The map is stored as a list of point pairs (start and end points of lines).
    
    The map can be read from a text file.
    Each line of the file should have format x1,y1,x2,y2
    where (x1,y1) is the start point of the line and (x2,y2) is the end point of the line.
    
    The map can be used to simulate rangefinder readings.
    """
    def __init__(self,path,max_dist=80,noise_range=0):
        """ Creates an obstacle map.
            Arguments:
                path: path to a text file containing the lines
                max_dist: maximum rangefinder reading (cm)
                noise_range: maximum noise to add to rangefinder reading (cm)
        """
        self.max_dist = max_dist
        self.noise_range = noise_range

        # read map from file
        self.lines = np.loadtxt(path,delimiter=',')
        
        # get width and height (hack)
        self.width = int(max(np.max(self.lines[:,0]),np.max(self.lines[:,2]))+1)
        self.height = int(max(np.max(self.lines[:,1]),np.max(self.lines[:,3]))+1)
        
        # get line starting points
        self.lx = self.lines[:,0]
        self.ly = self.lines[:,1]

        # get line rays
        self.rx = self.lines[:,2] - self.lines[:,0]
        self.ry = self.lines[:,3] - self.lines[:,1]
        
        # get length of rays
        self.lengths = np.hypot(self.rx,self.ry)

        # normalize rays
        self.rx /= self.lengths
        self.ry /= self.lengths

    def draw(self,surface):
        """ Draws the obstacle map onto the surface. """
        for line in self.lines:
            pygame.draw.line(surface,(0,0,0),line[0:2],line[2:4])
    
    def get_first_hit(self,T_sonar_map,add_noise=True):
        """ Calculates distance that sonar would report given current pose.
            Arguments:
                T_sonar_map: sonar-to-map transformation matrix
            Returns:
                First-hit distance or zero if no hit.
        """
        # get sonar center point
        cx = T_sonar_map[0,2]
        cy = T_sonar_map[1,2]

        # get sonar direction
        sx = T_sonar_map[0,0]
        sy = T_sonar_map[1,0]

        # get denominator
        denom = self.rx*sy - self.ry*sx

        # get sonar ray lengths
        sonar_dists = (cx*self.ry - cy*self.rx - self.lx*self.ry + self.ly*self.rx)/denom

        # get line ray lengths
        line_dists = (cx*sy - cy*sx - self.lx*sy + self.ly*sx)/denom
        
        # test whether line intersections are valid
        valid = np.logical_and(line_dists > 0,line_dists < self.lengths)
        
        # set invalid distances to infinity
        sonar_dists[np.logical_not(valid)] = np.inf
        sonar_dists[sonar_dists<0] = np.inf

        # find minimum hit distance
        index = np.argmin(sonar_dists)

        # test if first hit is within range
        if sonar_dists[index] < self.max_dist:
            if add_noise and self.noise_range > 0:
                # sample noise
                noise = np.random.randint(-self.noise_range,self.noise_range+1)

                # add noise to true value
                measurement = sonar_dists[index] + noise

                # clip to correct range
                measurement = np.clip(measurement,1,np.inf)
                
                return measurement
            else:
                return sonar_dists[index]
        
        # return 0 for no hit
        return 0.

def add_box(lines,x1,y1,x2,y2):
    """ Adds a box to the list of lines. """
    lines = np.append(lines,[[x1,y1,x2,y1]],axis=0)
    lines = np.append(lines,[[x2,y1,x2,y2]],axis=0)
    lines = np.append(lines,[[x2,y2,x1,y2]],axis=0)
    lines = np.append(lines,[[x1,y2,x1,y1]],axis=0)
    return lines

if __name__ == '__main__':
    # run this script to make an example map

    width = 64
    height = 64
    lines = np.ndarray((0,4))
    
    # border
    lines = add_box(lines,0,0,width-1,height-1)
    
    # obstacle
    lines = add_box(lines,40,40,50,50)
    
    np.savetxt('map.txt',lines,delimiter=',')

