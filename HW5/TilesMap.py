import pygame
import math
import numpy as np
from RobotLib.Math import *

W = 128
H = 128


class TilesMap:
    """
    Maintains a tiles map consisting of a random grid of black and white tiles.
    
    The map is stored as a list of point pairs.
    
    The map can be used to simulate line sensor readings.
    """

    def __init__(self, path):
        """ Creates an tiles map.
        """

        generate_map()

        self.rectangles = np.loadtxt(path, delimiter=',')
        self.tilegrid = np.zeros((W, H))
        self.width = W
        self.height = H

        for rect in self.rectangles:
            x = int(rect[0])
            y = int(rect[1])
            width = int(rect[2])
            height = int(rect[3])
            self.tilegrid[x:x + width, y:y + height] = 1

    def draw(self, surface):
        """ Draws the obstacle map onto the surface. """
        for rect in self.rectangles:
            pygame.draw.rect(surface, (255, 0, 0), rect)

    def get_tile(self, cx, cy):
        """ Calculates if robot is within a tile or not.
            Arguments:
                robot_cx: robot x position
                robot_cy: robot y position
            Returns:
                1 if it is inside a tile and 0 if it is not
        """
        if 0 < cx < self.width and 0 < cy < self.height and \
                self.tilegrid[int(cx), int(cy)] == 0:
            return 900
        return 200


def generate_map():
    rectangles = np.ndarray((0, 4))
    max_tiles = np.random.randint(20, 40, size=1)
    for i in range(max_tiles):
        # obstacle
        x = np.random.randint(1, W - W / 4, size=1)
        y = np.random.randint(1, H - H / 4, size=1)
        width = np.random.randint(1, H / 4, size=1)
        height = np.random.randint(1, W / 4, size=1)
        rectangles = np.append(rectangles, [[x[0], y[0], width[0], height[0]]], axis=0)

    np.savetxt('map-tiles.txt', rectangles, delimiter=',')


if __name__ == '__main__':
    # run this script to make an example map

    generate_map()
