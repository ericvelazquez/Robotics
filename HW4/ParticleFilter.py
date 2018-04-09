import pygame
import math
import numpy as np
from RobotLib.Math import *

class ParticleFilter:
    """
    Implements a particle filter for Monte Carlo Localization.
    """
    def __init__(self,num_particles,alpha,robot,omap):
        """ Creates the particle filter algorithm class.
            Arguments:
                num_particles: number of particles
                alpha: list of four coefficients for motion model
                robot: Robot object
                omap: ObstacleMap object
        """
        self.num_particles = num_particles
        self.alpha = alpha
        self.robot = robot
        self.omap = omap

        # make a matrix of particles
        # each row is a particle (cx,cy)
        self.particles = np.zeros((num_particles,2),dtype='float32')
        
        # weight each particle
        self.particle_weights = np.zeros(num_particles,dtype='float32')
        self.particle_weights[:] = 1./num_particles

        # initialize the particles to random positions around the robot's position
        self.particles[:,0] = self.robot.x + np.random.randn(self.num_particles)*10.
        self.particles[:,1] = self.robot.y + np.random.randn(self.num_particles)*10.

    def draw(self,surf):
        """ Draw particles onto a surface
            Args:
                surf: surface to draw on
        """
        pass

    def generate(self,time_delta):
        """ Update particles by sampling from the motion model.
            Arguments:
                time_delta: time elapsed since last update
        """
        pass

    def update(self):
        """ Update particle weights according to the rangefinder reading. """
        pass

    def sample(self):
        """ Re-sample particles according to their weights. """
        # ensure that weights sum up to one
        self.particle_weights /= np.sum(self.particle_weights)

        # calculate the cumulative sum of the particle weights
        sum_particle_weights = np.cumsum(self.particle_weights)
        
        # select N random numbers in the range [0,1)
        sample = np.random.sample(self.num_particles)

        # find which "bin" each random number falls into using binary search
        inds = np.searchsorted(sum_particle_weights,sample)

        # select the new particles according to the bin indices
        new_particles = self.particles[inds,:]
        
        # save the new set of particles
        self.particles = new_particles

