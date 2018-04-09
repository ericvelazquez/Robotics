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
        self.particles[:,0] = self.robot.x + np.random.randn(self.num_particles)*100.
        self.particles[:,1] = self.robot.y + np.random.randn(self.num_particles)*100.

    def draw(self,surf):
        """ Draw particles onto a surface
            Args:
                surf: surface to draw on
        """
        for particle in self.particles:
            pygame.draw.circle(surf, (0, 0, 0), particle,2)

    def generate(self,time_delta):
        """ Update particles by sampling from the motion model.
            Arguments:
                time_delta: time elapsed since last update
        """
        for particle in self.particles:
            T_robot_map = transform(particle[0],particle[1],self.robot.theta)
            T_motion = self.sample_motion_model(time_delta)
            T_robot_map = T_robot_map * T_motion

            # extract new position
            particle[0] = T_robot_map[0, 2]
            particle[1] = T_robot_map[1, 2]

    def update(self):
        """ Update particle weights according to the rangefinder reading. """
        T_sonar_robot = self.robot.get_sonar_robot_transform()
        z = self.robot.sonar_distance  # rangefinder reading
        square_sigma = 10
        for i,particle in enumerate(self.particles):
            T_robot_map = transform(particle[0], particle[1], self.robot.theta)
            T_sonar_map = T_robot_map * T_sonar_robot
            d = self.omap.get_first_hit(T_sonar_map) # expected rangefinder reading
            self.particle_weights[i] = np.exp((-(z-d)**2)/square_sigma)

        self.particle_weights /= np.sum(self.particle_weights)


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

    def sample_motion_model(self, time_delta):
        """ Samples from the velocity motion model.
            Arguments:
                time_delta: time elapsed since last update
            Returns:
                T_motion transformation matrix corresponding to the sampled motion
        """
        min_val = 0.00000000001
        # self.alpha = [.1,.1,0.1,0.1]
        lin_std = np.sqrt((self.alpha[0] * (self.robot.lin_vel ** 2)) + (self.alpha[1] * (self.robot.ang_vel ** 2)))+min_val
        ang_std = np.sqrt((self.alpha[2] * (self.robot.lin_vel ** 2)) + (self.alpha[3] * (self.robot.ang_vel ** 2)))+min_val
        lin_noise = np.random.normal(0,lin_std,1)
        ang_noise = np.random.normal(0, ang_std, 1)
        lin_vel_noise = self.robot.lin_vel + lin_noise
        ang_vel_noise = self.robot.ang_vel + ang_noise

        if ang_vel_noise == 0:
            # pure linear motion
            T_motion = transform(lin_vel_noise * time_delta, 0, 0)
        elif lin_vel_noise == 0:
            # pure rotational motion
            T_motion = transform(0,0,ang_vel_noise * time_delta)
        else:
            # move origin to ICC
            R = lin_vel_noise / ang_vel_noise
            T_motion = transform(0,R,0)
            # perform rotation
            T_motion = T_motion * transform(0,0,ang_vel_noise * time_delta)
            # move origin back
            T_motion = T_motion * transform(0,-R,0)

        return T_motion
