import pygame
import math
import numpy as np
from RobotLib.Math import *
import time

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

        # initialize to random poses
        self.particles[:,0] = self.robot.x + np.random.randn(self.num_particles)*10.
        self.particles[:,1] = self.robot.y + np.random.randn(self.num_particles)*10.
    
    def sample_motion_model(self,time_delta):
        """ Sample from the motion model given the requested angular and linear velocities. """
        v_sq = self.robot.lin_vel*self.robot.lin_vel
        w_sq = self.robot.ang_vel*self.robot.ang_vel
        v_hat = np.random.normal(loc=self.robot.lin_vel,scale=math.sqrt(self.alpha[0]*v_sq+self.alpha[1]*w_sq))
        w_hat = np.random.normal(loc=self.robot.ang_vel,scale=math.sqrt(self.alpha[2]*v_sq+self.alpha[3]*w_sq))
        if w_hat == 0:
            # pure linear motion
            T_motion = transform(v_hat * time_delta, 0, 0)
        else:
            # move origin to ICC
            R = v_hat / w_hat
            T_motion = transform(0,R,0)
            # perform rotation
            T_motion = T_motion * transform(0,0,w_hat * time_delta)
            # move origin back
            T_motion = T_motion * transform(0,-R,0)
        return T_motion
    
    def generate(self,time_delta):
        """ Update particles. """
        for i in range(len(self.particles)):
            T_robot_map = transform(self.particles[i,0],self.particles[i,1],self.robot.theta)
            T_motion = self.sample_motion_model(time_delta)
            T_robot_map = T_robot_map * T_motion
            self.particles[i,0] = T_robot_map[0,2]
            self.particles[i,1] = T_robot_map[1,2]
    
    def sensor_model(self,dist,true_dist):
        """ Calculate sensor model. """
        return np.exp(-np.square(dist-true_dist)/10)
    
    def update(self):
        """ Update particle weights. """
        self.particle_weights[:] = 1e-5
        for i in range(len(self.particles)):
            T_robot_map = transform(self.particles[i,0],self.particles[i,1],self.robot.theta)
            T_sonar_map = T_robot_map * self.robot.get_sonar_robot_transform()
            true_dist = self.omap.get_first_hit(T_sonar_map,add_noise=False)
            if true_dist > 0:
                self.particle_weights[i] = self.sensor_model(self.robot.sonar_distance,true_dist)
        self.particle_weights /= np.sum(self.particle_weights)
    
    def sample(self):
        """ Sample particles according to their likelihood. """
        sum_particle_weights = np.cumsum(self.particle_weights)
        sample = np.random.sample(self.num_particles)
        inds = np.searchsorted(sum_particle_weights,sample)
        new_particles = self.particles[inds,:]
        self.particles = new_particles
    
    def draw(self,surface):
        """ Draws the particles onto the surface. """
        for i in range(self.num_particles):
            pygame.draw.circle(surface,(0,0,0),(self.particles[i,0],self.particles[i,1]),2)
    
