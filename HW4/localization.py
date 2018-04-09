import pygame
import sys
import time
import math
import argparse
from RobotLib.FrontEnd import *
from RobotLib.IO import *
from RobotLib.Math import *
from Robot import Robot
from ObstacleMap import ObstacleMap
from Rangefinder import Rangefinder
from ParticleFilter import ParticleFilter
import numpy as np

class MyFrontEnd(FrontEnd):
    def __init__(self,omap_path,sparki):
        self.omap = ObstacleMap(omap_path,noise_range=0)
        self.rangefinder = Rangefinder(cone_width=deg2rad(15.),obstacle_width=1.)
    
        FrontEnd.__init__(self,self.omap.width,self.omap.height)

        self.sparki = sparki
        self.robot = Robot()

        # center robot
        self.robot.x = self.omap.width*0.5
        self.robot.y = self.omap.height*0.5

        # create particle filter
        alpha=[0.5,0.5,0.5,0.5,0.5,0.5]
        self.particle_filter = ParticleFilter(num_particles=200,alpha=alpha,robot=self.robot,omap=self.omap)
        
        self.sonar_step = deg2rad(1.)

    def keydown(self,key):
        # update velocities based on key pressed
        if key == pygame.K_UP: # set positive linear velocity
            self.robot.lin_vel = 20.0
        elif key == pygame.K_DOWN: # set negative linear velocity
            self.robot.lin_vel = -20.0
        elif key == pygame.K_LEFT: # set positive angular velocity
            self.robot.ang_vel = 100.*math.pi/180.
        elif key == pygame.K_RIGHT: # set negative angular velocity
            self.robot.ang_vel = -100.*math.pi/180.
        elif key == pygame.K_k: # set positive servo angle
            self.robot.sonar_angle = 45.*math.pi/180.
        elif key == pygame.K_l: # set negative servo angle
            self.robot.sonar_angle = -45.*math.pi/180.
    
    def keyup(self,key):
        # update velocities based on key released
        if key == pygame.K_UP or key == pygame.K_DOWN: # set zero linear velocity
            self.robot.lin_vel = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT: # set zero angular velocity
            self.robot.ang_vel = 0
        elif key == pygame.K_k or key == pygame.K_l: # set zero servo angle
            self.robot.sonar_angle = 0
        
    def draw(self,surface):
        # draw obstacle map
        self.omap.draw(surface)

        # draw robot
        self.robot.draw(surface)
        
        # draw particles
        self.particle_filter.draw(surface)
    
    def update(self,time_delta):
        # Get map/sonar transformation matrices
        T_map_sonar = self.robot.get_robot_sonar_transform()*self.robot.get_map_robot_transform()
        T_sonar_map = self.robot.get_robot_map_transform()*self.robot.get_sonar_robot_transform()

        if self.sparki.port == '':
            # calculate sonar distance from map
            self.robot.sonar_distance = self.omap.get_first_hit(T_sonar_map)
        else:
            # get current rangefinder reading
            self.robot.sonar_distance = self.sparki.dist
        
        # update particles
        self.particle_filter.generate(time_delta)
        self.particle_filter.update()
        self.particle_filter.sample()
        
        # calculate servo setting
        servo = int(self.robot.sonar_angle*180./math.pi)

        # calculate motor settings
        left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()

        # send command
        self.sparki.send_command(left_speed,left_dir,right_speed,right_dir,servo)
        
        # update robot position
        self.robot.update(time_delta)

def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='Particle filter localization demo')
    parser.add_argument('--omap', type=str, default='map.txt', help='path to obstacle map txt file')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()
    
    with SparkiSerial(port=args.port) as sparki:
        # make frontend
        frontend = MyFrontEnd(args.omap,sparki)
    
        # run frontend
        frontend.run()

if __name__ == '__main__':
    main()
