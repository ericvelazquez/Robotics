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
import numpy as np

class MyFrontEnd(FrontEnd):
    def __init__(self,omap_path,sparki):
        self.omap = ObstacleMap(omap_path)
    
        FrontEnd.__init__(self,self.omap.width,self.omap.height)

        self.sparki = sparki
        self.robot = Robot()
        
        # center robot
        self.robot.x = self.omap.width*0.5
        self.robot.y = self.omap.height*0.5

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
    
    def update(self,time_delta):
        T_sonar_map = self.robot.get_robot_map_transform()*self.robot.get_sonar_robot_transform()
        if self.sparki.port == '':
            # calculate sonar distance from map
            self.robot.sonar_distance = self.omap.get_first_hit(T_sonar_map)
        else:
            # get current rangefinder reading
            self.robot.sonar_distance = self.sparki.dist
        
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
    parser = argparse.ArgumentParser(description='Bayesian mapping demo')
    parser.add_argument('--omap', type=str, default='map.png', help='path to obstacle map png file')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()
    
    with SparkiSerial(port=args.port) as sparki:
        # make frontend
        frontend = MyFrontEnd(args.omap,sparki)
    
        # run frontend
        frontend.run()

if __name__ == '__main__':
    main()
