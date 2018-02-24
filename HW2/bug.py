import pygame
import sys
import time
import math
import argparse
import numpy as np
from RobotLib.FrontEnd import *
from RobotLib.IO import *
from Robot import Robot
from OccupancyMap import OccupancyMap
from RobotLib.Math import *


#PID constants
K_linear = 1
K_angular = 2
D_threshold = 5
SONAR_THRESHOLD = 70
class MyFrontEnd(FrontEnd):
    def __init__(self,sparki,omap_path):
        self.omap = OccupancyMap(omap_path)
        FrontEnd.__init__(self,self.omap.width,self.omap.height)
        self.sparki = sparki
        self.robot = Robot()

        # center robot
        self.robot.x = self.omap.width*0.5
        self.robot.y = self.omap.height*0.5
        
        # zero out robot velocities
        self.robot.lin_vel = 0
        self.robot.ang_vel = 0

        # Goal location in the robot's reference frame
        self.goal = vec(self.robot.x, self.robot.y)

        # Angle to goal
        self.angle = 0

        # Distance to goal
        self.distance = 0

        
    def mouseup(self,x,y,button):
        """
        Set the goal on a mouse click. If the click is inside an obstacle it wont be setted
        :param x: mouse x coordinate
        :param y: mouse y coordinate
        :param button: mouse button
        """
        if self.omap.gridDilate[x][y] == 0:
            self.goal = vec(x,y)

    def draw(self,surface):
        # draw occupancy map
        self.omap.draw(surface)

        # draw robot
        self.robot.draw(surface)

    def update(self,time_delta):


        # get sonar distance
        if self.sparki.port == '':
            # simulate rangefinder
            T_sonar_map = self.robot.get_robot_map_transform() * self.robot.get_sonar_robot_transform()
            self.robot.sonar_distance = self.omap.get_first_hit(T_sonar_map)
        else:
            # read rangefinder
            self.robot.sonar_distance = self.sparki.dist

        T_map_robot = self.robot.get_map_robot_transform()

        # get angle and distance to the goal
        goal_rf = mul(T_map_robot, self.goal)
        self.angle = math.atan2(goal_rf[1], goal_rf[0])
        self.distance = math.sqrt((goal_rf[0] - 0) ** 2 + (goal_rf[1] - 0) ** 2)

        # check rangefinder
        if self.robot.sonar_distance > 0 and self.robot.sonar_distance < SONAR_THRESHOLD:
            self.robot.ang_vel = K_angular * 25 * 3.1415 / 180
            self.robot.lin_vel = 0
        else:
            # Calculate linear and angular velocity
            self.robot.lin_vel = K_linear * self.distance
            self.robot.ang_vel = K_angular * self.angle

            # Use threshold
            if self.distance < D_threshold:
                self.robot.lin_vel = 0
                self.robot.ang_vel = 0

    
        # calculate motor settings
        left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()
        
        # send command
        self.sparki.send_command(left_speed,left_dir,right_speed,right_dir)
        
        # update robot position
        self.robot.update(time_delta)

def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='template')
    parser.add_argument('--omap', type=str, default='map.png', help='path to occupancy map image file')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()

    with SparkiSerial(port=args.port) as sparki:
        # make frontend
        frontend = MyFrontEnd(sparki,args.omap)
    
        # run frontend
        frontend.run()

if __name__ == '__main__':
    main()
