import pygame
import sys
import time
import math
import argparse
from RobotLib.FrontEnd import *
from RobotLib.IO import *
from Robot import Robot
import numpy as np

class MyFrontEnd(FrontEnd):
    def __init__(self,width,height,sparki):
        FrontEnd.__init__(self,width,height)
        self.sparki = sparki
        self.robot = Robot()
        
        # center robot
        self.robot.x = width*0.5
        self.robot.y = height*0.5

        # message callback
        self.sparki.message_callback = self.message_received
        
        # last timestamp received
        self.last_timestamp = 0

    def keydown(self,key):
        # update velocities based on key pressed
        if key == pygame.K_UP: # set positive linear velocity
            self.robot.requested_lin_vel = 2.0
        elif key == pygame.K_DOWN: # set negative linear velocity
            self.robot.requested_lin_vel = -2.0
        elif key == pygame.K_LEFT: # set positive angular velocity
            self.robot.requested_ang_vel = 10.*math.pi/180.
        elif key == pygame.K_RIGHT: # set negative angular velocity
            self.robot.requested_ang_vel = -10.*math.pi/180.
        elif key == pygame.K_k: # set positive servo angle
            self.robot.requested_sonar_angle = 45.*math.pi/180.
        elif key == pygame.K_l: # set negative servo angle
            self.robot.requested_sonar_angle = -45.*math.pi/180.
    
    def keyup(self,key):
        # update velocities based on key released
        if key == pygame.K_UP or key == pygame.K_DOWN: # set zero linear velocity
            self.robot.requested_lin_vel = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT: # set zero angular velocity
            self.robot.requested_ang_vel = 0
        elif key == pygame.K_k or key == pygame.K_l: # set zero servo angle
            self.robot.requested_sonar_angle = 0
        
    def draw(self,surface):
        # draw robot
        self.robot.draw(surface)
    
    def update(self):
        """ Sends command to robot, if not in simulator mode. """
        if self.sparki.port is not '':
            # calculate servo setting
            servo = int(self.robot.requested_sonar_angle*180./math.pi)

            # calculate motor settings
            left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()

            # send command
            self.sparki.send_command(left_speed,left_dir,right_speed,right_dir,servo)

    def message_received(self,message):
        """ Callback for when a message is received from the robot or the simulator.
            Arguments:
                message: dictionary with status values
        """
        if self.last_timestamp == 0:
            self.last_timestamp = message['timestamp']
            return
        
        # calculate time_delta
        time_delta = (message['timestamp'] - self.last_timestamp)/1000.
        self.last_timestamp = message['timestamp']
        
        # update linear and angular velocities
        if 'left_motor_speed' in message.keys():
            # calculate linear and angular velocities
            left_speed = message['left_motor_speed']
            left_dir = message['left_motor_dir']
            right_speed = message['right_motor_speed']
            right_dir = message['right_motor_dir']
            
            self.robot.compute_velocities(left_speed,left_dir,right_speed,right_dir)
        else:
            # copy requested to actual
            self.robot.lin_vel = self.robot.requested_lin_vel
            self.robot.ang_vel = self.robot.requested_ang_vel
        
        # update robot position
        self.robot.update(time_delta)
        
        # update servo ping distance
        if 'rangefinder' in message.keys():
            # convert to cm
            self.robot.sonar_distance = message['rangefinder']

def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='Differential drive controller')
    parser.add_argument('--width', type=int, default=256, help='map width')
    parser.add_argument('--height', type=int, default=256, help='map height')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()
    
    with SparkiSerial(port=args.port) as sparki:
        # make frontend
        frontend = MyFrontEnd(args.width,args.height,sparki)
    
        # run frontend
        frontend.run()

if __name__ == '__main__':
    main()
