import pygame
import sys
import time
import math
import argparse
from RobotLib.FrontEnd import *
from RobotLib.IO import *
import numpy as np

R = 10
L = 10
class Robot():
    def __init__(self, sparki):
        self.sparki = sparki  #SparkiSerial class from RobotLib.IO
        self.right_velocity = 0
        self.left_velocity = 0
        self.left_dir = 1
        self.right_dir = 1
        self.velocity = 0
        self.w = 0

    def stop_moving(self):
        self.velocity = 0

    def move_forward(self):
        self.velocity = 70
        self.left_dir = 1
        self.right_dir = 1

    def move_backward(self):
        self.velocity = 70
        self.left_dir = 0
        self.right_dir = 0

    def turn_left(self):
        self.w = self.velocity/R

    def turn_right(self):
        self.w = -self.velocity/R

    def stop_turn(self):
        self.w = 0

    def send_command(self):
        self.right_velocity = self.velocity + self.w * L / 2
        self.left_velocity = self.velocity - self.w * L / 2
        self.sparki.send_command(left_speed=self.left_velocity, left_dir=self.left_dir ,
                                 right_speed=self.right_velocity, right_dir=self.right_dir,
                                 servo_angle=0,
                                 gripper_status=0)



class MyFrontEnd(FrontEnd):
    """ Custom sub-class of FrontEnd
        You can write custom sub-routines here to respond to UI events and calculate updates
    """

    def __init__(self,width,height,robot):
        FrontEnd.__init__(self,width,height)
        self.robot = robot # Robot class

    def mouseup(self,x,y,button):
        # x,y is position of mouse click
        print('mouse clicked at %d, %d'%(x,y))

    def keydown(self,key):
        # see https://www.pygame.org/docs/ref/key.html for pygame key names, such as pygame.K_UP
        if key == pygame.K_UP:
            self.robot.move_forward()
        elif key == pygame.K_DOWN:
            self.robot.move_backward()
        elif key == pygame.K_RIGHT:
            self.robot.turn_right()
        elif key == pygame.K_LEFT:
            self.robot.turn_left()
        print('key pressed')

    def keyup(self,key):
        # see https://www.pygame.org/docs/ref/key.html for pygame key names, such as pygame.K_UP
        if key == pygame.K_UP:
            self.robot.stop_moving()
        elif key == pygame.K_DOWN:
            self.robot.stop_moving()
        elif key == pygame.K_RIGHT:
            self.robot.stop_turn()
        elif key == pygame.K_LEFT:
            self.robot.stop_turn()
        print('key released')
        
    def draw(self,surface):
        # draw robot here
        #
        # draw a rectangle for the robot
        # draw a red line from the sonar to the object it is pinging
        #
        # use pygame.draw.line(surface,color,point1,point2) to draw a line
        # for example, pygame.draw.line(surface,(0,0,0),(0,0),(10,10))
        # draws a black line from (0,0) to (10,0)
        pygame.draw.line(surface,(0,0,0),(0,0),(10,10))

    def update(self,time_delta):
        # this function is called approximately every 50 milliseconds
        # time_delta is the time in seconds since the last update() call
        # 
        # you can send an update command to sparki here
        # use self.sparki.send_command(left_speed,left_dir,right_speed,right_dir,servo,gripper_status)
        # see docs in RobotLib/IO.py for more information
        #
        # if you send a message more than once per millisecond, the message will be dropped
        # so, only call send_command() once per update()
        #
        # you can also calculate dead reckoning (forward kinematics) and other things like PID control here
        self.robot.send_command()

def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='Template')
    parser.add_argument('--width', type=int, default=256, help='map width')
    parser.add_argument('--height', type=int, default=256, help='map height')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()
    
    with SparkiSerial(port=args.port) as sparki:
        # make frontend
        robot = Robot(sparki)
        frontend = MyFrontEnd(args.width,args.height,robot)
    
        # run frontend
        frontend.run()

if __name__ == '__main__':
    main()
