import pygame
import sys
import time
import math
import argparse
import threading
from RobotLib.FrontEnd import *
from RobotLib.IO import *
import numpy as np

class MyFrontEnd(FrontEnd):
    """ Custom sub-class of FrontEnd
        You can write custom sub-routines here to respond to UI events and calculate updates
    """

    def __init__(self,width,height,sparki):
        FrontEnd.__init__(self,width,height)
        self.sparki = sparki
        
        # message callback
        self.sparki.message_callback = self.message_received
        
        # last timestamp received
        self.last_timestamp = 0
        
    def mouseup(self,x,y,button):
        # x,y is position of mouse click
        print('mouse clicked at %d, %d'%(x,y))

    def keydown(self,key):
        # see https://www.pygame.org/docs/ref/key.html for pygame key names, such as pygame.K_UP
        print('key pressed')

    def keyup(self,key):
        # see https://www.pygame.org/docs/ref/key.html for pygame key names, such as pygame.K_UP
        print('key released')
        
    def draw(self,surface):
        pass

    def update(self):
        pass

    def message_received(self,message):
        """ Callback for when a message is received from the robot or the simulator.
            Arguments:
                message: dictionary with status values
        """
        # check if there is no previous timestamp
        if self.last_timestamp == 0:
            self.last_timestamp = message['timestamp']
            return

        # calculate time_delta based on the current and previous timestamps
        time_delta = (message['timestamp'] - self.last_timestamp)/1000.
        self.last_timestamp = message['timestamp']

def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='Template')
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
