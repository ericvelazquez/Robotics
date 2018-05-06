import pygame
import sys
import time
import math
import argparse
from RobotLib.FrontEnd import *
from RobotLib.IO import *
from RobotLib.Math import *
from Robot import Robot
from TilesMap import TilesMap
from ParticleFilter import ParticleFilter
import numpy as np


class MyFrontEnd(FrontEnd):
    def __init__(self, tmap_path, sparki):
        self.tmap = TilesMap(tmap_path)

        FrontEnd.__init__(self, self.tmap.width, self.tmap.height)

        self.sparki = sparki
        self.robot = Robot()

        # center robot
        self.robot.x = self.tmap.width * 0.5
        self.robot.y = self.tmap.height * 0.5

        # create particle filter
        alpha = [0.5, 0.5, 0.5, 0.5]
        self.particle_filter = ParticleFilter(num_particles=200, alpha=alpha, robot=self.robot, tmap=self.tmap)

        # set message callback
        self.sparki.message_callback = self.message_received

        # last timestamp received
        self.last_timestamp = 0

    def keydown(self, key):
        # update velocities based on key pressed
        if key == pygame.K_UP:  # set positive linear velocity
            self.robot.requested_lin_vel = 20.0
        elif key == pygame.K_DOWN:  # set negative linear velocity
            self.robot.requested_lin_vel = -20.0
        elif key == pygame.K_LEFT:  # set positive angular velocity
            self.robot.requested_ang_vel = 100. * math.pi / 180.
        elif key == pygame.K_RIGHT:  # set negative angular velocity
            self.robot.requested_ang_vel = -100. * math.pi / 180.
        elif key == pygame.K_k:  # set positive servo angle
            self.robot.requested_sonar_angle = 45. * math.pi / 180.
        elif key == pygame.K_l:  # set negative servo angle
            self.robot.requested_sonar_angle = -45. * math.pi / 180.

    def keyup(self, key):
        # update velocities based on key released
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero linear velocity
            self.robot.requested_lin_vel = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero angular velocity
            self.robot.requested_ang_vel = 0
        elif key == pygame.K_k or key == pygame.K_l:  # set zero servo angle
            self.robot.requested_sonar_angle = 0

    def draw(self, surface):
        # draw obstacle map
        self.tmap.draw(surface)

        # draw robot
        self.robot.draw(surface)

        # draw particles
        self.particle_filter.draw(surface)

    def update(self):
        """ Sends command to robot, if not in simulator mode. """
        if self.sparki.port is not '':
            # calculate servo setting
            servo = int(self.robot.requested_sonar_angle * 180. / math.pi)

            # calculate motor settings
            left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()

            # send command
            self.sparki.send_command(left_speed, left_dir, right_speed, right_dir, servo)

    def message_received(self, message):
        """ Callback for when a message is received from the robot or the simulator.
            Arguments:
                message: dictionary with status values
        """
        # check if there is no previous timestamp
        if self.last_timestamp == 0:
            self.last_timestamp = message['timestamp']
            return

        # calculate time_delta based on the current and previous timestamps
        time_delta = (message['timestamp'] - self.last_timestamp) / 1000.
        self.last_timestamp = message['timestamp']
        # print(time_delta)

        # update linear and angular velocities
        if 'left_motor_speed' in message.keys():
            # calculate linear and angular velocities from reported motor settings
            left_speed = message['left_motor_speed']
            left_dir = message['left_motor_dir']
            right_speed = message['right_motor_speed']
            right_dir = message['right_motor_dir']

            # print(left_speed,left_dir,right_speed,right_dir)
            self.robot.compute_velocities(left_speed, left_dir, right_speed, right_dir)
        else:
            # copy requested velocities to actual velocities
            self.robot.lin_vel = self.robot.requested_lin_vel
            self.robot.ang_vel = self.robot.requested_ang_vel

        # update robot position using forward kinematics
        self.robot.update(time_delta)

        # update simulate tile sensor reading
        self.robot.is_tile = self.tmap.get_tile(self.robot.x, self.robot.y)

        # update particles
        self.particle_filter.generate(time_delta)
        self.particle_filter.update()
        self.particle_filter.sample()


def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='Particle filter localization demo')
    parser.add_argument('--tmap', type=str, default='map-tiles.txt', help='path to tiles map txt file')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()

    with SparkiSerial(port=args.port) as sparki:
        # make frontend
        frontend = MyFrontEnd(args.tmap, sparki)

        # run frontend
        frontend.run()


if __name__ == '__main__':
    main()
