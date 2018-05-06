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
    def __init__(self, omap_path, sparki):
        self.omap = ObstacleMap(omap_path, noise_range=1)
        self.rangefinder = Rangefinder(cone_width=deg2rad(15.), obstacle_width=1.)

        FrontEnd.__init__(self, self.omap.width, self.omap.height)

        self.sparki = sparki
        self.robot = Robot()

        # center robot
        self.robot.x = self.omap.width * 0.5
        self.robot.y = self.omap.height * 0.5

        # create particle filter
        alpha = [0.5, 0.5, 0.5, 0.5]
        self.particle_filter = ParticleFilter(num_particles=50, alpha=alpha, robot=self.robot, omap=self.omap)

        # set message callback
        self.sparki.message_callback = self.message_received

        # last timestamp received
        self.last_timestamp = 0

    def keydown(self, key):
        # update velocities based on key pressed
        if key == pygame.K_UP:  # set positive linear velocity
            self.robot.requested_lin_vel = 2.0
        elif key == pygame.K_DOWN:  # set negative linear velocity
            self.robot.requested_lin_vel = -2.0
        elif key == pygame.K_LEFT:  # set positive angular velocity
            self.robot.requested_ang_vel = 10. * math.pi / 180.
        elif key == pygame.K_RIGHT:  # set negative angular velocity
            self.robot.requested_ang_vel = -10. * math.pi / 180.
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
        self.omap.draw(surface)

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

        # update sonar ping distance
        if 'rangefinder' in message.keys():
            # convert to cm
            self.robot.sonar_distance = message['rangefinder']
        else:
            # simulate rangefinder reading
            T_sonar_map = self.robot.get_robot_map_transform() * self.robot.get_sonar_robot_transform()
            self.robot.sonar_distance = self.omap.get_first_hit(T_sonar_map)

        # update particles
        self.particle_filter.generate(time_delta)
        self.particle_filter.update()
        self.particle_filter.sample()


def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='Particle filter localization demo')
    parser.add_argument('--omap', type=str, default='map.txt', help='path to obstacle map txt file')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()

    with SparkiSerial(port=args.port) as sparki:
        # make frontend
        frontend = MyFrontEnd(args.omap, sparki)

        # run frontend
        frontend.run()


if __name__ == '__main__':
    main()
