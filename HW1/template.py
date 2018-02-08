import pygame
import sys
import time
import math
import argparse
from RobotLib.FrontEnd import *
from RobotLib.IO import *
from RobotLib.Math import *
import numpy as np
from threading import Timer

R = 10.0  # cm (Distance from ICC)
L = 8.4  # cm (Distance between wheels)
DISPLAY_SIZE = 256
V = 2 # 2 cm/s (Robot velocity)
D = 5 # cm (wheel diameter)
VEL_MAX = 1000.0/4096.0*D/2.0*2.0*math.pi #steps/sec*1/max_steps*radius*2pi
class Robot():
    def __init__(self, sparki):
        self.sparki = sparki  # SparkiSerial class from RobotLib.IO
        self.right_velocity = 0
        self.left_velocity = 0
        self.left_dir = 1
        self.right_dir = 1
        self.velocity = 0.0
        self.w = 0
        self.theta = 0
        self.cx = DISPLAY_SIZE / 2
        self.cy = DISPLAY_SIZE / 2
        self.gripper = 0
        self.servo_angle = 0

    def stop_moving(self):
        self.velocity = 0.0

    def move_forward(self):
        self.velocity = V

    def move_backward(self):
        self.velocity = -V

    def turn_left(self):
        self.w = self.get_new_w()

    def turn_right(self):
        self.w = -self.get_new_w()

    def get_new_w(self):
        if self.velocity > 0:
            return V / R
        else:
            # Question: which angular velocity we set here in order to rotate good in all velocities?
            return V

    def stop_turn(self):
        self.w = 0

    def turn_right_servo(self):
        if self.servo_angle < 90:
            self.servo_angle += 10

    def turn_left_servo(self):
        if self.servo_angle > -90:
            self.servo_angle -= 10

    def update_variables(self, time_delta):
        self.right_velocity = self.velocity + self.w * L / 2
        self.left_velocity = self.velocity - self.w * L / 2

        self.right_velocity, self.right_dir = self.normalizeVelocity(self.right_velocity)
        self.left_velocity, self.left_dir = self.normalizeVelocity(self.left_velocity)

        t_move = transform(self.velocity * time_delta,0,self.w * time_delta)
        newT = self.robot_to_map_transfromation().dot(t_move)
        self.cx = newT.item((0, 2))
        self.cy = newT.item((1, 2))
        self.theta = angle(newT)

    def send_command(self):
        self.sparki.send_command(left_speed=self.toPercent(self.left_velocity), left_dir=self.left_dir,
                                 right_speed=self.toPercent(self.right_velocity), right_dir=self.right_dir,
                                 servo_angle=self.servo_angle,
                                 gripper_status=self.gripper)

    def toPercent(self, vel):
        return int(vel / VEL_MAX * 100)

    def normalizeVelocity(self, vel):
        if vel < 0 or self.velocity < 0:
            dir = 0
            vel = abs(vel)
        else:
            dir = 1
        return vel, dir

    def robot_to_map_transfromation(self):
        return transform(self.cx, self.cy, self.theta)

    def robot_to_map_frame(self, px, py):
        T = self.robot_to_map_transfromation()
        p = np.array([px, py, 1]).reshape(3, 1)
        c = T.dot(p)
        return c

    def sonar_to_map_frame(self, px, py):
        T_sonar_to_robot = transform(2.5, 0, self.servo_angle * math.pi / 180)
        T_robot_to_map = self.robot_to_map_transfromation()
        p_sonar = np.array([px, py, 1]).reshape(3, 1)
        return T_robot_to_map.dot(T_sonar_to_robot).dot(p_sonar)

    def draw_robot(self, surface):

        p1 = self.robot_to_map_frame(-5, 4.5)  # top left
        p2 = self.robot_to_map_frame(5, 4.5)  # top right
        p3 = self.robot_to_map_frame(5, -4.5)  # bottom left
        p4 = self.robot_to_map_frame(-5, -4.5)  # bottom right
        p1_sonar = self.sonar_to_map_frame(0, 0)
        dist = self.sparki.dist
        if dist == 4294967295:
            dist = 0
        p2_sonar = self.sonar_to_map_frame(dist, 0)

        pygame.draw.line(surface, (0, 0, 0), project(p1), project(p2))
        pygame.draw.line(surface, (0, 0, 0), project(p2), project(p3))
        pygame.draw.line(surface, (0, 0, 0), project(p3), project(p4))
        pygame.draw.line(surface, (0, 0, 0), project(p4), project(p1))
        pygame.draw.line(surface, (255, 0, 0), project(p1_sonar), project(p2_sonar))

    def changeGripper(self):
        self.gripper = (self.gripper + 1) % 3

    def testCode1(self):
        t = 10.0/V # time to move 10 cm
        self.move_forward()
        wait(t, self.stop_moving)

    def testCode2(self):
        self.move_forward()  # Move forward 5 seconds at 2 cm/s => 10 cm
        wait(5, self.stop_moving)
        sec_90 = math.pi / 2 / V
        wait(5 + 1, self.turn_right)
        wait(5 + 1 + sec_90, self.stop_turn)
        wait(5 + 1 + sec_90 + 1, self.move_forward)
        wait(5 + 1 + sec_90 + 1 + 5, self.stop_moving)


class MyFrontEnd(FrontEnd):
    """ Custom sub-class of FrontEnd
        You can write custom sub-routines here to respond to UI events and calculate updates
    """

    def __init__(self, width, height, robot):
        FrontEnd.__init__(self, width, height)
        self.robot = robot  # Robot class

    def mouseup(self, x, y, button):
        # x,y is position of mouse click
        print('mouse clicked at %d, %d' % (x, y))

    def keydown(self, key):
        # see https://www.pygame.org/docs/ref/key.html for pygame key names, such as pygame.K_UP
        if key == pygame.K_UP:
            self.robot.move_forward()
        elif key == pygame.K_DOWN:
            self.robot.move_backward()
        elif key == pygame.K_RIGHT:
            self.robot.turn_right()
        elif key == pygame.K_LEFT:
            self.robot.turn_left()
        elif key == pygame.K_SPACE:
            self.robot.changeGripper()
        elif key == pygame.K_d:
            self.robot.turn_left_servo()
        elif key == pygame.K_a:
            self.robot.turn_right_servo()
        elif key == pygame.K_1:
            self.robot.testCode1()
        elif key == pygame.K_2:
            self.robot.testCode2()

    def keyup(self, key):
        # see https://www.pygame.org/docs/ref/key.html for pygame key names, such as pygame.K_UP
        if key == pygame.K_UP:
            self.robot.stop_moving()
        elif key == pygame.K_DOWN:
            self.robot.stop_moving()
        elif key == pygame.K_RIGHT:
            self.robot.stop_turn()
        elif key == pygame.K_LEFT:
            self.robot.stop_turn()

    def draw(self, surface):
        # draw robot here
        #
        # draw a rectangle for the robot
        # draw a red line from the sonar to the object it is pinging
        #
        # use pygame.draw.line(surface,color,point1,point2) to draw a line
        # for example, pygame.draw.line(surface,(0,0,0),(0,0),(10,10))
        # draws a black line from (0,0) to (10,0)
        self.robot.draw_robot(surface)

    def update(self, time_delta):
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
        self.robot.update_variables(time_delta)
        self.robot.send_command()


def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='Template')
    parser.add_argument('--width', type=int, default=DISPLAY_SIZE, help='map width')
    parser.add_argument('--height', type=int, default=DISPLAY_SIZE, help='map height')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()

    with SparkiSerial(port=args.port) as sparki:
        # make frontend
        robot = Robot(sparki)
        frontend = MyFrontEnd(args.width, args.height, robot)

        # run frontend
        frontend.run()


def wait(sec, func):
    t = Timer(float(sec), func)
    t.start()


if __name__ == '__main__':
    main()
