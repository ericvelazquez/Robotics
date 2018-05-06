import argparse
import math
import time

from Robot import Robot
from RobotLib.FrontEnd import *
from RobotLib.IO import *

D = 40

class MyFrontEnd(FrontEnd):
    """ Class for object_grabing final project
    """

    def __init__(self, width, height, sparki):
        FrontEnd.__init__(self, width, height)
        self.sparki = sparki
        self.robot = Robot()

        # message callback
        self.sparki.message_callback = self.message_received

        # last timestamp received
        self.last_timestamp = 0

        # Maintains the state of the robot
        # 0 - Centered
        # 1 - Looking for object
        # 2 - Centering object
        # 3 - Going to object
        # 4 - Grabbing object
        # 5 - Going to base
        # 6 - Release object
        self.state = 0

        # last timestamp for update use
        self.last_timestamp_update = 0

        # timestamp orientations of the object. We use timestamp because compass is not workings properly
        self.object_right_timestamp = 0
        self.object_left_timestamp = 0

        # time to desired orientation
        self.rotate_until_timestamp = 0


    def mouseup(self, x, y, button):
        # x,y is position of mouse click
        print('mouse clicked at %d, %d' % (x, y))

    def keydown(self, key):
        # see https://www.pygame.org/docs/ref/key.html for pygame key names, such as pygame.K_UP
        print('key pressed')

    def keyup(self, key):
        # see https://www.pygame.org/docs/ref/key.html for pygame key names, such as pygame.K_UP
        print('key released')

    def draw(self, surface):
        pass

    def update(self):
        gripper = 0
        if self.state == 0:
            # open grip
            if self.last_timestamp_update == 0:
                self.last_timestamp_update = self.getMillis()

            gripper = 1

            if (self.getMillis() - self.last_timestamp_update) > 4000:
                # change to sate 1
                gripper = 0
                self.last_timestamp_update = 0
                print "state 1"
                self.state = 1

        elif self.state == 1:
            # keep rotating until object found
            self.robot.requested_ang_vel = 5. * math.pi / 180.
            if self.robot.sonar_distance < D:
                #object found
                self.robot.requested_ang_vel = 0
                self.resetObjectTimestamps()
                print "state 2"
                self.state = 2

        elif self.state == 2:
            # Rotate robot to face the center of the object
            if self.robot.sonar_distance == 4294967295:
                return
            if self.rotate_until_timestamp == 0:
                self.robot.requested_ang_vel = 5. * math.pi / 180.
                if self.robot.sonar_distance < D and self.object_right_timestamp == 0:
                    print 'get right timestamp'
                    self.object_right_timestamp = self.getMillis()
                elif self.robot.sonar_distance > D and self.object_right_timestamp != 0:
                    # No more facing object
                    print 'get left timestamp'
                    self.object_left_timestamp = self.getMillis()
                    self.rotate_until_timestamp = self.getMillis() + (self.object_left_timestamp - self.object_right_timestamp)/2
            elif self.rotate_until_timestamp != 0 and self.rotate_until_timestamp-self.getMillis()>0:
                # Move right
                print 'move to center'
                self.robot.requested_ang_vel = -5. * math.pi / 180.
            else:
                # Facing the center of the object
                self.rotate_until_timestamp = 0
                self.robot.requested_ang_vel = 0
                print "state 3"
                self.state = 3

        elif self.state == 3:
            # keep moving forward until distance < 5. As sensor cannot detect closer than 5, keep moving for 1 second
            # in order to grab the object correctly
            self.robot.requested_lin_vel = 2.0

            if self.robot.sonar_distance < 5 and self.last_timestamp_update == 0:
                self.last_timestamp_update = self.getMillis()

            if self.last_timestamp_update != 0 and (self.getMillis() - self.last_timestamp_update) > 500:
                self.last_timestamp_update = 0
                self.robot.requested_lin_vel = 0
                print "state 4"
                self.state = 4

        elif self.state == 4:
            if self.last_timestamp_update == 0:
                self.last_timestamp_update = self.getMillis()

            gripper = 2

            if (self.getMillis() - self.last_timestamp_update) > 6000:
                # change to sate 1
                gripper = 0
                self.last_timestamp_update = 0
                print "state 5"
                self.state = 5

        elif self.state == 5:
            #rotate until facing the base
            #keep moving until base
            #state 5
            pass

        elif self.state == 6:
            print "state 6"
            if self.last_timestamp_update == 0:
                self.last_timestamp_update = self.getMillis()

            gripper = 1

            if (self.getMillis() - self.last_timestamp_update) > 4000:
                # change to sate 1
                gripper = 0
                self.last_timestamp_update = 0
                self.state = 1
            pass

        """ Sends command to robot, if not in simulator mode. """
        if self.sparki.port is not '':
            # calculate servo setting
            servo = int(self.robot.requested_sonar_angle * 180. / math.pi)

            # calculate motor settings
            left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()

            # send command
            self.sparki.send_command(left_speed, left_dir, right_speed, right_dir, 0, gripper)

    def getMillis(self):
        return int(round(time.time() * 1000))

    def resetObjectTimestamps(self):
        self.object_right_timestamp = 0
        self.object_left_timestamp = 0

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
            self.robot.sonar_distance = message['rangefinder']

        # update sonar angle
        if 'servo_angle' in message.keys():
            self.robot.sonar_angle = message['servo_angle']

        # update gripper status
        if 'gripper_status' in message.keys():
            self.robot.gripper_status = message['gripper_status']

        # update line sensors
        if 'line_left' in message.keys():
            if message['line_left'] < 500:
                self.robot.line_left = 1
            else:
                self.robot.line_left = 0

            if message['line_center'] < 500:
                self.robot.line_center = 1
            else:
                self.robot.line_center = 0

            if message['line_right'] < 500:
                self.robot.line_right = 1
            else:
                self.robot.line_right = 0

        # update compass
        if 'compass' in message.keys():
            self.robot.compass = message['compass']/100

        if self.robot.sonar_distance != 4294967295:
            print('rangefinder', self.robot.sonar_distance)
        # print('servo_angle', self.robot.sonar_angle)
        # print('compass', self.robot.compass)
        # print('girpper_status', self.robot.gripper_status)


def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='Template')
    parser.add_argument('--width', type=int, default=256, help='map width')
    parser.add_argument('--height', type=int, default=256, help='map height')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()

    with SparkiSerial(port=args.port) as sparki:
        # make frontend
        frontend = MyFrontEnd(args.width, args.height, sparki)

        # run frontend
        frontend.run()


if __name__ == '__main__':
    main()
