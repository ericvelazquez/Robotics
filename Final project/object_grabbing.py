import argparse

from Robot import Robot
from RobotLib.FrontEnd import *
from RobotLib.IO import *


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
        pass

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
            self.robot.line_left = message['line_left']
            self.robot.line_center = message['line_center']
            self.robot.line_right = message['line_right']

        # update compass
        if 'compass' in message.keys():
            self.robot.compass = message['compass']

        print('rangefinder', self.robot.sonar_distance)
        print('servo_angle', self.robot.sonar_angle)
        print('gripper_status', self.robot.gripper_status)
        print('line_left', self.robot.line_left)
        print('line_center', self.robot.line_center)
        print('line_right', self.robot.line_right)
        print('compass', self.robot.compass)


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
