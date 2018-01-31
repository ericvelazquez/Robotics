import time
import argparse
from RobotLib.IO import SparkiSerial

def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='Motor test')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()

    with SparkiSerial(port=args.port) as sparki:
        # set motors to turn right
        print("SparkiSerial")
        sparki.set_motors(100,1,100,0)
        # wait for one second
        time.sleep(1)
        # set motors to turn right
        sparki.set_motors(100,0,100,1)
        # wait for one second
        time.sleep(1)
        # stop motors
        sparki.stop_motors()

if __name__ == '__main__':
    main()
