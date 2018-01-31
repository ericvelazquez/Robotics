import time
import argparse
from RobotLib.IO import SparkiSerial

def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='Rangefinder test')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()

    with SparkiSerial(port=args.port) as sparki:
        for i in range(10):
            # print the distance in cm recorded by the rangefinder
            print('distance: %d cm'%sparki.dist)
            # wait one second
            time.sleep(1)

if __name__ == '__main__':
    main()
