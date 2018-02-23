import serial
import time
import struct
import threading

def _compute_checksum(msg):
    """ Compute simple 8-bit checksum """
    c = 0
    for b in msg:
        c += b
    c = c % 256
    if c == 255:
        c = 254
    return c

def _frame_message(msg):
    """ Add checksum and magic number to a message """

    # append checksum
    msg += bytearray([_compute_checksum(msg)])

    # add magic number at beginning
    msg = bytearray([255]) + msg

    # return message
    return msg

def _make_packet(values):
    """ Pack a command with byte values into a packet """

    # pack data
    msg = bytearray(values)
    
    # return framed message
    return _frame_message(msg)

class SparkiSerial:
    """ Class for communicating with Sparki robot over serial
        Set port to '' for simulator mode
    """
    def __init__(self,port='/dev/tty.Sparki-DevB',baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.timer = None
        self.dist = 0
        self.motors_running = 0
        self.light_left = 0
        self.light_center = 0
        self.light_right = 0
        self.last_send_time = 0
        self.read_period = 0
        self.min_send_period = 0.01
        self.should_stop = False
        self.any_message_received = False

    def __enter__(self):
        if self.port == '':
            return self
        # try to connect to serial port
        print('connecting to Sparki on port %s at rate %d...'%(self.port,self.baudrate))
        for i in range(5):
            try:
                print('...try %d'%(i+1))
                self.ser = serial.Serial(self.port,baudrate=self.baudrate,timeout=0.01)
                break
            except:
                self.ser = None
                pass
        if self.ser is None:
            raise ValueError('Could not open port %s'%self.port)
        print('connected!')
        
        # clear buffers
        self.ser.flushInput()
        self.ser.flushOutput()
        
        # create timer to read status messages
        self.timer = threading.Timer(self.read_period,self._read_status)
        self.timer.start()
        
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.port == '':
            return

        # stop motors and gripper
        self.send_command()

        # stop update timer
        self.should_stop = True
        self.timer.cancel()
        
        # close serial port
        self.ser.close()
        
    def send_command(self,
                     left_speed=0,left_dir=0,
                     right_speed=0,right_dir=0,
                     servo_angle=0,
                     gripper_status=0):
        """ Send a command to the robot.
            Arguments:
                left_speed: left motor speed (0-100%, 100% = 1000 steps/sec)
                left_dir: left motor direction (0 = counter-clockwise; 1 = clockwise)
                right_speed: right motor speed (0-100%, 100% = 1000 steps/sec)
                right_dir: right motor direction (0 = clockwise; 1 = counter-clockwise)
                servo_angle: servo angle (-90-90 degrees)
                gripper_status: 0 = stop, 1 = open, 2 = close
        """
        self._write_message(_make_packet([left_speed,left_dir,right_speed,right_dir,servo_angle+90,gripper_status]))
    
    def _write_message(self,msg):
        if self.port == '':
            return
        current_time = time.time()
        if current_time - self.last_send_time > self.min_send_period:
            self.last_send_time = current_time
            self.ser.write(msg)
        else:
            print('message dropped')
    
    def _read_status(self):
        while not self.should_stop:
            try:
                # read magic number
                magic = self.ser.read(1)
                assert(len(magic) == 1)
                
                # check magic number
                if isinstance(magic,str):
                    magic = struct.unpack('B',magic)[0]
                else:
                    magic = magic[0]
                assert(magic == 255)

                # read data
                data = self.ser.read(8)
                assert(len(data) == 8)
                
                # unpack data
                if isinstance(data,str):
                    data_bytes = [struct.unpack('B',data[i])[0] for i in range(8)]
                else:
                    data_bytes = data
            
                # read checksum
                checksum = self.ser.read(1)
                assert(len(checksum) == 1)
                
                # unpack checksum
                if isinstance(checksum,str):
                    checksum = struct.unpack('B',checksum)[0]
                else:
                    checksum = checksum[0]
            
                # compare checksums
                assert(checksum == _compute_checksum(data_bytes))
            
                # record values
                self.any_message_received = True
                self.dist = struct.unpack('I',data[0:4])[0]
                self.motors_running = int(data_bytes[4])
                self.light_left = int(data_bytes[5])
                self.light_center = int(data_bytes[6])
                self.light_right = int(data_bytes[7])
            except Exception as e:
                continue
            break

        if not self.should_stop:
            # create another timer
            self.timer = threading.Timer(self.read_period,self._read_status)
            self.timer.start()


