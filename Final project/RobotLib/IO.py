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
        self.read_timer = None
        self.write_timer = None
        self.message = None
        self.dist = 0
        self.motors_running = 0
        self.light_left = 0
        self.light_center = 0
        self.light_right = 0
        self.last_send_time = 0
        self.read_period = 0.0
        self.write_period = 0.01
        self.should_stop = False
        self.any_message_received = False
        self.message_callback = None

    def __enter__(self):
        if self.port is not '':
            # try to connect to serial port
            print('connecting to Sparki on port %s at rate %d...'%(self.port,self.baudrate))
            for i in range(5):
                try:
                    print('...try %d'%(i+1))
                    self.ser = serial.Serial(self.port,baudrate=self.baudrate)#,timeout=0.01)
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
            self.read_timer = threading.Timer(self.read_period,self._read_status)
            self.read_timer.start()

            # make lock for messages
            self.message_lock = threading.Lock()

            # intialize command message
            #self.send_command()

            # create timer to write command messages
            #self.write_timer = threading.Timer(self.write_period,self._write_message)
            #self.write_timer.start()
        else:
            # simulator mode

            # create timer to simulate status messages
            self.read_timer = threading.Timer(self.read_period,self._read_status)
            self.read_timer.start()
        
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.port is not '':
            # stop motors and gripper
            self.send_command()

            # stop update timers
            self.should_stop = True
            self.read_timer.cancel()
            #self.write_timer.cancel()
            
            # close serial port
            self.ser.close()
        else:
            # stop update timers
            self.should_stop = True
            self.read_timer.cancel()
        
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
        message = _make_packet([left_speed,left_dir,right_speed,right_dir,servo_angle+90,gripper_status])
        self._write_message(message)
    
    def _write_message(self,message):
        if self.port == '':
            return
        self.ser.write(message)
        #if not self.should_stop:
            ## create another timer
            #self.write_timer = threading.Timer(self.write_period,self._write_message)
            #self.write_timer.start()
    
    def _read_status(self):
        if self.port is not '':
            self.ser.flushInput()
        while not self.should_stop:
            if self.port is not '':
                try:
                    #print('0')

                    # read magic number
                    magic = self.ser.read(1)
                    assert(len(magic) == 1)
                    #print('1')
                    
                    # check magic number
                    if isinstance(magic,str):
                        magic = struct.unpack('B',magic)[0]
                    else:
                        magic = magic[0]
                    #print(magic)
                    assert(magic == 255)
                    #print('2')

                    # read data
                    data = self.ser.read(31)
                    assert(len(data) == 31)
                    #print('3')
                    
                    # unpack data
                    if isinstance(data,str):
                        data_bytes = [struct.unpack('B',data[i])[0] for i in range(31)]
                    else:
                        data_bytes = data
                
                    # read checksum
                    checksum = self.ser.read(1)
                    assert(len(checksum) == 1)
                    #print('4')
                    
                    # unpack checksum
                    if isinstance(checksum,str):
                        checksum = struct.unpack('B',checksum)[0]
                    else:
                        checksum = checksum[0]
                
                    # compare checksums
                    assert(checksum == _compute_checksum(data_bytes))
                    #print('5')
                
                    # record values
                    self.any_message_received = True
                    message = {}
                    message['left_motor_speed'] = int(data_bytes[0])
                    message['left_motor_dir'] = int(data_bytes[1])
                    message['right_motor_speed'] = int(data_bytes[2])
                    message['right_motor_dir'] = int(data_bytes[3])
                    message['servo_angle'] = int(data_bytes[4])
                    message['gripper_status'] = int(data_bytes[5])
                    message['rangefinder'] = struct.unpack('I',data[6:10])[0]
                    message['line_left'] = struct.unpack('I',data[10:14])[0]
                    message['line_center'] = struct.unpack('I',data[14:18])[0]
                    message['line_right'] = struct.unpack('I',data[18:22])[0]
                    message['compass'] = struct.unpack('I',data[22:26])[0]
                    message['timestamp'] = struct.unpack('I',data[26:30])[0]
                except Exception as e:
                    continue

                if self.message_callback is not None:
                    self.message_callback(message)
            else:
                # simulate message with just timestamp
                message = {}
                message['timestamp'] = time.time()*1000.0 # time in ms
                if self.message_callback is not None:
                    self.message_callback(message)
            break

        if not self.should_stop:
            # create another timer
            self.read_timer = threading.Timer(self.read_period,self._read_status)
            self.read_timer.start()


