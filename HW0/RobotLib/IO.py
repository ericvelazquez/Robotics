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

def _make_byte_packet(command,value1=0,value2=0,value3=0,value4=0):
    """ Pack a command with byte values into a packet """

    # pack data
    msg = bytearray([command,value1,value2,value3,value4])
    
    # return framed message
    return _frame_message(msg)

def _make_int_packet(command,value):
    """ Pack a command with an integer value into a packet """

    # pack data
    msg = bytearray([command]) + struct.pack('I',value)

    # return framed message
    return _frame_message(msg)

class SparkiSerial:
    """ Class for communicating with Sparki robot over serial
        set port to '' for simulator mode
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
            raise ValueException('Could not open port %s'%self.port)
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
        self.stop_motors()
        time.sleep(self.min_send_period)
        self.stop_gripper()
        time.sleep(self.min_send_period)

        # stop update timer
        self.should_stop = True
        self.timer.cancel()
        
        # close serial port
        self.ser.close()
        
    def stop_motors(self):
        self._write_message(_make_byte_packet(0))
 
    def step_forward(self,steps):
        self._write_message(_make_int_packet(1,steps))

    def step_backward(self,steps):
        self._write_message(_make_int_packet(2,steps))

    def step_left(self,steps):
        self._write_message(_make_int_packet(3,steps))

    def step_right(self,steps):
        self._write_message(_make_int_packet(4,steps))

    def set_motors(self,leftSpeed,leftDir,rightSpeed,rightDir):
        print("set_motors")
        self._write_message(_make_byte_packet(5,leftSpeed,leftDir,rightSpeed,rightDir))

    def set_servo(self,angle):
        self._write_message(_make_byte_packet(6,angle))

    def set_led(self,red,green,blue):
        self._write_message(_make_byte_packet(7,red,green,blue))
    
    def stop_gripper(self):
        self._write_message(_make_byte_packet(8))

    def close_gripper(self):
        self._write_message(_make_byte_packet(9))

    def open_gripper(self,width):
        self._write_message(_make_int_packet(10,width))

    def wait_for_movement(self,delay=0.2,timeout=5):
        """ Wait for a step movement to end, such as step_foward() """
        if self.port == '':
            return
        # wait for first message
        start_time = time.time()
        while not self.any_message_received:
            if time.time() - start_time > timeout:
                break
        # wait for motors to start running
        start_time = time.time()
        while not self.motors_running:
            if time.time() - start_time > timeout:
                break
        # wait for motors to stop running
        start_time = time.time()
        while self.motors_running:
            if time.time() - start_time > timeout:
                break
    
    def _write_message(self,msg):
        if self.port == '':
            print("Error no port")
            return
        current_time = time.time()
        #print(current_time - self.last_send_time)
        if current_time - self.last_send_time > self.min_send_period:
            self.last_send_time = current_time
            self.ser.write(msg)
            print("Message sent")
    
    def _read_status(self):
        while not self.should_stop:
            try:
                # read magic number
                magic = self.ser.read(1)
                assert(len(magic) == 1)
                
                # check magic number
                magic = struct.unpack('B',magic)[0]
                assert(magic == 255)

                # read data
                data = self.ser.read(8)
                assert(len(data) == 8)
                
                # unpack data
                data_bytes = [struct.unpack('B',data[i])[0] for i in range(8)]
            
                # read checksum
                checksum = self.ser.read(1)
                assert(len(checksum) == 1)
                
                # unpack checksum
                checksum = struct.unpack('B',checksum)[0]
            
                # compare checksums
                assert(checksum == _compute_checksum(data_bytes))
            
                #print([int(data_bytes[i]) for i in range(8)])

                # record values
                self.any_message_received = True
                self.dist = struct.unpack('I',data[0:4])[0]
                self.motors_running = int(data_bytes[4])
                self.light_left = int(data_bytes[5])
                self.light_center = int(data_bytes[6])
                self.light_right = int(data_bytes[7])
            except:
                continue
            break

        if not self.should_stop:
            # create another timer
            self.timer = threading.Timer(self.read_period,self._read_status)
            self.timer.start()


