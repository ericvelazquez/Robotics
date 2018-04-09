#include <Sparki.h>

// USB communication
//#define MySerial Serial

// Bluetooth communication
#define MySerial Serial1

#define DEBUG_MESSAGES

byte gMessageBuf[16]; // buffer for Bluetooth messages
unsigned long gLastSendTime = 0; // last time the status packet was sent

// Rate in ms at which status packets are sent
#define kSendPeriod 20

void setup() {
  MySerial.begin(115200);   // open Serial
  
  sparki.clearLCD();
  sparki.println("clearing buffer...");
  sparki.updateLCD();
  while ( MySerial.available() ) MySerial.read();
  sparki.println("done");
  sparki.updateLCD();
  
  sparki.servo(0);       // center servo
  sparki.RGB(0,0,0);     // turn off LED
}

byte computeChecksum(byte buf[], int len) {
  // compute simple 8-bit checksum by adding
  byte sum = 0;
  for ( int i = 0; i < len; i++ ) sum += buf[i];
  // make sure we don't hit the magic number
  if ( sum == 255 ) sum = 254;
  return sum;
}

bool receiveMessage() {
  // check if bytes are available
  if ( !MySerial.available() ) return false;
  
  // read first byte and check for magic number
  byte magic = MySerial.read();
  if ( magic != 255 ) {
    return false;
  }
  
  // read rest of packet including checksum
  if ( MySerial.readBytes(gMessageBuf, 6+1) != 6+1 ) {
    return false;
  }

  // compare checksums
  byte checksum = computeChecksum(gMessageBuf,6);
  if ( checksum != gMessageBuf[6] ) {
    return false;
  }
    
  return true;
}

void parseCommand() {
  // parse command from Bluetooth

#ifdef DEBUG_MESSAGES
  sparki.clearLCD(); // wipe the screen
#endif

  // parse command
  byte left_motor_speed = gMessageBuf[0]; // 0-100; 100% = 1000 steps/sec
  byte left_motor_dir = gMessageBuf[1]; // 1 = counter-clockwise; 0 = clockwise
  byte right_motor_speed = gMessageBuf[2];  // 0-100; 100% = 1000 steps/sec
  byte right_motor_dir = gMessageBuf[3];  // 1 = clockwise; 0 = counter-clockwise
  byte servo_angle = gMessageBuf[4];  // 0-180; 90 is center
  byte gripper_status = gMessageBuf[5]; // 0 = stop; 1 = open; 2 = close
  
#ifdef DEBUG_MESSAGES
  sparki.print("Set motors ");
  sparki.print(left_motor_speed);
  sparki.print(" ");
  sparki.print(left_motor_dir);
  sparki.print(" ");
  sparki.print(right_motor_speed);
  sparki.print(" ");
  sparki.println(right_motor_dir);
#endif
  sparki.motorRotate(MOTOR_LEFT,left_motor_dir?DIR_CCW:DIR_CW,left_motor_speed);
  sparki.motorRotate(MOTOR_RIGHT,right_motor_dir?DIR_CW:DIR_CCW,right_motor_speed);

#ifdef DEBUG_MESSAGES
  sparki.print("Set servo ");
  sparki.println(int(servo_angle)-90);
#endif
  sparki.servo(90-int(servo_angle));
  
  if ( gripper_status == 0 ) {
#ifdef DEBUG_MESSAGES
    sparki.println("Stop gripper");
#endif
    sparki.gripperStop();
  } else if ( gripper_status == 1 ) {
#ifdef DEBUG_MESSAGES
    sparki.println("Open gripper");
#endif
    sparki.gripperOpen();
  } else if ( gripper_status == 2 ) {
#ifdef DEBUG_MESSAGES
      sparki.print("Close gripper ");
#endif
      sparki.gripperClose();
  }

#ifdef DEBUG_MESSAGES
  sparki.updateLCD(); // display all of the information written to the screen
#endif
}

void sendStatusPacket() {
  // magic number
  gMessageBuf[0] = 255;

  // rangefinder ping (4 byte integer)
  unsigned long dist = sparki.ping_single();
  *((unsigned long *)(gMessageBuf+1)) = dist;

  // are motors running?
  gMessageBuf[5] = sparki.areMotorsRunning();

  // light sensors
  gMessageBuf[6] = sparki.lightLeft();
  gMessageBuf[7] = sparki.lightCenter();
  gMessageBuf[8] = sparki.lightRight();

  // checksum
  gMessageBuf[9] = computeChecksum(gMessageBuf+1,8);

  // send packet
  MySerial.write(gMessageBuf,10);
}

void loop() {
  // receive message from Bluetooth or USB if available
  if ( receiveMessage() ) {
    parseCommand();
  }

  unsigned long current_time = millis();

  if ( current_time - gLastSendTime > kSendPeriod ) {
    gLastSendTime = current_time;
    sendStatusPacket();
  }
}
