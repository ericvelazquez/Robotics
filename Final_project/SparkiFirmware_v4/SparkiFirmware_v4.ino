#include <Sparki.h>

// USB communication
//#define MySerial Serial

// Bluetooth communication
#define MySerial Serial1

#define DEBUG_MESSAGES

byte gMessageBuf[33]; // buffer for Bluetooth messages
unsigned long gLastSendTime = 0; // last time the status packet was sent

byte gLeftMotorSpeed = 0;
byte gLeftMotorDir = 0;
byte gRightMotorSpeed = 0;
byte gRightMotorDir = 0;
byte gServoAngle = 0;
byte gGripperStatus = 0;

// Rate in ms at which status packets are sent
#define kSendPeriod 50

void setup() {
  MySerial.begin(115200);   // open Serial
  
  sparki.clearLCD();
//  sparki.println("clearing buffer...");
//  sparki.updateLCD();
  while ( MySerial.available() ) MySerial.read();
//  sparki.println("done");
  sparki.updateLCD();
  
  sparki.servo(0);       // center servo
  sparki.RGB(0,0,0);     // turn off LED
}

byte computeChecksum(byte buf[], int len) {
  // compute simple 8-bit checksum by adding
  int sum = 0;
  for ( int i = 0; i < len; i++ ) sum += buf[i];
  sum %= 256;
  // make sure we don't hit the magic number
  if ( sum == 255 ) sum = 254;
  return sum;
}

bool receiveMessage() {
  if ( !MySerial.available() ) return false;
  
  byte magic = 0;
  while ( magic != 255 ) {
    // read first byte
    if ( MySerial.readBytes(&magic, 1) != 1 ) return false;
  }
  
  // read rest of packet including checksum
  if ( MySerial.readBytes(gMessageBuf, 6+1) != 6+1 ) return false;

//  for ( int i = 0; i < 3; i++ ) {
//    sparki.print(gMessageBuf[i]);
//    sparki.print(" ");
//  }
//  sparki.println();
//  sparki.print(" ");
//  for ( int i = 4; i < 7; i++ ) {
//    sparki.print(gMessageBuf[i]);
//    sparki.print(" ");
//  }
//  sparki.println();
//  sparki.updateLCD();
  
  // compare checksums
  byte checksum = computeChecksum(gMessageBuf,6);
  if ( checksum != gMessageBuf[6] ) {
//    sparki.print("bad checksum: ");
//    sparki.print(checksum);
//    sparki.print(" ");
//    sparki.println(gMessageBuf[6]);
//    sparki.updateLCD();
    sparki.RGB(255,0,0);
    return false;
  }

//  sparki.println("got message");
//  sparki.updateLCD();
    sparki.RGB(0,255,0);
  
  return true;
}

void parseCommand() {
  // parse command from Bluetooth

#ifdef DEBUG_MESSAGES
  sparki.clearLCD(); // wipe the screen
#endif

  // parse command
  gLeftMotorSpeed = gMessageBuf[0]; // 0-100; 100% = 1000 steps/sec
  gLeftMotorDir = gMessageBuf[1]; // 1 = counter-clockwise; 0 = clockwise
  gRightMotorSpeed = gMessageBuf[2];  // 0-100; 100% = 1000 steps/sec
  gRightMotorDir = gMessageBuf[3];  // 1 = clockwise; 0 = counter-clockwise
  gServoAngle = gMessageBuf[4];  // 0-180; 90 is center
  gGripperStatus = gMessageBuf[5]; // 0 = stop; 1 = open; 2 = close
  
#ifdef DEBUG_MESSAGES
  sparki.print("Set motors ");
  sparki.print(gLeftMotorSpeed);
  sparki.print(" ");
  sparki.print(gLeftMotorDir);
  sparki.print(" ");
  sparki.print(gRightMotorSpeed);
  sparki.print(" ");
  sparki.println(gRightMotorDir);
#endif
  sparki.motorRotate(MOTOR_LEFT,gLeftMotorDir?DIR_CCW:DIR_CW,gLeftMotorSpeed);
  sparki.motorRotate(MOTOR_RIGHT,gRightMotorDir?DIR_CW:DIR_CCW,gRightMotorSpeed);

#ifdef DEBUG_MESSAGES
  sparki.print("Set servo ");
  sparki.println(int(gServoAngle)-90);
#endif
  sparki.servo(90-int(gServoAngle));
  
  if ( gGripperStatus == 0 ) {
#ifdef DEBUG_MESSAGES
    sparki.println("Stop gripper");
#endif
    sparki.gripperStop();
  } else if ( gGripperStatus == 1 ) {
#ifdef DEBUG_MESSAGES
    sparki.println("Open gripper");
#endif
    sparki.gripperOpen();
  } else if ( gGripperStatus == 2 ) {
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

  // motor settings
  gMessageBuf[1] = gLeftMotorSpeed;
  gMessageBuf[2] = gLeftMotorDir;
  gMessageBuf[3] = gRightMotorSpeed;
  gMessageBuf[4] = gRightMotorDir;
  gMessageBuf[5] = gServoAngle;
  gMessageBuf[6] = gGripperStatus;

  // rangefinder ping (4 byte integer)
  unsigned long dist = sparki.ping_single();
  *((unsigned long *)(gMessageBuf+7)) = dist;

  // line sensors (4 byte integers)
  unsigned long left = sparki.lineLeft();
  *((unsigned long *)(gMessageBuf+11)) = left;

  unsigned long center = sparki.lineCenter();
  *((unsigned long *)(gMessageBuf+15)) = center;
  
  unsigned long right = sparki.lineRight();
  *((unsigned long *)(gMessageBuf+19)) = right;

  // compass reading: centidegrees (4 byte integer)
  unsigned long centidegrees = (unsigned long) (sparki.compass() * 100.0);
  *((unsigned long *)(gMessageBuf+23)) = centidegrees;

  // timestamp
  unsigned long timestamp = millis();
  *((unsigned long *)(gMessageBuf+27)) = timestamp;

  // checksum
  gMessageBuf[32] = computeChecksum(gMessageBuf+1,31);
  
  // send packet
  MySerial.write(gMessageBuf,33);
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
