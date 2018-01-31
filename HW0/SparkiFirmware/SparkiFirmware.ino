#include <Sparki.h>

#define DEBUG_MESSAGES

byte gMessageBuf[16]; // buffer for Bluetooth messages
unsigned long gLastSendTime = 0; // last time the status packet was sent

// Rate in ms at which status packets are sent
#define kSendPeriod 20

void setup() {
  Serial.begin(9600);   // open USB serial
  Serial1.begin(115200);  // open Bluetooth serial

  sparki.clearLCD();
  sparki.println("clearing buffer...");
  sparki.updateLCD();
  while ( Serial1.available() ) Serial1.read();
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
  if ( !Serial1.available() ) return false;
  sparki.print("Serial available");
  sparki.updateLCD();
  // read first byte and check for magic number
  byte magic = Serial1.read();
  if ( magic != 255 ) {
    Serial.print("bad magic number: ");
    Serial.println(magic);
    return false;
  }
  
  // read rest of packet including checksum
  if ( Serial1.readBytes(gMessageBuf, 6) != 6 ) {
    Serial.println("could not read entire packet");
    return false;
  }

  // compare checksums
  byte checksum = computeChecksum(gMessageBuf,5);
  if ( checksum != gMessageBuf[5] ) {
    Serial.println("bad checksum");
    return false;
  }

  #ifdef DEBUG_MESSAGES
  Serial.print("received packet: ");
  for ( int i = 0; i < 6; i++ ) Serial.print(gMessageBuf[i]);
  Serial.println();
  #endif
  
  return true;
}

void parseCommand() {
  // parse command from Bluetooth

#ifdef DEBUG_MESSAGES
  sparki.clearLCD(); // wipe the screen
#endif

  // parse command
  byte command = gMessageBuf[0];
  unsigned long intValue = *((unsigned long *)(gMessageBuf+1));
  byte value1 = gMessageBuf[1];
  byte value2 = gMessageBuf[2];
  byte value3 = gMessageBuf[3];
  byte value4 = gMessageBuf[4];
  switch ( command ) {
    case 0:  // stop motors
#ifdef DEBUG_MESSAGES
      sparki.println("Stop motors");
#endif
      sparki.moveStop();
      break;

    case 1: // step forward
#ifdef DEBUG_MESSAGES
      sparki.print("Step forward ");
      sparki.println(intValue);
#endif
      sparki.stepForward(intValue);
      break;

    case 2: // step backward
#ifdef DEBUG_MESSAGES
      sparki.print("Step backward ");
      sparki.println(intValue);
#endif
      sparki.stepBackward(intValue);
      break;

    case 3: // step left
#ifdef DEBUG_MESSAGES
      sparki.print("Step left ");
      sparki.println(intValue);
#endif
      sparki.stepLeft(intValue);
      break;

    case 4: // step right
#ifdef DEBUG_MESSAGES
      sparki.print("Step right ");
      sparki.println(intValue);
      sparki.stepRight(intValue);
#endif
      break;

    case 5: // set motors
#ifdef DEBUG_MESSAGES
      sparki.print("Set motors ");
      sparki.print(value1);
      sparki.print(" ");
      sparki.print(value2);
      sparki.print(" ");
      sparki.print(value3);
      sparki.print(" ");
      sparki.println(value4);
      Serial.println("set motors received");
#endif
      sparki.motorRotate(MOTOR_LEFT,value2?DIR_CCW:DIR_CW,value1);
      sparki.motorRotate(MOTOR_RIGHT,value4?DIR_CW:DIR_CCW,value3);
      break;

    case 6: // set servo
#ifdef DEBUG_MESSAGES
      sparki.print("Set servo ");
      sparki.println(value1);
      sparki.servo(value1);
#endif
      break;

    case 7: // set LED
#ifdef DEBUG_MESSAGES
      sparki.print("Set LED ");
      sparki.print(value1);
      sparki.print(" ");
      sparki.print(value2);
      sparki.print(" ");
      sparki.println(value3);
#endif
      sparki.RGB(value1,value2,value3);
      break;

    case 8: // stop gripper
#ifdef DEBUG_MESSAGES
      sparki.println("Stop gripper");
#endif
      sparki.gripperStop();
      break;

    case 9: // open gripper
#ifdef DEBUG_MESSAGES
      sparki.println("Open gripper");
#endif
      sparki.gripperOpen();
      break;
      
    case 10: // close gripper
#ifdef DEBUG_MESSAGES
      sparki.print("Close gripper ");
      sparki.println(intValue);
#endif
      sparki.gripperClose(intValue);
      break;
      
    default:
#ifdef DEBUG_MESSAGES
      sparki.print("Unknown command ");
      sparki.println(command);
#endif
      break;
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
  Serial1.write(gMessageBuf,10);
}

void loop() {
  // receive message from Bluetooth if available
  if ( receiveMessage() ) {
    sparki.print("Receiving");
    sparki.updateLCD();
    parseCommand();
  }

  unsigned long current_time = millis();

  if ( current_time - gLastSendTime > kSendPeriod ) {
    gLastSendTime = current_time;
    sendStatusPacket();
  }
}
