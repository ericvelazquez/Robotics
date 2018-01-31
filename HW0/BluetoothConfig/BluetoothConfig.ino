void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
}

void loop() {
  while ( Serial.available() ) Serial1.write(Serial.read());
  while ( Serial1.available() ) Serial.write(Serial1.read());
} 

// HC-06 Bluetooth commands:
// AT           Test
// AT+RESET     Reset
// AT+VERSION   Get version
// AT+NAME...   Set name
// AT+BAUD8     Set baud rate: 4 = 9600, 8 = 115200

