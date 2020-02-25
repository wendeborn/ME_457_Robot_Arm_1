/***************************************************
 * Testing Code to build serial communication with Robotis
 * Dynamixel Robot Arm
 * 
 * 
 * 
 * 
 */


int incomingByte = 0; // for incoming serial data
int led_pin = 14;     
void setup() {
  pinMode(led_pin, OUTPUT);  //Set up the built-in LED pin as an output
  Serial.begin(115200); // opens serial port, sets data rate to 9600 bps
}

void loop() {
  // reply only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
      if (incomingByte == 32) {
        digitalWrite(led_pin, LOW);   // set to as LOW LED is turn-on
        Serial.println("led_on");
        delay(100);                   // Wait for 0.1 second
        digitalWrite(led_pin, HIGH);  // set to as HIGH LED is turn-off
        Serial.println("led_off");
        //delay(100);                   // Wait for 0.1 second
    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
    }
  }
}
