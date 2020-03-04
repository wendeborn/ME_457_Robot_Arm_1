
const uint8_t joint_num = 5; // Number of joints in the robot arm

uint8_t incomingByte = 0; // for incoming serial data
uint16_t servo_value = 0; //for the 16 bit number for servo data
uint16_t joints[joint_num]; // Two bytes for each number

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}

void loop() {
  // reply only when you receive data:
  if (Serial.available() > 0) {             // Check to see if serial data is available
    // read the incoming byte:
    incomingByte = Serial.read();           // Read incoming byte and decide what to do with it
      if (incomingByte == 1);
        for(int i = 0; i < joint_num ; i++){
          uint8_t hi = Serial.read();
          uint8_t lo = Serial.read();
          uint16_t joint = hi << 8;         // Bit shift the hi byte to the highByte position in the variable
          joint |= lo;
          joints[i] = joint;
        }
    //servo_value = incomingByte << 8;
    //servo_value |= incomingByte;

    // say what you got:
   // Serial.print("Contents of joints: ");
   Serial.print(1);
    for (int i = 0; i < joint_num ; i++){
      Serial.print(highByte(joints[i]), DEC);
      Serial.print(lowByte(joints[i]),DEC);
    }
    //Serial.println(incomingByte, DEC);
    //Serial.print("I received servo_value: ");
    //Serial.println(servo_value, BIN);
  }
}
