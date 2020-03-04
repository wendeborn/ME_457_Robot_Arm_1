/***************************
* Arduino Sketch to control a Dynamixel robot arm
* ME 457/557  
* 3/3/2020
* Drew Wendeborn
* dw8@pdx.edu
***************************/

const uint8_t joint_num = 5;                // Number of joints in the robot arm
const uint8_t gripper_num = 1;
const uint16_t baudrate = 9600;

uint8_t incomingByte = 0;                   // for incoming serial data
uint16_t joints[joint_num];                 // Two bytes for each joint number
uint16_t grippers[gripper_num];

void setup() {
  Serial.begin(baudrate);                       // opens serial port, sets data rate to 9600 bps
}

void loop() {
   // reply only when you receive data:
  if (Serial.available() > 0) {             // Check to see if serial data is available
    // read the incoming byte:
     incomingByte = Serial.read(); 
      if (incomingByte == 1){
        sendThetas();
      } 
      else if (incomingByte == 2){
        driveJoints();
      }
      else if (incomingByte == 5){
        driveGripper();
      }
      else if (incomingByte == 6){
        sendGripper();
      }
      else {
         raiseError();
      }
  }
}

// send robot arm joints to MATLAB
void sendThetas(){                            
       Serial.write(1);
       for (int i = 0; i < joint_num ; i++){
          Serial.write(highByte(joints[i]));
          Serial.write(lowByte(joints[i]));
       }
}

//function to drive the robot joints
void driveJoints(){
   for(int i = 0; i < joint_num ; i++){
          uint8_t hi = Serial.read();
          uint8_t lo = Serial.read();
          uint16_t joint = hi << 8;           // Bit shift the hi byte to the highByte position in the variable
          joint |= lo;
          joints[i] = joint;
        }
}

//function to move gripper arm
void driveGripper(){
  for(int i = 0; i < gripper_num ; i++){
          uint8_t hi = Serial.read();
          uint8_t lo = Serial.read();
          uint16_t gripper = hi << 8;           // Bit shift the hi byte to the highByte position in the variable
          gripper |= lo;
          grippers[i] = gripper;
        }
}

//function to send gripper position to MATLAB
void sendGripper(){
          Serial.write(6);
            for (int i = 0; i < gripper_num ; i++){
              Serial.write(highByte(grippers[i]));
              Serial.write(lowByte(grippers[i]));
            }
}

//function to signal an error has occured
void raiseError(){
  Serial.write(4);
}
