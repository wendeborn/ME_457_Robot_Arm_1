/***************************
* Arduino Sketch to control a Dynamixel robot arm
* ME 457/557  
* 3/3/2020
* Drew Wendeborn
* dw8@pdx.edu
***************************/
#include <DynamixelWorkbench.h>

DynamixelWorkbench dxl_wb;

const uint8_t joint_num = 5;                // Number of joints in the robot arm, ordering base to end effector
const uint8_t gripper_num = 1;              // Number of grippers on end effector
const uint16_t baudrate = 9600;
const uint16_t arm_delay = 1000;            //Sets the delay in milliseconds between joint movements

uint16_t model_number = 1;                  // Dynamixel model number, defaults to 1
uint8_t incomingByte = 0;                   // for incoming serial data
uint16_t joints[joint_num];                 // Two bytes for each joint number
uint16_t grippers[gripper_num];

void setup() {

  dxl_wb.init("1",1000000);                 // Initialize serial communication between OpenCM and the Dynamixels
  Serial.begin(baudrate);                   // opens serial port, sets data rate to 9600 bps
}

void loop() {
   // reply only when you receive data:
  if (Serial.available() > 0) {             // Check to see if serial data is available
    // read the incoming byte:
     incomingByte = Serial.read(); 
     
      if (incomingByte == 1){               // If Arduino receives a 1 then it sends the joint thetas to MATLAB
        sendThetas();
      } 
      else if (incomingByte == 2){          // If Arduino receives a 2 then it reads the next bytes and drives the robot arm joints
        driveJoints();
      }
      else if (incomingByte == 5){          // Arduino receives a 5 and drives the gripper
        driveGripper();
      }
      else if (incomingByte == 6){          //Arduino receives a 6 and sends the gripper position(s) to MATLAB
        sendGripper();
      }
      else {
         raiseError();                      //If none of the triggers are received then send back an error
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
   for(int i = 0; i <= joint_num ; i++){
          uint8_t hi = Serial.read();
          uint8_t lo = Serial.read();
          uint16_t joint = hi << 8;           // Bit shift the hi byte to the highByte position in the variable
          joint |= lo;
          joints[i] = joint;
        }

   dxl_wb.ping(4, &model_number);
   dxl_wb.goalPosition(4, (int32_t)joints[3]);
  // Serial.write(joints[4]);
   delay(arm_delay);
   
   for (int i = 0; i < joint_num; i++){      //Could use the scan function here instead
    Serial.print("joint: ");
    Serial.println(joints[i]);
    Serial.print((int32_t)joints[i]);
         // dxl_wb.ping(i, &model_number);
          //dxl_wb.goalPosition(i,(int32_t)joints[i]);
          delay(arm_delay);
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
