#include <DynamixelWorkbench.h>

DynamixelWorkbench dxl_wb;

int dxl_id = 4;
int incomingByte = 0;
int theta_position = 0;
int baudrate = 115200;
uint16_t model_number = 1;
byte joints[] = {0,1,2,3,4};    // Could use different function here to search for dynamixels
int joint_num = sizeof(joints)/ sizeof(int);

void setup() {
  byte incoming = 0;
  
  dxl_wb.init("1",1000000);
  Serial.begin(baudrate); //opens serial port sets baudrate
 
}

void loop() {
  
//reply only when you receive data:
if (Serial.available()  > 0){
  Serial.readBytes( joints, 5);
    Serial.print("I received: ");
  for(int i = 0; i < 5; i++)
{
  Serial.print(joints[i]);
  Serial.print(" ");
}
 }
}
/***
  int dxl_id = 4;

  dxl_wb.ping(dxl_id, &model_number);
  dxl_wb.goalPosition(dxl_id,(int32_t)300);
  delay(1000);
  dxl_wb.goalPosition(dxl_id,(int32_t)700);
  delay(1000);
***/
