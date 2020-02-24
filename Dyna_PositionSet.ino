#include <DynamixelWorkbench.h>

DynamixelWorkbench dxl_wb;

void setup() {
  // put your setup code here, to run once:
  dxl_wb.init("1",1000000);
  
  uint16_t model_number = 1;
  dxl_wb.ping(1, &model_number);

  dxl_wb.goalPosition(1,(int32_t)512);
  delay(3000);

}

void loop() {
  // put your main code here, to run repeatedly:
  int dxl_id = 1;
  
  dxl_wb.goalPosition(dxl_id,(int32_t)0);
  delay(3000);
  dxl_wb.goalPosition(dxl_id,(int32_t)1023);
  delay(3000);
}
