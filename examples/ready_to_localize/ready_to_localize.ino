#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint8_t num_anchors = 4;
uint16_t anchors[4] = {0x6641,0x6639,0x663D,0x6670};
int32_t heights[4] = {2500, 2500, 2500, 2500};

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);
  
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }
  
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println(F("NOTES:"));
  Serial.println(F("- No parameters required."));
  Serial.println();
  Serial.println(F("- System will auto start calibration"));
  Serial.println();
  Serial.println(F("- System will auto start positioning"));
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println();
  Serial.println(F("Performing auto anchor calibration:"));

  // clear all previous devices in the device list
  Pozyx.clearDevices();
     
  int status = Pozyx.doAnchorCalibration(POZYX_2D, 10, num_anchors, anchors, heights);
  
  if (status != POZYX_SUCCESS){
    Serial.println(status);
    Serial.println(F("ERROR: calibration"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

  printCalibrationResult();
  delay(10000);

  Serial.println(F("Starting positioning: "));

}

void loop(){
  
  coordinates_t position;
  
  int status = Pozyx.doPositioning(&position, POZYX_2_5D, 1000);
  
  if (status == POZYX_SUCCESS){
    printCoordinates(position);
  }
}

void printCoordinates(coordinates_t coor){
  
  i++;
  
  Serial.print("pos(");
  Serial.print(i);
  Serial.print(") = [");
  Serial.print(coor.x);
  Serial.print(" ");
  Serial.print(coor.y);
  Serial.print(" ");
  Serial.print(coor.z);
  Serial.print("];");
  
  /*Serial.print("x_mm: ");
  Serial.print(coor.x);
  Serial.print("\t");
  Serial.print("y_mm: ");
  Serial.print(coor.y);
  Serial.print("\t");
  Serial.print("z_mm: ");
  Serial.print(coor.z);
  Serial.println(); 
  */
  /*
  Serial.print("pos;");
  Serial.print(coor.y + 500);
  Serial.print(";");
  Serial.println(coor.x + 500);
  */
}

void printCalibrationResult(){
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size);
  Serial.print("list size: ");
  Serial.println(status*list_size);
  
  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids,list_size);
  
  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);
  
  coordinates_t anchor_coor;
  for(int i=0; i<list_size; i++)
  {
    
    Serial.print("Anchor ");
    Serial.print(i);
    Serial.print(": 0x");
    Serial.println(device_ids[i], HEX);
        
    status = Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor);
    Serial.print(anchor_coor.x);
    Serial.print(";");
    Serial.println(anchor_coor.y);
    //printCoordinates("",anchor_coor);
    
  }    
}

