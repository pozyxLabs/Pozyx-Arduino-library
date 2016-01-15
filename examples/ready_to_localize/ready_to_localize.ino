#include "Pozyx.h"
#include "Pozyx_definitions.h"
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

device_coordinates_t anchor0,anchor1,anchor2,anchor3;
int status = 1;

////////////////////////////////////////////////

void setup(){
  Serial.println();
  Serial.begin(115200);

  delay(5000);
  
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
  
  int status = 1;
  status &= Pozyx.clearDevices();
  if (status = POZYX_FAILURE){
    Serial.println(F("ERROR: clearing devices"));
  }
  
  
  // used for heighth (LOUSBERG)
  //setManual();
  //setAuto();

  Serial.println(F("Performing auto anchor calibration:"));

  uint16_t anchors[4] = {0x6641,0x6639,0x663D,0x6670};
  int32_t heights[4] = {2500, 2500, 2500, 2500};
 
  status = Pozyx.doAnchorCalibration(POZYX_2D, 4, 10, anchors, heights);
  
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
  
  int status = 1;
  coordinates_t position;
  
  status &= Pozyx.doPositioning(&position, POZYX_2_5D, 1000);
  //status &= Pozyx.getCoordinates(&position);
  
  if (status == POZYX_SUCCESS){
    printCoordinates(position);
  }
  else{
    Serial.println(F("ERROR: positioning"));
  }
}

void printCoordinates(coordinates_t coor){
  
  Serial.print("x_mm: ");
  Serial.print(coor.x);
  Serial.print("\t");
  Serial.print("y_mm: ");
  Serial.print(coor.y);
  Serial.print("\t");
  Serial.print("z_mm: ");
  Serial.print(coor.z);
  Serial.println(); 
  
  /*
  Serial.print("pos;");
  Serial.print(coor.y + 500);
  Serial.print(";");
  Serial.println(coor.x + 500);
  */
}

/*
void setManual(){
  
  anchor0.network_id = 0x6668;
  anchor0.flag = 0x1; 
  anchor0.pos.x = 0;
  anchor0.pos.y = 0;
  anchor0.pos.z = 2742;
  Serial.println(Pozyx.addDevice(anchor0));
  
  anchor1.network_id = 0x666D;
  anchor1.flag = 0x1; 
  anchor1.pos.x = 0;
  anchor1.pos.y = 0;
  anchor1.pos.z = 2742;
  Serial.println(Pozyx.addDevice(anchor1));
  
  anchor2.network_id = 0x666C;
  anchor2.flag = 0x1; 
  anchor2.pos.x = 0;
  anchor2.pos.y = 0;
  anchor2.pos.z = 2742;
  Serial.println(Pozyx.addDevice(anchor2));
  
  anchor3.network_id = 0x6673;
  anchor3.flag = 0x1; 
  anchor3.pos.x = 0;
  anchor3.pos.y = 0;
  anchor3.pos.z = 2742;
  Serial.println(Pozyx.addDevice(anchor3));
  //coordinates_t anchor_coor;
  //Pozyx.getDeviceCoordinates(anchor3.network_id, &anchor_coor);
  //printCoordinates(anchor_coor);
  
}


void setAuto(){
  
  status &= Pozyx.clearDevices();
  
  Serial.println("AUTO calibration...");
  uint8_t params[10];
 params[0] = 2;    // 2D calibration
 params[1] = 10;    // 10 measurements
 
 uint16_t anchors[4] = {0x6641,0x6639,0x663D,0x6670};
 
 status &= Pozyx.doAnchorCalibration(POZYX_2_5D, anchors, 4, 10);

  uint8_t list_size;
  status &= Pozyx.getDeviceListSize(&list_size);
  Serial.print("list: ");
  Serial.println(status);
  
  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids,list_size);
  
  Serial.println("Calibration result:");
  Serial.print("Anchors found: ");
  Serial.println(list_size);
  
  for(int i=0; i<list_size; i++)
  {
    
    Serial.print("Anchor ");
    Serial.print(i);
    Serial.print(": 0x");
    Serial.println(device_ids[i], HEX);
    
    
    coordinates_t anchor_coor;
    status &= Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor);
    delay(100);
    Serial.print("A_");
    Serial.print(i);
    Serial.print(";");
    Serial.print(anchor_coor.x);
    Serial.print(";");
    Serial.println(anchor_coor.y);
    //printCoordinates("",anchor_coor);
    
  }    
}
*/

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

