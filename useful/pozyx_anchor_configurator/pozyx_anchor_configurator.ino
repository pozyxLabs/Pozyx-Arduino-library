#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <stdlib.h>

// change this if you want to perform remote configuration 
uint16_t remote_id = NULL;

#define MAX_DEVICES                20
uint8_t devlist_size = 0;
uint16_t device_ids[MAX_DEVICES];

void setup() {
  Serial.begin(115200);
  while(!Serial); // for the Arduino Leonardo/Micro only
  
  Pozyx.begin();  
  
  Serial.println("\n\nAnchor configurator started.");
  Serial.println("------------------------------------");
  
  if(remote_id == NULL) {
    Serial.print(F("Configuring the local pozyx device with network id: 0x"));
    uint16_t netid = 0;
    Pozyx.getNetworkId(&netid);
    Serial.println(netid, HEX);
  }else{
    uint8_t whoami=0;
    Pozyx.getWhoAmI(&whoami, remote_id);
    if(whoami == 0x43){
      Serial.print(F("Configuring the remote pozyx device with network id: 0x"));
      Serial.println(remote_id, HEX);
    }else{
      Serial.print(F("Cannot connect to the remote device with network id 0x"));
      Serial.println(remote_id, HEX);
      delay(50);
      abort();
    }
  }
  
  printLocalizationOptions(remote_id);
  printMain(remote_id);
}

#define STATE_MAIN                0
#define STATE_ADD_HEX             1
#define STATE_ADD_X               2
#define STATE_ADD_Y               3
#define STATE_ADD_Z               4
#define STATE_REMOVE_ID           5
#define STATE_OPTION_ALGORITHM      6
#define STATE_OPTION_DIM            7
#define STATE_OPTION_NUM_ANCHORS    8
#define STATE_OPTION_ANCHOR_SEL     9
#define STATE_MANUAL_IDS          10

String input;
int state = STATE_MAIN;
int dev_Selected;
device_coordinates_t new_device;
int algorithm = -1;
int numAnchors;
int selected_anchor_num = 0;
uint16_t anchor_sel[15];

void loop() {

  if (Serial.available() > 0) {    // is a character available?
    char rx_byte = Serial.read();       // get the character
  
    if(rx_byte != '\r' && rx_byte != '\n') 
      input += rx_byte;
      
    if(rx_byte == '\r') {
      
      if(state == STATE_MAIN)
      {
        if(input == "add")
        {
          Serial.println("\tAdding new anchor:");
          Serial.print("\tplease give network id (hex): ");
          state = STATE_ADD_HEX;
        }
        else if(input == "remove")    
        {       
          Serial.print("\tGive network id of device to remove (hex): "); 
          state = STATE_REMOVE_ID;  
        }
        else if(input == "remove all")  
        {
          Pozyx.clearDevices(remote_id);
          Pozyx.saveConfiguration(POZYX_FLASH_NETWORK, NULL, 0, remote_id);
          printMain(remote_id);          
        }        
        else if(input == "localization options")
        {          
          Serial.println("\tselect positioning algoirthm [LS, UWB_only, default] ");
          state = STATE_OPTION_ALGORITHM;  
        }else{
          Serial.println("\tUnknown command.");
        }
      }else if(state == STATE_ADD_HEX)
      {
        char input_hex[sizeof(input)];
        input.toCharArray(input_hex, sizeof(input));
        long network_id = strtol(input_hex, NULL, 16);
        if(network_id >0 && network_id <= 0xFFFF)
        {
          new_device.network_id = (uint16_t)network_id;
          Serial.print("0x");
          Serial.println(network_id, HEX);
          Serial.print("\tSet anchor x-coordinate(mm): ");
          state = STATE_ADD_X;
        }else{
          Serial.print(F("\n\tNot a valid network id, try again: "));
        }
      }else if(state == STATE_ADD_X)
      {
        int x_mm = input.toInt();
        if(x_mm >=-100000 && x_mm < 100000)
        {
          new_device.pos.x = x_mm;
          Serial.println(x_mm);
          Serial.print(F("\tSet anchor y-coordinate(mm): "));
          state = STATE_ADD_Y;
        }else{
          Serial.print(F("\n\tNot a valid x-coordinate, try again: "));
        }
      }else if(state == STATE_ADD_Y)
      {
        int y_mm = input.toInt();
        if(y_mm >=-100000 && y_mm < 100000)
        {
          new_device.pos.y = y_mm;
          Serial.println(y_mm);
          Serial.print(F("\tSet anchor z-coordinate(mm): "));
          state = STATE_ADD_Z;
        }else{
          Serial.print(F("\n\tNot a valid y-coordinate, try again: "));
        }
      }else if(state == STATE_ADD_Z)
      {
        int z_mm = input.toInt();
        if(z_mm >=-100000 && z_mm < 100000)
        {
          new_device.pos.z = z_mm;
          Serial.println(z_mm);
          
          new_device.flag = 0x1; 
          int result = Pozyx.addDevice(new_device, remote_id);          	
          Pozyx.saveConfiguration(POZYX_FLASH_NETWORK, NULL, 0, remote_id);
          if(result == POZYX_SUCCESS)
          {
            Serial.println(F("\tAnchor successfully added."));
          }else{
            Serial.println(F("\tFailed to add anchor."));
          }
          state = STATE_MAIN;
          printMain(remote_id);
        }else{
          Serial.print(F("\n\tNot a valid x-coordinate, try again: "));
        }
      }else if(state == STATE_REMOVE_ID)
      {
        char input_hex[sizeof(input)];
        input.toCharArray(input_hex, sizeof(input));
        long network_id = strtol(input_hex, NULL, 16);
        if(network_id >0 && network_id <= 0xFFFF)
        {         
          Serial.print("0x");        
          Serial.println(network_id, HEX);        
          // remove anchor
          remove_anchor(network_id, remote_id);   
          Pozyx.saveConfiguration(POZYX_FLASH_NETWORK, NULL, 0, remote_id);       
          Serial.println("\tAnchor removed");
          
          state = STATE_MAIN;
          printMain(remote_id);
          
        }else{
          Serial.println(F("\n\tNot a valid network id, try again:"));
        }
      } else if(state == STATE_OPTION_ALGORITHM)
      {
        algorithm = -1;
        
        if(input == "LS") {          
          algorithm = POZYX_POS_ALG_LS;
        }else if(input == "UWB_only" || input == "default") {
          algorithm = POZYX_POS_ALG_UWB_ONLY ;
        }else{
          Serial.println("\tInvalid input");
        }
        
        if(algorithm != -1)
        {
          Serial.println(F("\tSelect the localization dimension [2D, 2.5D, 3D]"));
          state = STATE_OPTION_DIM;
        }
      }else if(state == STATE_OPTION_DIM)
      {
        int dim = -1;
        if(input == "2D")
        {
          dim = 2;
        }else if(input == "2.5D"){
          dim = 1;
        }else if(input == "3D"){
          dim = 3;
        }else{
          Serial.println("\tInvalid input");
        }
        
        if(dim != -1)
        {
          Pozyx.setPositionAlgorithm(algorithm,dim, remote_id);
          Serial.println(F("\tChoose the number of anchors to use [3-15]"));
          state = STATE_OPTION_NUM_ANCHORS;
        }
      }else if(state == STATE_OPTION_NUM_ANCHORS)
      {
        numAnchors = input.toInt();
        if(numAnchors >= 3 && numAnchors <= 15 )
        {
          Serial.println(F("\tSelect the anchor selection method [manual, automatic]"));
          state = STATE_OPTION_ANCHOR_SEL;
        }else{
          Serial.println(F("\tInvalid input"));
        }
      }else if(state == STATE_OPTION_ANCHOR_SEL)
      {
        if(input == "manual")
        {
          Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, numAnchors, remote_id );
          uint8_t regs[2] = {POZYX_POS_ALG, POZYX_POS_NUM_ANCHORS};
          Pozyx.saveConfiguration(POZYX_FLASH_REGS, regs, 2, remote_id);          
          state = STATE_MANUAL_IDS;
          Serial.print(F("\tenter network id of anchor 1 (hex): "));
          selected_anchor_num = 1;
        }else if(input == "automatic")
        {          
          Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO , numAnchors, remote_id );
          uint8_t regs[2] = {POZYX_POS_ALG, POZYX_POS_NUM_ANCHORS};
          Pozyx.saveConfiguration(POZYX_FLASH_REGS, regs, 2, remote_id);
          Serial.println(F("\tLocalization options saved"));
          printLocalizationOptions(remote_id);
          state = STATE_MAIN;
          printMain(remote_id);
        }else{
          Serial.println(F("\tInvalid input, please try again"));
        }
      }else if(state == STATE_MANUAL_IDS)
      {
        char input_hex[sizeof(input)];
        input.toCharArray(input_hex, sizeof(input));
        long network_id = strtol(input_hex, NULL, 16);
        if(network_id >0 && network_id <= 0xFFFF)
        {         
          Serial.print("0x");        
          Serial.println(network_id, HEX);    
        
          anchor_sel[selected_anchor_num-1] = network_id;
                  
          if(selected_anchor_num < numAnchors)
          {
            Serial.print(F("\tenter network id of anchor "));
            Serial.print(++selected_anchor_num);
            Serial.print(F(" (hex): "));        
          }else
          {
            Pozyx.setPositioningAnchorIds(anchor_sel, numAnchors, remote_id);
            Pozyx.saveConfiguration(POZYX_FLASH_ANCHOR_IDS, NULL, 0, remote_id);
            printLocalizationOptions(remote_id);
            state = STATE_MAIN;
            printMain(remote_id);
          }
          
        }else{
          Serial.print(F("\n\tNot a valid network id, try again: "));
        }
      }  
      
      input = "";
    }  
    
  } // end: if (Serial.available() > 0)

}

void printLocalizationOptions(uint16_t remote_id)
{
  uint8_t algorithm = 0;
  Pozyx.getPositionAlgorithm(&algorithm, remote_id);
  uint8_t dimension = 0;
  Pozyx.getPositionDimension(&dimension, remote_id);
  
  Serial.print(F("using the "));
  if(algorithm == POZYX_POS_ALG_UWB_ONLY  )
    Serial.print(F("UWB-only"));  
  else if(algorithm == POZYX_POS_ALG_LS  )
    Serial.print(F("Least squares"));  
  Serial.print(F(" algorithm in "));  
  if(dimension == POZYX_2D )
    Serial.println(F("2D."));
  else if(dimension == POZYX_2_5D )
    Serial.println(F("3D with fixed height (2.5D)."));
  else if(dimension == POZYX_3D )
    Serial.println(F("3D."));
    
  uint8_t mode = 0;
  Pozyx.getAnchorSelectionMode(&mode, remote_id);
  uint8_t numAnchors = 0;
  Pozyx.getNumberOfAnchors(&numAnchors, remote_id);
  
  Serial.print(F("using "));
  Serial.print(numAnchors);
  Serial.print(F(" anchors that are "));  
  if(mode == POZYX_ANCHOR_SEL_MANUAL)
  {
    Serial.println(F("manually selected."));
    uint16_t anchors[numAnchors];
    for(int i=0;i<numAnchors;i++)    {
      anchors[i] = 0;
    }
    
    int status = Pozyx.getPositioningAnchorIds(anchors, numAnchors, remote_id);
    for(int i=0;i<numAnchors;i++)
    {
      Serial.print("\t0x");
      Serial.print(anchors[i],HEX);
    }
    Serial.println();
  }else if(mode == POZYX_ANCHOR_SEL_AUTO)
    Serial.println(F("automatically selected."));  
}

void printMain(uint16_t remote_id)
{
  Serial.println(F("\nCurrent devices in device list: (max 20)"));

  int result = Pozyx.getDeviceListSize(&devlist_size, remote_id);  
  if(devlist_size > 0)
  {                
    uint16_t devices[devlist_size];
    result = Pozyx.getDeviceIds(device_ids, devlist_size, remote_id);
    
    int i;
    for(i=0; i<devlist_size; i++)
    {
      coordinates_t anchor_coor;
      Serial.print("\tAnchor ");
      Serial.print(i+1);
      Serial.print(") 0x");
      Serial.print(device_ids[i], HEX);
      result = Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor, remote_id);
      Serial.print("\tx= ");
      Serial.print(anchor_coor.x);
      Serial.print("\ty= ");
      Serial.print(anchor_coor.y);
      Serial.print("\tz= ");
      Serial.println(anchor_coor.z);
    } 
  }else
  {
    Serial.println("\tNone.");
  }  
  
  Serial.println(F("Options [add, remove, remove all, localization options]"));
}

void remove_anchor(uint16_t network_id, uint16_t remote_id)
{  
  uint8_t devlist_size = 0;
  int result = Pozyx.getDeviceListSize(&devlist_size, remote_id);  
  
  if(devlist_size > 0)
  {                
    uint16_t device_ids[devlist_size];
    device_coordinates_t devices[devlist_size];
    result = Pozyx.getDeviceIds(device_ids, devlist_size, remote_id);
       
    // load all the current devices
    for(int i=0; i<devlist_size; i++)    {   
      coordinates_t tmp;
      result = Pozyx.getDeviceCoordinates(device_ids[i], &tmp, remote_id); 
      devices[i].network_id = device_ids[i];
      devices[i].flag = 1;
      devices[i].pos = tmp;     
    } 
    
    // clear the current devices
    Pozyx.clearDevices(remote_id);
    
    
    // add all the devices except the one to delete
    for(int i=0; i<devlist_size; i++)
    {
      if(device_ids[i] != network_id)
      {
        Pozyx.addDevice(devices[i], remote_id);
      }
    }
  } 
}

