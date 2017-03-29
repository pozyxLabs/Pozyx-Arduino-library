#line 2 "unit_tests_core.ino"
#include <ArduinoUnit.h>

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

/**
 * These unit tests verify that the all functionalities regarding saving and loading from/to the flash memory work.
 * These functionalities allow a configuration to be saved and remembered even after a pozyx device is reset.
 * This requires firmware version v1.0
 *
 * Author, Samuel Van de Velde, Pozyx Labs
 *
 * functions tested:
 * - Pozyx.reset
 * - Pozyx.saveConfiguration
 * - Pozyx.clearConfiguration
 * - Pozyx.getNumRegistersSaved
*  - Pozyx.isRegisterSaved
 *
 * This tests the registers:
 * - POZYX_FLASH_DETAILS
 * - POZYX_FLASH_RESET
 * - POZYX_FLASH_SAVE
 */
 
 
void setup()
{
  Serial.begin(115200);
  while(!Serial); // for the Arduino Leonardo/Micro only
  
  Wire.begin();
  
  delay(5000);
  
  Serial.println(F("Start testing flash functionality"));  
  Serial.println(F("---------------------------------------------------------\n")); 
  
  Test::exclude("flash*");
  //Test::exclude("reset*");     
}

void loop()
{
  Test::run();
}

// can we reset the device? we test this to check if the value for the led_config register
// is reset to default after reset.
test(reset)
{   
  uint8_t ledconfig, ledconfig2;
  int result;

  // read the register and change the content
  ledconfig = 0x20;
  Pozyx.regWrite(POZYX_POS_ALG, &ledconfig, 1);
  delay(2);
  Pozyx.regRead(POZYX_POS_ALG, &ledconfig2, 1);  
  assertEqual(ledconfig, ledconfig2);
  
  // reset the system
  Pozyx.resetSystem();  
  
  // after the reset, we should read the default value again.
  ledconfig2 = 0;
  Pozyx.regRead(POZYX_POS_ALG, &ledconfig2, 1);  
  assertNotEqual(ledconfig, ledconfig2);
}

// can we reset the device multiple times? we test this by checking if the value for the led_config register
// is reset to default after reset.
test(multi_reset)
{   
  uint8_t ledconfig, ledconfig2; 
  int i;

  // read the register and change the content
  for(i=0; i<10; i++)
  {
    ledconfig = 0x20;
    Pozyx.regWrite(POZYX_POS_ALG, &ledconfig, 1);
    delay(2);
    Pozyx.regRead(POZYX_POS_ALG, &ledconfig2, 1);  
    assertEqual(ledconfig, ledconfig2);
       
    
    // reset the system
    Pozyx.resetSystem();  
    
    // after the reset, we should read the default value again.
    ledconfig2 = 0;
    Pozyx.regRead(POZYX_POS_ALG, &ledconfig2, 1);  
    assertNotEqual(ledconfig, ledconfig2);
  }
}

/*
 * Test if we can save a single 8-bit register to flash memory
 */
test(flash8bitRegSave)
{
  uint8_t result, num_regs; 
  
  // first perform a memory clear 
  result = Pozyx.clearConfiguration();
  assertEqual(result, POZYX_SUCCESS);
  
  // now read the number of registers that have been saved in flash, should be zero because we just cleared the memory
  num_regs = Pozyx.getNumRegistersSaved();
  assertEqual(num_regs, 0);   
  
  // same if we check if the register is saved
  result = Pozyx.isRegisterSaved(POZYX_POS_ALG);
  assertEqual(num_regs, 0);  
  
  // write a new value in the register
  uint8_t reg_saved = 0x32;
  result = Pozyx.regWrite(POZYX_POS_ALG, &reg_saved, 1);
  assertEqual(result, POZYX_SUCCESS);  
  
  // save to flash  
  uint8_t reg = POZYX_POS_ALG;
  result = Pozyx.saveConfiguration(POZYX_FLASH_REGS, &reg, 1);
  assertEqual(result, POZYX_SUCCESS);  
  
  // now read the number of registers that have been saved in flash again, we saved one register so this should be one
  num_regs = 0;
  num_regs = Pozyx.getNumRegistersSaved();
  assertEqual(num_regs, 1);   
  
  result = Pozyx.isRegisterSaved(POZYX_POS_ALG);
  assertEqual(result, 1);  
  
  // reset the system to see if saving worked
  Pozyx.resetSystem();  
  
  // now read the number of registers that have been saved in flash again, we saved one register so this should be one
  num_regs = 0;
  num_regs = Pozyx.getNumRegistersSaved();
  assertEqual(num_regs, 1);   
  
  result = Pozyx.isRegisterSaved(POZYX_POS_ALG);
  assertEqual(result, 1);  
  
  uint8_t reg_check = 0;
  Pozyx.regRead(POZYX_POS_ALG, &reg_check, 1);
  assertEqual(reg_check, reg_saved);  
  
  // finally perform a memory clear  
  result = Pozyx.clearConfiguration();
  assertEqual(result, POZYX_SUCCESS);  
  
  // reset the system such that all registers are loaded with their default values
  Pozyx.resetSystem();  
}

/*
 * Test if we can save a single 32bit register to flash memory
 */
test(flash32bitRegSave)
{
  uint8_t result, num_regs; 
  
  // first perform a memory clear 
  result = Pozyx.clearConfiguration();
  assertEqual(result, POZYX_SUCCESS);
  
  // now read the number of registers that have been saved in flash, should be zero because we just cleared the memory
  num_regs = Pozyx.getNumRegistersSaved();
  assertEqual(num_regs, 0);   
  
  // same if we check if the register is saved
  result = Pozyx.isRegisterSaved(POZYX_POS_X);
  assertEqual(result, 0);  
  
  // write a new value in the register
  uint32_t reg_saved = 0x69C1C7D5;
  result = Pozyx.regWrite(POZYX_POS_X, (uint8_t*)&reg_saved, 4);
  assertEqual(result, POZYX_SUCCESS);  
  
  // save to flash  
  uint8_t reg = POZYX_POS_X;
  result = Pozyx.saveConfiguration(POZYX_FLASH_REGS, &reg, 1);
  assertEqual(result, POZYX_SUCCESS);  
  
  // now read the number of registers that have been saved in flash again, we saved one register so this should be one
  num_regs = 0;
  num_regs = Pozyx.getNumRegistersSaved();
  assertEqual(num_regs, 1);   
  
  result = Pozyx.isRegisterSaved(POZYX_POS_X);
  assertEqual(result, 1);  
  
  // reset the system to see if saving worked
  Pozyx.resetSystem();  
  
  // now read the number of registers that have been saved in flash again, we saved one register so this should be one
  num_regs = 0;
  num_regs = Pozyx.getNumRegistersSaved();
  assertEqual(num_regs, 1);   
  
  result = Pozyx.isRegisterSaved(POZYX_POS_X);
  assertEqual(result, 1);  
  result = Pozyx.isRegisterSaved(POZYX_POS_X+1);
  assertEqual(result, 0);  
  
  uint32_t reg_check = 0;
  Pozyx.regRead(POZYX_POS_X, (uint8_t*)&reg_check, 4);
  assertEqual(reg_check, reg_saved);  
  
  // finally perform a memory clear  
  result = Pozyx.clearConfiguration();
  assertEqual(result, POZYX_SUCCESS);  
  
  // reset the system such that all registers are loaded with their default values
  Pozyx.resetSystem();  
}

/*
 * Test if we can save a multiple registers to flash memory
 */
test(flashMultiRegSave)
{
  uint8_t result, num_regs; 
  uint8_t registers[10] = {POZYX_POS_X, POZYX_POS_Z, POZYX_POS_Y, POZYX_INT_CONFIG, POZYX_NETWORK_ID, POZYX_UWB_XTALTRIM, POZYX_INT_MASK, POZYX_GPIO1, POZYX_POS_ALG, POZYX_POS_NUM_ANCHORS};
  int reg_size[10] = {4,4,4,1,2,1,1,1,1,1};
  uint32_t reg_vals[10] = {0x12345678, 0x23456789, 0xABCDEF09, 0x01, 0x7784, 0x05, 0x03, 0x1 ,0x32, 7};
  int i;
    
  // first perform a memory clear 
  result = Pozyx.clearConfiguration();
  assertEqual(result, POZYX_SUCCESS);
  
  // now read the number of registers that have been saved in flash, should be zero because we just cleared the memory
  num_regs = Pozyx.getNumRegistersSaved();
  assertEqual(num_regs, 0);   
    
  // write new values in the registers
  for(i=0; i< 5; i++)
  {
    result = Pozyx.regWrite(registers[i], (uint8_t*)&reg_vals[i], reg_size[i]);
    assertEqual(result, POZYX_SUCCESS); 
    delay(5); 
    uint32_t reg_check = 0;
    Pozyx.regRead(registers[i], (uint8_t*)&reg_check, reg_size[i]);
    assertEqual(reg_check, reg_vals[i]);  
  }
    
  // save the first 5 registers to flash  
  result = Pozyx.saveConfiguration(POZYX_FLASH_REGS, registers, 5);
  assertEqual(result, POZYX_SUCCESS);  
  
  // now read the number of registers that have been saved in flash again, we saved one register so this should be one
  num_regs = 0;
  num_regs = Pozyx.getNumRegistersSaved();
  assertEqual(num_regs, 5);   
  
  for(i=0; i< 10; i++)
  {
    result = Pozyx.isRegisterSaved(registers[i]);
    if(i<5){
      assertEqual(result, 1);  
    }else{
      assertEqual(result, 0);  
    }
  }
  
  // write new values in the registers
  for(i=5; i< 10; i++)
  {
    result = Pozyx.regWrite(registers[i], (uint8_t*)&reg_vals[i], reg_size[i]);
    assertEqual(result, POZYX_SUCCESS);  
    delay(2); 
    uint32_t reg_check = 0;
    Pozyx.regRead(registers[i], (uint8_t*)&reg_check, reg_size[i]);
    assertEqual(reg_check, reg_vals[i]);  
  }
  
  // save the remaining 5 registers to flash  
  result = Pozyx.saveConfiguration(POZYX_FLASH_REGS, registers+5, 5);
  assertEqual(result, POZYX_SUCCESS);  
  
  // now read the number of registers that have been saved in flash again, we saved one register so this should be one
  num_regs = 0;
  num_regs = Pozyx.getNumRegistersSaved();
  assertEqual(num_regs, 10);   
  
  for(i=0; i< 10; i++){
    result = Pozyx.isRegisterSaved(registers[i]);
    assertEqual(result, 1);  
  }
  
  // reset the system to see if saving worked
  Pozyx.resetSystem();  
  
  // now read the number of registers that have been saved in flash again, we saved one register so this should be one
  num_regs = 0;
  num_regs = Pozyx.getNumRegistersSaved();
  assertEqual(num_regs, 10);   
  
  for(i=0; i< 10; i++)
  {
    result = Pozyx.isRegisterSaved(registers[i]);
    assertEqual(result, 1);  
    
    uint32_t reg_check = 0;
    Pozyx.regRead(registers[i], (uint8_t*)&reg_check, reg_size[i]);
    assertEqual(reg_check, reg_vals[i]);  
  }
    
  // finally perform a memory clear  
  result = Pozyx.clearConfiguration();
  assertEqual(result, POZYX_SUCCESS);  
  
  // reset the system such that all registers are loaded with their default values
  Pozyx.resetSystem(); 
}


/*
 * Test if we can save the list of anchors, set by setPositioningAnchorIds()
 */
test(flashAnchorsSave){
  uint16_t anchors[4], anchors_afterreset[4];
  int result, i;
  
  // perform a memory clear  
  result = Pozyx.clearConfiguration();
  assertEqual(result, POZYX_SUCCESS);  
  
  // create some anchor ids and store them
  for(i=0; i<4; i++){
    anchors[i] = i*500;
  }  
  result = Pozyx.setPositioningAnchorIds(anchors, 4);
  assertEqual(result, POZYX_SUCCESS);
  
  // now save the anchors ids
  result = Pozyx.saveConfiguration(POZYX_FLASH_ANCHOR_IDS);
  assertEqual(result, POZYX_SUCCESS);  
  
  // reset the system
  Pozyx.resetSystem();  
  delay(500);
  
  // read out the anchors again
  result = Pozyx.getPositioningAnchorIds(anchors_afterreset, 4);  
  for(i=0; i<4; i++){
    assertEqual(anchors_afterreset[i], anchors[i]);
  }
  
  // finally perform a memory clear  
  result = Pozyx.clearConfiguration();
  assertEqual(result, POZYX_SUCCESS);  
  
  // reset the system
  Pozyx.resetSystem();    
}

test(flashDeviceListSave)
{
  int result;  
  uint8_t list_size;
  
  // perform a memory clear  
  result = Pozyx.clearConfiguration();
  assertEqual(result, POZYX_SUCCESS);  
  
  // reset the system
  Pozyx.resetSystem();  
  
  // verify that the list is clear
  list_size = 10;
  result = Pozyx.getDeviceListSize(&list_size);
  assertEqual(result, POZYX_SUCCESS);  
  assertEqual(list_size, 0);  
  
  // add some devices to the list
  int i=0;
   for(i=0; i<4; i++){
     device_coordinates_t anchor;
     anchor.network_id = i+1;
     anchor.flag = 0x1; 
     anchor.pos.x = i+1;
     anchor.pos.y = i+2;
     anchor.pos.z = i+3;
     Pozyx.addDevice(anchor);
   }
   
   // verify that the list is clear
   list_size = 10;
   result = Pozyx.getDeviceListSize(&list_size);
   assertEqual(result, POZYX_SUCCESS);  
   assertEqual(list_size, 4);  
   
   // save the list to flash
   result = Pozyx.saveConfiguration(POZYX_FLASH_NETWORK);
   assertEqual(result, POZYX_SUCCESS);  
  
  // reset the system
  Pozyx.resetSystem();  
  
  list_size = 0;
  result = Pozyx.getDeviceListSize(&list_size);
  assertEqual(result, POZYX_SUCCESS);  
  assertEqual(list_size, 4); 
  
  uint16_t device_ids[4];
  Pozyx.getDeviceIds(device_ids,4);
  
  // verify that the information about each device is still the same
  for(i=0; i<4; i++){
      assertEqual(device_ids[i], i+1);      
      coordinates_t coordinates;
      result = Pozyx.getDeviceCoordinates(device_ids[i], &coordinates);
      assertEqual(result, POZYX_SUCCESS); 
      assertEqual(coordinates.x, i+1);    
      assertEqual(coordinates.y, i+2);    
      assertEqual(coordinates.z, i+3);     
  }  
}