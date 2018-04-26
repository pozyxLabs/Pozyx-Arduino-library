/**
 * This sketch showcases how to save the UWB channel number into flash memory so that next time, this channel number will be used by default
 *
 * Author, Samuel Van de Velde, Pozyx Labs
 *
 */

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

// channel to set this device to
uint8_t channel_num = 3;

void setup() {
  Serial.begin(115200);
  while(!Serial); // for the Arduino Leonardo/Micro only
  
  Wire.begin();  
  
  Serial.println(F("Changing UWB channel"));  
  Serial.println(F("---------------------------------------------------------\n")); 
  
  uint8_t result, num_regs;   
  
  uint8_t channel_old = 0;
  result = Pozyx.regRead(POZYX_UWB_CHANNEL, &channel_old, 1);  
  Serial.print("Previously set channel: ");
  Serial.println(channel_old);
  
  if(channel_old == channel_num)
  {
    Serial.println("Not changing anything");
  }else
  {  
    // if you want: you can perform a memory clear
    //result = PozyxNew.clearConfiguration(); 
        
    // write a new value in the register
    result = Pozyx.regWrite(POZYX_UWB_CHANNEL, &channel_num, 1);
    delay(100);
    
    // save to flash  
    uint8_t reg = POZYX_UWB_CHANNEL;
    result = Pozyx.saveConfiguration(POZYX_FLASH_REGS, &reg, 1);
    if(result != POZYX_SUCCESS)
    {
      Serial.println(result);
      Serial.println(POZYX_SUCCESS);
      Serial.println("could not save the UWB channel");
    }
    
    // now read the number of registers that have been saved in flash, should be zero because we just cleared the memory
    num_regs = Pozyx.getNumRegistersSaved();
    Serial.print("Number of registers saved: ");
    Serial.println(num_regs);
    
    result = Pozyx.isRegisterSaved(POZYX_UWB_CHANNEL);
    if(result != POZYX_SUCCESS) {
      Serial.println("Channel was not saved.");
      Serial.println(result);
    }  
    
    // reset the Pozyx device to verify if it worked
    Pozyx.resetSystem();
    
    channel_num = 0;
    result = Pozyx.regRead(POZYX_UWB_CHANNEL, &channel_num, 1);  
    Serial.println("Channel after reset: ");
    Serial.println(channel_num);
  }
      
}

void loop() {
  // put your main code here, to run repeatedly:

}
