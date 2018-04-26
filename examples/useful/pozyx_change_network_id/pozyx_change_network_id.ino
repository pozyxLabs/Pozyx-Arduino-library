#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>


uint16_t new_id = 0x6000;    // the new network id of the pozyx device, change as desired
bool remote = false;         // whether to use the remote device
uint16_t remote_id = 0x6068; // the remote ID

void setup(){
  Serial.begin(115200);
  Serial.print("Setting the POZYX ID to 0x");
  Serial.println(new_id, HEX);

  if (!remote){
    remote_id = NULL;
  }
  
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    abort();
  }
  
  Pozyx.setNetworkId(new_id, remote_id);

  uint8_t regs[1] = {POZYX_NETWORK_ID};
  
  int status = Pozyx.saveConfiguration(POZYX_FLASH_REGS, regs, 1, remote_id);
  if(status == POZYX_SUCCESS){
    Serial.println("Saving to flash was successful! Resetting system...");
    Pozyx.resetSystem(remote_id);
  }else{
    Serial.println("Saving to flash was unsuccessful!");
  }
}

void loop() {
  
} 
