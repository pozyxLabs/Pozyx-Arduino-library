#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>


void setup(){
  Serial.begin(115200);

  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    abort();
  }

  // this performs device check locally.
  device_check(NULL);

  // uncomment and fill in own remote ID to check that device.
  // device_check(0x10);

  network_check_discovery();

}

void device_check(uint16_t remote_id){
  uint8_t data[5] = {0,0,0,0,0};

  if (remote_id == NULL){
    Pozyx.regRead(POZYX_WHO_AM_I, data, 5);
    Serial.println("local device:");
  }else{
    Pozyx.remoteRegRead(remote_id, POZYX_WHO_AM_I, data, 5);
    Serial.print("device 0x");
    Serial.println(remote_id, HEX);
  }

  Serial.print("who am i: ");
  Serial.println(data[0], HEX);
  Serial.print("firmware version: 0x");
  Serial.println(data[1], HEX);
  Serial.print("hardware version: 0x");
  Serial.println(data[2], HEX);
  Serial.print("self test result: 0b");
  Serial.println(data[3], BIN);
  Serial.print("error: 0x");
  Serial.println(data[4], HEX);
}


void network_check_discovery(){
  if( Pozyx.doDiscovery(POZYX_DISCOVERY_ALL_DEVICES) == POZYX_SUCCESS){
    uint8_t num_devices = 0;
    Pozyx.getDeviceListSize(&num_devices);
    Serial.print("Discovery found: ");
    Serial.print(num_devices);
    Serial.println(" device(s).");
    uint16_t tags[num_devices];
    if (num_devices == 0){
      Serial.println("FIN.");
      return;
    }
    Pozyx.getDeviceIds(tags, num_devices);

    for(int i = 0; i < num_devices; i++){
      Serial.print("0x");
      Serial.print(tags[i], HEX);
      if (i != num_devices - 1){
        Serial.print(", ");
      }
    }
    Serial.println();
  }
}

void loop() {

}
