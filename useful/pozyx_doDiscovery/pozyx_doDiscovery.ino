/**
 * This sketch performs continuous discovery, to discover any pozyx devices nearby using the same UWB settings as the connected pozyx device
 * It is possible to use custom UWB settings to look for differently configured pozyx devices
 *
 * Author, Samuel Van de Velde, Pozyx Labs
 *
 */

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

// indicate if we must use the configured UWB settings of the device, or that we want to use the settings below
boolean bUseDefaultSettings = true;    

// UWB settings to use when bUseDefaultSettings == false
uint8_t channel = 3;        // values 1,2,3,4,5,7
uint8_t bitrate = 0;        // values 0, 1, 2
uint8_t prf = 2;            // values 1, 2
uint8_t plen = 0x08;

int prev_list_size = -1;
uint16_t* devices = new uint16_t[0];
uint16_t* prev_devices = new uint16_t[0];

void setup() {
  Serial.begin(115200);
  while(!Serial); // for the Arduino Leonardo/Micro only
  
  Wire.begin();  
  
  Serial.println(F("Perform discovery"));  
  Serial.println(F("---------------------------------------------------------\n")); 
  Serial.println(F("Channel parameters for discovery: "));  
  
  if(!bUseDefaultSettings)
  {
    UWB_settings_t UWB_settings;
    UWB_settings.channel = channel;
    UWB_settings.bitrate = bitrate;
    UWB_settings.prf = prf;
    UWB_settings.plen = plen;
    UWB_settings.gain_db = 0.0f; 
    int result = Pozyx.setUWBSettings(&UWB_settings); 
    delay(100);
  }
  
  print_uwb_settings(); 
   
  Serial.println(F("\nContinuously performing discovery.."));  
        
}

void loop() {
  // Get new device list
  Pozyx.clearDevices();
  int status = Pozyx.doDiscovery(POZYX_DISCOVERY_ALL_DEVICES);
  uint8_t list_size = 0;
  status = Pozyx.getDeviceListSize(&list_size);
  devices = new uint16_t[list_size];
  if (list_size>0)
    status = Pozyx.getDeviceIds(devices, list_size);

  // Check if device list has changed
  bool hasChanged;
  if (prev_list_size != list_size) {
    hasChanged = true;
  } else {
    hasChanged = !deviceListsAreEqual(devices, prev_devices, list_size);
  }

  // Print device list when changed
  if(hasChanged)
  {
    Serial.print("Number of pozyx devices discovered: ");
    Serial.println(list_size);
  
    if(list_size > 0)
    {
      Serial.println("List of device IDs: ");
      for(int i=0; i<list_size; i++)
      {      
        Serial.print("\t0x");
        Serial.println(devices[i], HEX);
      }
    }
  }

  delete[] prev_devices;
  prev_devices = devices;
  prev_list_size = list_size;
}

void print_uwb_settings(){
  UWB_settings_t UWB_settings;
  Pozyx.getUWBSettings(&UWB_settings);
  Serial.print("Channel number:\t\t");
  Serial.println(UWB_settings.channel);
  
  Serial.print("bitrate:\t\t");
  if(UWB_settings.bitrate == 0)
    Serial.println("110kbit/s");
  else if(UWB_settings.bitrate == 1)
    Serial.println("850kbit/s");
  else if(UWB_settings.bitrate == 2)
    Serial.println("8.5Mbit/s");
    
  Serial.print("PRF: \t\t\t");
  if(UWB_settings.prf == 1)
    Serial.println("16MHz");
  else
    Serial.println("64MHz");
  Serial.print("Preamble length: \t");
  Serial.println(getPreambleLength(UWB_settings.plen));
  Serial.print("gain (dB): \t\t");
  Serial.println(UWB_settings.gain_db);
  Serial.println();

}

int getPreambleLength(int plen_code)
{
  if(plen_code == 0x0C)
	return 4096;
  else if(plen_code == 0x28 )
  	return 2048;
  else if(plen_code == 0x18 )
  	return 1536;
  else if(plen_code == 0x08 )
  	return 1024;
  else if(plen_code == 0x34 )
  	return 512;
  else if(plen_code == 0x24 )
  	return 256;
  else if(plen_code == 0x14 )
  	return 128;
  else if(plen_code == 0x04 )
	return 64;  
  else
    Serial.println("Illigal preamble code");
  
  return 0;
}

bool deviceListsAreEqual(uint16_t* deviceList1, uint16_t* deviceList2, int list_size) {
  bool areEqual = true;
  int i = 0;
  while (areEqual && i<list_size) {
    areEqual = listHasDevice(deviceList2, list_size, deviceList1[i]);
    i++;
  }
  return areEqual;
}

bool listHasDevice(uint16_t* deviceList, int list_size, uint16_t device) {
  bool hasDevice = false;
  int i = 0;
  while (!hasDevice && i<list_size) {
    hasDevice = deviceList[i] == device;
    i++;
  }
  return hasDevice;
}
