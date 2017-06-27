/**
 * This sketch will scan for ALL pozyx devices on all UWB settings
 * After scanning, it is possible to select a device and set its UWB settings for later later use
 * All settings configured this way are saved in flash memory.
 *
 * Note that in order to use this sketch, it is necessary to enable the CR (carriage return) in the Serial Monitor
 *
 * Example use:
 * ---------------------------------------
 * wait untill all devices are found.
 * input '1' + 'enter' to select the first device, which is the local device
 * next, input 'channel' + 'enter', then '3' + 'enter', to switch to the 3th channel
 * next, input 'bitrate' + 'enter', then '850'+'enter', to select bitrate 850kb/sec
 * finally, input 'save' + 'enter' to save the settings
 *
 * Author, Samuel Van de Velde, Pozyx Labs
 *
 */

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

// all possible options
uint8_t channels[6] = {1,2,3,4,5,7};
uint8_t bitrate[3] = {0, 1, 2};
uint8_t prf[2] = {1, 2};
uint8_t plens[8] = {0x0C, 0x28, 0x18, 0x08, 0x34, 0x24, 0x14, 0x04};
int plen_vals[8] = {4096, 2048, 1536, 1024, 512 , 256 , 128 , 64};

// data about the devices found
#define MAX_DEVICES    20
int devices_found;
uint16_t devId[MAX_DEVICES];
UWB_settings_t devSettings[MAX_DEVICES];

bool long_plen = false;

void setup() {
  Serial.begin(115200);
  while(!Serial); // for the Arduino Leonardo/Micro only

  Pozyx.begin(false);

  Serial.println(F("Perform scanning"));
  Serial.println(F("---------------------------------------------------------\n"));

  Serial.println("Local device: ");
  Pozyx.getNetworkId(&devId[devices_found]);
  Pozyx.getUWBSettings(&devSettings[devices_found]);
  devices_found++;
  Serial.print("1) 0x");
  Serial.print(devId[0], HEX);
  Serial.print(": channel ");
  Serial.print(devSettings[0].channel);
  printChannelSettings(devSettings[0]);

  Serial.println(F("\nScanning remote devices.."));

  int chan_num, bitrate_num, prf_num, plen_num;

  for(chan_num = 0; chan_num < 6; chan_num++)
  {
    Serial.print(F("\nChannel: "));
    Serial.print(channels[chan_num]);
  for(plen_num = 0; plen_num < 8; plen_num++)  {
  for(prf_num = 0; prf_num < 2; prf_num++)  {
  for(bitrate_num = 0; bitrate_num < 3; bitrate_num++)  {

    UWB_settings_t UWB_settings;
    UWB_settings.channel = channels[chan_num];
    UWB_settings.bitrate = bitrate[bitrate_num];
    UWB_settings.prf = prf[prf_num];
    UWB_settings.plen = plens[plen_num];
    UWB_settings.gain_db = 0.0f;
    int result = Pozyx.setUWBSettings(&UWB_settings);
    delay(20);

    Pozyx.clearDevices();
    if (long_plen) {
      Pozyx.doDiscovery(POZYX_DISCOVERY_ALL_DEVICES, 3, 200);
    } else {
      Pozyx.doDiscovery(POZYX_DISCOVERY_ALL_DEVICES);
    }

    uint8_t list_size = 0;
    result = Pozyx.getDeviceListSize(&list_size);
    if(list_size > 0)
    {
      uint16_t devices[list_size];
      result = Pozyx.getDeviceIds(devices, list_size);

      int i;
      int bFirstFound = 1;
      for(i=0; i<list_size; i++)
      {
        uint8_t plen_read = 0;
        Pozyx.remoteRegRead(devices[i], POZYX_UWB_PLEN, &plen_read, 1);
        if(plen_read != plens[plen_num])
        {
          continue;
        }

        // print only when found for the first time
        if(bFirstFound){
          Serial.println();
          printChannelSettings(UWB_settings);
          bFirstFound = 0;
        }

        // add the device to the list of this sketch
        devId[devices_found] = devices[i];
        devSettings[devices_found] = UWB_settings;
        devices_found++;

        uint8_t data[3];
        Pozyx.remoteRegRead(devices[i], POZYX_FIRMWARE_VER, data, 3);
        uint8_t dev = (data[1] & 0xE0)>>5;

        Serial.print("\t");
        Serial.print(devices_found);
        Serial.print(") 0x");
        Serial.print(devices[i], HEX);

        if(dev == 0)
          Serial.print(F(": anchor v1."));
        else
          Serial.print(F(": tag v1."));
        Serial.print(data[1]&0x1F);
        Serial.print(", FW v");
        Serial.print((data[0]&0xF0)>>4);
        Serial.print(".");
        Serial.print((data[0]&0x0F));

        if(dev == 0){
          if(data[2] != 0b110000){
            Serial.print(F(". WARNING SELFTEST: "));
            Serial.print(data[2], BIN);
          }
        }else if(dev == 1)
        {
          if(data[2] != 0b111111){
            Serial.print(". WARNING SELFTEST: ");
            Serial.print(data[2], BIN);
          }
        }
        Serial.println();

      }
    }
  }
  }
  }
  }

  Serial.println(F("\nScan ended."));

  if(devices_found == 0)
    abort();

  Serial.print(F("\nPlease select a device [1-"));
  Serial.print(devices_found);
  Serial.println("]: ");

}

#define STATE_DEV_SELECTION        0
#define STATE_DEV_OPTION           1
#define STATE_DEV_OPTION_CHANNEL   2
#define STATE_DEV_OPTION_BITRATE   3
#define STATE_DEV_OPTION_PRF       4
#define STATE_DEV_OPTION_PLEN      5
#define STATE_DEV_OPTION_SAVE      6

String input;
int state = STATE_DEV_SELECTION;
UWB_settings_t tmp_settings;
int dev_Selected;

void loop() {

  if (Serial.available() > 0) {    // is a character available?
    char rx_byte = Serial.read();       // get the character

    if(rx_byte != '\r' && rx_byte != '\n')
      input += rx_byte;

    if(rx_byte == '\r') {

      if(state == STATE_DEV_SELECTION)
      {
        int deviceIdx = input.toInt()-1;
        if(deviceIdx >=0 && deviceIdx < devices_found)
        {
          dev_Selected = deviceIdx;
          Serial.print(F("\tDevice selected: 0x"));
          Serial.println(devId[deviceIdx], HEX);
          Serial.println(F("\tOptions [channel, bitrate, prf, plen, save, return]:"));
          tmp_settings = devSettings[dev_Selected];
          state = STATE_DEV_OPTION;
        }else{
          Serial.println(F("\tUnknown device"));
          Serial.print(F("\nPlease select a device [1-"));
          Serial.print(devices_found);
          Serial.println("]: ");
        }
      }else if(state == STATE_DEV_OPTION)
      {
        if(input == "channel")    {
          Serial.println(F("\t\tPlease select channel [1,2,3,4,5,7,return]: "));
          state = STATE_DEV_OPTION_CHANNEL;
        }else if(input == "bitrate")    {
          Serial.println(F("\t\tPlease select bitrate [110,850,6810,return]: "));
          state = STATE_DEV_OPTION_BITRATE;
        }else if(input == "prf")    {
          Serial.println(F("\t\tPlease select channel [16,64,return]: "));
          state = STATE_DEV_OPTION_PRF;
        }else if(input == "plen")    {
          Serial.println(F("\t\tPlease select plen [64,128,256,512,1024,1536,2048,4096,return]: "));
          state = STATE_DEV_OPTION_PLEN;
        }else if(input == "save")
        {
          if(dev_Selected != 0)
          {
            // go to the old settings of the target device
            Pozyx.setUWBSettings(&devSettings[dev_Selected]);
            delay(20);
            // update the settings of the target device
            tmp_settings.gain_db = 0;
            Pozyx.setUWBSettings(&tmp_settings, devId[dev_Selected]);
            devSettings[dev_Selected] = tmp_settings;
            delay(20);

            // go to the new settings of the target device
            Pozyx.setUWBSettings(&devSettings[dev_Selected]);
            delay(20);

            // save the settings on the target device
            uint8_t reg[3] = {POZYX_UWB_CHANNEL, POZYX_UWB_RATES, POZYX_UWB_PLEN};
            Pozyx.saveConfiguration(POZYX_FLASH_REGS, reg, 3, devId[dev_Selected]);

            int result = Pozyx.isRegisterSaved(POZYX_UWB_CHANNEL, devId[dev_Selected] );
            if(result != POZYX_SUCCESS) {
              Serial.println(F("\t\tSave unsuccessful."));
              Serial.println(result);
            }else{
              Serial.println(F("\t\tSettings saved. "));
            }
          }else{
            devSettings[0] = tmp_settings;
            devSettings[0].gain_db = 0;
            Pozyx.setUWBSettings(&devSettings[0]);
            delay(20);

            // save the settings on the target device
            uint8_t reg[3] = {POZYX_UWB_CHANNEL, POZYX_UWB_RATES, POZYX_UWB_PLEN};
            Pozyx.saveConfiguration(POZYX_FLASH_REGS, reg, 3);

            int result = Pozyx.isRegisterSaved(POZYX_UWB_CHANNEL);
            if(result != POZYX_SUCCESS) {
              Serial.println(F("\t\tSave unsuccessful."));
              Serial.println(result);
            }else{
              Serial.println(F("\t\tSettings saved. "));
            }
          }
          goToDeviceSelect();
        }else if(input == "return")
        {
          goToDeviceSelect();
        }else{
          Serial.println(state);
          Serial.println(input);
          Serial.println(F("\t\tInvalid option"));
          state = STATE_DEV_OPTION;
        }
      }else
      {
        if(input == "return")
          state = STATE_DEV_OPTION;
        else{
          int int_input = input.toInt();
          boolean success = false;

          if(state == STATE_DEV_OPTION_CHANNEL && int_input > 0 && int_input <=7 && int_input != 6)
          {
            tmp_settings.channel = int_input;
            success = true;
          }else if(state == STATE_DEV_OPTION_PRF)
          {
            if(int_input == 16){
              tmp_settings.prf = 1;  success = true;
            }else if(int_input == 64){
              tmp_settings.prf = 2;  success = true;
            }
          }else if(state == STATE_DEV_OPTION_BITRATE)
          {
            if(int_input == 110){
              tmp_settings.bitrate = 0; success = true;
            }else if(int_input == 850){
              tmp_settings.bitrate = 1; success = true;
            }else if(int_input == 6810){
              tmp_settings.bitrate = 2; success = true;
            }
          }else if(state == STATE_DEV_OPTION_PLEN)
          {
            int j;
            for(j=0; j<8; j++)
            {
              if(int_input == plen_vals[j]){
                tmp_settings.plen = plens[j]; success = true;
              }
            }
          }else{
          }

          if(success)
            Serial.println(F("\t\tOption set"));
          else
            Serial.println(F("\t\tOption not set"));
          state = STATE_DEV_OPTION;
        }
      }

      input = "";
    }

  } // end: if (Serial.available() > 0)

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

void printChannelSettings(UWB_settings_t UWB_settings)
{
  Serial.print("   - bitrate: ");
  if(UWB_settings.bitrate == 0)
     Serial.print("110kbit/s");
  else if(UWB_settings.bitrate == 1)
     Serial.print("850kbit/s");
  else if(UWB_settings.bitrate == 2)
     Serial.print("6.81Mbit/s");

  Serial.print(" - PRF: ");
  if(UWB_settings.prf == 1)
    Serial.print("16MHz");
  else
    Serial.print("64MHz");

  Serial.print(" - plen: ");
  Serial.println(getPreambleLength(UWB_settings.plen));
}

void goToDeviceSelect()
{
  Serial.println(F("------------------------------------------------------------"));
  int i;
  for(i=0; i<devices_found; i++)
  {
      Serial.print(i+1);
      Serial.print(") 0x");
      Serial.print(devId[i], HEX);
      Serial.print(": channel ");
      Serial.print(devSettings[i].channel);
      printChannelSettings(devSettings[i]);
  }

  Serial.print(F("\nPlease select a device [1-"));
  Serial.print(devices_found);
  Serial.println("]: ");

  state = STATE_DEV_SELECTION;
}
