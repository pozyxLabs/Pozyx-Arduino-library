#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[10];// = {0x6018, 0x601f, 0x6048, 0x603e};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t heights[10];// = {2500, 2120, 1260, 1760};              // anchor z-coordinates in mm

void setup()
{
  int status;
  uint8_t listsize;
  
  Serial.begin(115200);

  if(Pozyx.begin(true) == POZYX_FAILURE)
  {
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

  Pozyx.clearDevices();

  Serial.println(F("\n----------POZYX DIAGNOSTICS----------"));

  Serial.println(F("ACTIVATING LEDS"));

  Pozyx.setLed(1, true);
  Pozyx.setLed(2, true);
  Pozyx.setLed(3, true);
  Pozyx.setLed(4, true);

  delay(1000);

  // UWB Settings
  Serial.println("");
  UWB_settings_t settings;
  Pozyx.getUWBSettings(&settings);

  Serial.println(F("UWB settings:"));
  Serial.print(F("Channel: "));
  Serial.println(settings.channel);
  Serial.print(F("Bitrate: "));
  Serial.println(settings.bitrate);
  Serial.print(F("PRF: "));
  Serial.println(settings.prf);
  Serial.print(F("Plen: "));
  Serial.println(settings.plen);
  Serial.print(F("Gain: "));
  Serial.println(settings.gain);
  Serial.print(F("Trim: "));
  Serial.println(settings.trim);
  
  // Calibration Status
  Serial.println();
  uint8_t calib_status;
  Pozyx.getCalibrationStatus(&calib_status);
  
  Serial.println(F("Calibration Status:"));
  Serial.print(F("MAG: "));
  if((calib_status & 0x03) == 0x03) Serial.println(F("ok"));
  else Serial.println(F("fail"));
  Serial.print(F("ACC: "));
  if((calib_status & 0x0C) == 0x0C) Serial.println(F("ok"));
  else Serial.println(F("fail"));
  Serial.print(F("GYR: "));
  if((calib_status & 0x30) == 0x30) Serial.println(F("ok"));
  else Serial.println(F("fail"));
  Serial.print(F("SYS: "));
  if((calib_status & 0xC0) == 0xC0) Serial.println(F("ok"));
  else Serial.println(F("fail"));
  
  // Sensor Mode
  Serial.println();
  uint8_t sensormode;
  Pozyx.getSensorMode(&sensormode);
  
  Serial.print(F("Sensor Mode: "));
  switch(sensormode)
  {
    case 0: Serial.println(F("MODE_OFF")); break;
    case 1: Serial.println(F("ACCONLY")); break;
    case 2: Serial.println(F("MAGONLY")); break;
    case 3: Serial.println(F("GYROONLY")); break;
    case 4: Serial.println(F("ACCMAGx")); break;
    case 5: Serial.println(F("ACCGYRO")); break;
    case 6: Serial.println(F("MAGGYRO")); break;
    case 7: Serial.println(F("AMG")); break;
    case 8: Serial.println(F("IMU")); break;
    case 9: Serial.println(F("COMPASS")); break;
    case 10: Serial.println(F("M4G")); break;
    case 11: Serial.println(F("NDOF_FMC_OFF")); break;
    case 12: Serial.println(F("NDOF")); break;
    default: Serial.println(F("ERROR"));
  }

  // Position Algorithm
  Serial.println();
  uint8_t posalg;
  Pozyx.getPositionAlgorithm(&posalg);
  
  Serial.print(F("Position Algorithm: "));
  if(posalg == POZYX_POS_ALG_UWB_ONLY) Serial.println(F("UWB-only"));
  else if(posalg == POZYX_POS_ALG_TRACKING) Serial.println(F("Tracking"));
  else if(posalg == POZYX_POS_ALG_LS) Serial.println(F("Least-Squares"));
  else Serial.println(posalg, BIN);
  
  // Position Dimension
  Serial.println();
  uint8_t posdim;
  Pozyx.getPositionDimension(&posdim);
  
  Serial.print(F("Position Dimension: "));
  if(posdim == POZYX_2D) Serial.println(F("2D"));
  else if(posdim == POZYX_2_5D) Serial.println(F("2,5D"));
  else if(posdim == POZYX_3D) Serial.println(F("3D"));
  else Serial.println(posdim, BIN);

  // Temperature
  Serial.println();
  float32_t temperature;
  Pozyx.getTemperature_c(&temperature);
  
  Serial.print(F("Temperature: "));
  Serial.println(temperature);

  // Get some Sensor Data
  Serial.println();
  sensor_data_t sensordata;
  uint8_t sdstatus = Pozyx.getAllSensorData(&sensordata);

  Serial.print(F("Sensor Data: "));
  Serial.println(sdstatus);
  Serial.print(F("\nPressure: "));
  Serial.println(sensordata.pressure);
  Serial.println(F("\nAcceleration:"));
  Serial.print(F("x: "));
  Serial.println(sensordata.acceleration.x);
  Serial.print(F("y: "));
  Serial.println(sensordata.acceleration.y);
  Serial.print(F("z: "));
  Serial.println(sensordata.acceleration.z);
  Serial.println(F("\nMagnetic:"));
  Serial.print(F("x: "));
  Serial.println(sensordata.magnetic.x);
  Serial.print(F("y: "));
  Serial.println(sensordata.magnetic.y);
  Serial.print(F("z: "));
  Serial.println(sensordata.magnetic.z);
  Serial.println(F("\nAngular_vel:"));
  Serial.print(F("x: "));
  Serial.println(sensordata.angular_vel.x);
  Serial.print(F("y: "));
  Serial.println(sensordata.angular_vel.y);
  Serial.print(F("z: "));
  Serial.println(sensordata.angular_vel.z);
  Serial.println(F("\nEuler_angles:"));
  Serial.print(F("heading: "));
  Serial.println(sensordata.euler_angles.heading);
  Serial.print(F("roll: "));
  Serial.println(sensordata.euler_angles.roll);
  Serial.print(F("pitch: "));
  Serial.println(sensordata.euler_angles.pitch);
  Serial.println(F("\nQuaternion:"));
  Serial.print(F("x: "));
  Serial.println(sensordata.quaternion.x);
  Serial.print(F("y: "));
  Serial.println(sensordata.quaternion.y);
  Serial.print(F("z: "));
  Serial.println(sensordata.quaternion.z);
  Serial.println(F("\nLinear_acceleration:"));
  Serial.print(F("x: "));
  Serial.println(sensordata.linear_acceleration.x);
  Serial.print(F("y: "));
  Serial.println(sensordata.linear_acceleration.y);
  Serial.print(F("z: "));
  Serial.println(sensordata.linear_acceleration.z);
  Serial.println(F("\nGravity_vector:"));
  Serial.print(F("x: "));
  Serial.println(sensordata.gravity_vector.x);
  Serial.print(F("y: "));
  Serial.println(sensordata.gravity_vector.y);
  Serial.print(F("z: "));
  Serial.println(sensordata.gravity_vector.z);
  Serial.print(F("\nTemperature: "));
  Serial.println(sensordata.temperature);
  


  
  /*status = Pozyx.doAnchorCalibration(POZYX_2_5D, 10, num_anchors, anchors, heights);
  if(status != POZYX_SUCCESS)
  {
    //Serial.println(status);
    Serial.println(F("ERROR: doAnchorCalibration"));
    Serial.println(Pozyx.getSystemError());
    delay(100);
    abort();
  }
  else
  {
    Serial.println(F("DONE: doAnchorCalibration"));
  }
*/

  Serial.println();
  status = Pozyx.doDiscovery(POZYX_DISCOVERY_ALL_DEVICES, 5, 10);
  if(status != POZYX_SUCCESS)
  {
    Serial.println(status);
    Serial.println(F("ERROR: doDiscovery"));
    Serial.println(Pozyx.getSystemError());
    delay(100);
    abort();
  }
  else
  {
    Serial.println(F("DONE: doDiscovery"));
  }
  
  status = Pozyx.getDeviceListSize(&listsize);
  if(status != POZYX_SUCCESS)
  {
    Serial.println(status);
    Serial.println(F("ERROR: getDeviceListSize"));
    Serial.println(Pozyx.getSystemError());
    delay(100);
    abort();
  }
  else
  {
    Serial.println(F("DONE: getDeviceListSize"));

    Serial.print(F("listsize: "));
    Serial.println(listsize);
  }
  
  // don't actually work as intended
  status = Pozyx.getTagIds(anchors, listsize);
  //status = Pozyx.getAnchorIds(anchors, listsize); // nope
  //status = Pozyx.getDeviceIds(anchors, listsize);
  if(status != POZYX_SUCCESS)
  {
    Serial.println(status);
    Serial.println(F("ERROR: getDeviceIds"));
    Serial.println(Pozyx.getSystemError());
    delay(100);
    abort();
  }
  else
  {
    Serial.println(F("DONE: getDeviceIds"));
    
    for(int i = 0; i < listsize; i++)
    {
      if(anchors[i] == 0) Serial.println(F("invalid id"));
      else Serial.println(anchors[i], HEX);
    }
  }
}

void loop()
{
  int8_t status;
  uint8_t listsize;

  status = Pozyx.doDiscovery(POZYX_DISCOVERY_ALL_DEVICES, 5, 10);
  if(status != POZYX_SUCCESS)
  {
    Serial.println(status);
    Serial.println(F("ERROR: doDiscovery"));
    Serial.println(Pozyx.getSystemError());
    delay(100);
    abort();
  }
  else
  {
    delay(200);
    status = Pozyx.getDeviceListSize(&listsize);
    if(status != POZYX_SUCCESS)
    {
      Serial.println(status);
      Serial.println(F("ERROR: getDeviceListSize"));
      Serial.println(Pozyx.getSystemError());
      delay(100);
      abort();
    }
    else
    {
      delay(200);
      status = Pozyx.getTagIds(anchors, listsize);
      //status = Pozyx.getAnchorIds(anchors, listsize); // nope
      //status = Pozyx.getDeviceIds(anchors, listsize);
      if(status != POZYX_SUCCESS)
      {
        Serial.println(status);
        Serial.println(F("ERROR: getXIds"));
        Serial.println(Pozyx.getSystemError());
        delay(100);
        abort();
      }
      else
      {
        Serial.println(F("ok"));
      }
    }
  }
  
  delay(500);
}
