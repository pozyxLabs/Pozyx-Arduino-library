/**
  Pozyx.cpp - Library for Arduino Pozyx shield.
  Copyright (c) Pozyx Laboratories.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Pozyx.h"
#include <Wire.h>

extern "C" {
  #include "Pozyx_definitions.h"
}


// FUNCTIONS

int PozyxClass::getWhoAmI(uint8_t *whoami , uint16_t remote_id)
{
  if (remote_id == NULL){
    return regRead(POZYX_WHO_AM_I, whoami, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_WHO_AM_I, whoami, 1);
  }
}

int PozyxClass::getFirmwareVersion(uint8_t *firmware, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_FIRMWARE_VER, firmware, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_FIRMWARE_VER, firmware, 1);
  }
}

int PozyxClass::getHardwareVersion(uint8_t *hardware, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_HARDWARE_VER, hardware, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_HARDWARE_VER, hardware, 1);
  }
}

int PozyxClass::getSelftest(uint8_t *selftest, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_ST_RESULT, selftest, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_ST_RESULT, selftest, 1);
  }
}
  /*
  static int getStatus();
  static int getErrorCode();
  */

int PozyxClass::getInterruptStatus(uint8_t *interrupts, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_INT_STATUS, interrupts, 1); 
  }
  else{
    return remoteRegRead(remote_id, POZYX_INT_STATUS, interrupts, 1); 
  }  
}

int PozyxClass::getCalibrationStatus(uint8_t *calibration_status, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_CALIB_STATUS, calibration_status, 1); 
  }
  else{
    return remoteRegRead(remote_id, POZYX_CALIB_STATUS, calibration_status, 1); 
  }
}

int PozyxClass::getInterruptMask(uint8_t *mask, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_INT_MASK, mask, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_INT_MASK, mask, 1);
  }
}
int PozyxClass::setInterruptMask(uint8_t mask, uint16_t remote_id)
{
  int status;
  if(remote_id == NULL){
    status = regWrite(POZYX_INT_MASK, &mask, 1);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_INT_MASK, &mask, 1);
    delay(POZYX_DELAY_REMOTE_WRITE);
  }

  return status;
}

int PozyxClass::getUpdateInterval(uint16_t *ms, uint16_t remote_id)
{

  if(remote_id == NULL){
    return regRead(POZYX_POS_INTERVAL, (uint8_t *) ms, 2);
  }
  else{
    return remoteRegRead(remote_id, POZYX_POS_INTERVAL, (uint8_t *) ms, 2);
  }
}

int PozyxClass::setUpdateInterval(uint16_t ms, uint16_t remote_id)
{ 
  int status;
  
  if(remote_id == NULL){
    status = regWrite(POZYX_POS_INTERVAL, (uint8_t *) &ms, 2);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_POS_INTERVAL, (uint8_t *) &ms, 2);
    delay(POZYX_DELAY_REMOTE_WRITE);
  }

  return status;
}

int PozyxClass::getConfigGPIO(int gpio_num, uint8_t *config, uint16_t remote_id){
  int gpio_register;
  
  switch(gpio_num){
      case 1  :
         gpio_register = POZYX_CONFIG_GPIO1;
         break; 
      case 2  :
         gpio_register = POZYX_CONFIG_GPIO2;
         break; 
      case 3  :
         gpio_register = POZYX_CONFIG_GPIO3;
         break;
      case 4  :
         gpio_register = POZYX_CONFIG_GPIO4;
         break;  
      default : 
         return -1;
  }

  if(remote_id == NULL){
    return regRead(gpio_register, config, 1);
  }
  else{
    return remoteRegRead(remote_id, gpio_register, config, 1);
  }
}

int PozyxClass::convertGPIOMode(int config)
{
  return (config & 0b00000111);
}

int PozyxClass::convertGPIOPull(int config)
{
  return ((config & 0b00011000) >> 3);
}

int PozyxClass::setConfigGPIO(int gpio_num, int mode, int pull, uint16_t remote_id)
{
  int gpio_register;
  int status;
  uint8_t mask;
  
  switch(gpio_num){
      case 1  :
         gpio_register = POZYX_CONFIG_GPIO1;
         break; 
      case 2  :
         gpio_register = POZYX_CONFIG_GPIO2;
         break; 
      case 3  :
         gpio_register = POZYX_CONFIG_GPIO3;
         break;
      case 4  :
         gpio_register = POZYX_CONFIG_GPIO4;
         break;  
      default : 
         return -1;
  }

  mask = mode + (pull << 3);

  if(remote_id == NULL){
    status = regWrite(gpio_register, &mask, 1);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, gpio_register, &mask, 1);
    delay(POZYX_DELAY_REMOTE_WRITE);
  }

  return status;
}

int PozyxClass::getNumberOfAnchors(uint8_t *nr_anchors, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_POS_NUM_ANCHORS, nr_anchors, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_POS_NUM_ANCHORS, nr_anchors, 1);
  }
}

int PozyxClass::convertAnchorMode(int nr_anchors)
{
  return ((nr_anchors & 0b10000000) >> 7);
}

int PozyxClass::convertAnchorNumber(int nr_anchors)
{
  return (nr_anchors & 0b00001111);
}

int PozyxClass::setNumberOfAnchors(uint8_t nr_anchors, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regWrite(POZYX_POS_NUM_ANCHORS, &nr_anchors, 1);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_POS_NUM_ANCHORS, &nr_anchors, 1);
    delay(POZYX_DELAY_REMOTE_WRITE);
  }

  return status;
}

int PozyxClass::getNetworkId(uint16_t *network_id)
{
  return regRead(POZYX_NETWORK_ID, (uint8_t *) network_id, 2);
}

int PozyxClass::setNetworkId(uint16_t network_id, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regWrite(POZYX_NETWORK_ID, (uint8_t *) &network_id, 2);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_NETWORK_ID, (uint8_t *) &network_id, 2);
    delay(POZYX_DELAY_REMOTE_WRITE);
  }

  return status;
}


int PozyxClass::getUWBSettings(UWB_settings_t *UWB_settings, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_UWB_CHANNEL, (uint8_t *) UWB_settings, sizeof(UWB_settings_t));
  }
  else{
    return remoteRegRead(remote_id, POZYX_UWB_CHANNEL, (uint8_t *) UWB_settings, sizeof(UWB_settings_t));
  }
}

int PozyxClass::setUWBSettings(UWB_settings_t UWB_settings, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regWrite(POZYX_UWB_CHANNEL, (uint8_t *) &UWB_settings, sizeof(UWB_settings_t));
    delay(2 * POZYX_DELAY_LOCAL_WRITE);
    if (status == POZYX_FAILURE){
      return status;
    }
    status = regWrite(POZYX_UWB_GAIN, (uint8_t *) &(UWB_settings.gain), 1);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_UWB_CHANNEL, (uint8_t *) &UWB_settings, sizeof(UWB_settings_t));
    delay(2 * POZYX_DELAY_REMOTE_WRITE);
    if (status == POZYX_FAILURE){
      return status;
    }
    status = remoteRegWrite(remote_id, POZYX_UWB_GAIN, (uint8_t *) &(UWB_settings.gain), 1);
    delay(POZYX_DELAY_REMOTE_WRITE);
  }

  return status;  
}

int PozyxClass::getSensorMode(uint8_t *sensor_mode, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_SENSORS_MODE, (uint8_t *) sensor_mode, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_SENSORS_MODE, (uint8_t *) sensor_mode, 1);
  }
}

int PozyxClass::setSensorMode(uint8_t sensor_mode, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regWrite(POZYX_SENSORS_MODE, (uint8_t *) &sensor_mode, 1);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_SENSORS_MODE, (uint8_t *) &sensor_mode, 1);
    delay(POZYX_DELAY_REMOTE_WRITE);
  }

  return status;
}


int PozyxClass::getCoordinates(coordinates_t *coordinates, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_POS_X, (uint8_t *) coordinates, sizeof(coordinates_t));
  }
  else{
    return remoteRegRead(remote_id, POZYX_POS_X, (uint8_t *) coordinates, sizeof(coordinates_t));
  }
}

int PozyxClass::setCoordinates(coordinates_t coordinates, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regWrite(POZYX_POS_X, (uint8_t *) &coordinates, sizeof(coordinates_t));
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_POS_X, (uint8_t *) &coordinates, sizeof(coordinates_t));
    delay(POZYX_DELAY_REMOTE_WRITE);
  }

  return status;
}

int PozyxClass::getPositionError(pos_error_t *pos_error, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_POS_ERR_X, (uint8_t *) pos_error, sizeof(pos_error_t));
  }
  else{
    return remoteRegRead(remote_id, POZYX_POS_ERR_X, (uint8_t *) pos_error, sizeof(pos_error_t));
  }
}


/**
*
* SENSOR DATA
*
*/
int PozyxClass::getSensorData(sensor_data_t *sensor_data, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_PRESSURE, (uint8_t *) sensor_data, sizeof(sensor_data_t));
  }
  else{
    return remoteRegRead(remote_id, POZYX_PRESSURE, (uint8_t *) sensor_data, sizeof(sensor_data_t));
  }
}

int PozyxClass::getPressure(uint32_t *pressure, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_PRESSURE, (uint8_t *) pressure, 4); 
  }
  else{
    return remoteRegRead(remote_id, POZYX_PRESSURE, (uint8_t *) pressure, 4); 
  }
}

int PozyxClass::getAcceleration(acceleration_t *acceleration, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_ACCEL_X, (uint8_t *) acceleration, sizeof(acceleration_t));
  }
  else{
    return remoteRegRead(remote_id, POZYX_ACCEL_X, (uint8_t *) acceleration, sizeof(acceleration_t));
  } 
}

int PozyxClass::getMagnetic(magnetic_t *magnetic, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_MAGN_X, (uint8_t *) magnetic, sizeof(magnetic_t)); 
  }
  else{
    return remoteRegRead(remote_id, POZYX_MAGN_X, (uint8_t *) magnetic, sizeof(magnetic_t)); 
  }
}

int PozyxClass::getGyro(gyro_t *gyro, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_GYRO_X, (uint8_t *) gyro, sizeof(gyro_t)); 
  }
  else{
    return remoteRegRead(remote_id, POZYX_GYRO_X, (uint8_t *) gyro, sizeof(gyro_t)); 
  }
}

int PozyxClass::getEulerAngles(euler_angles_t *euler_angles, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_EUL_HEADING, (uint8_t *) euler_angles, sizeof(euler_angles_t)); 
  }
  else{
    return remoteRegRead(remote_id, POZYX_EUL_HEADING, (uint8_t *) euler_angles, sizeof(euler_angles_t));
  }
}

int PozyxClass::getQuaternion(quaternion_t *quaternion, uint16_t remote_id)
{ 
  if(remote_id == NULL){
    return regRead(POZYX_QUAT_W, (uint8_t *) quaternion, sizeof(quaternion_t)); 
  }
  else{
    return remoteRegRead(remote_id, POZYX_QUAT_W, (uint8_t *) quaternion, sizeof(quaternion_t)); 
  }
}

int PozyxClass::getLinearAcceleration(linear_acceleration_t *linear_acceleration, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_LIA_X, (uint8_t *) linear_acceleration, sizeof(linear_acceleration_t));
  }
  else{
    return remoteRegRead(remote_id, POZYX_LIA_X, (uint8_t *) linear_acceleration, sizeof(linear_acceleration_t));
  }
}

int PozyxClass::getGravityVector(gravity_vector_t *gravity_vector, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_GRAV_X, (uint8_t *) gravity_vector, sizeof(gravity_vector_t)); 
  }
  else{
    return remoteRegRead(remote_id, POZYX_GRAV_X, (uint8_t *) gravity_vector, sizeof(gravity_vector_t));
  }
}

int PozyxClass::getTemperature(int8_t *temperature, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_TEMPERATURE, (uint8_t *) temperature, 1); 
  }
  else{
    return remoteRegRead(remote_id, POZYX_TEMPERATURE, (uint8_t *) temperature, 1); 
  }
}

/**
*
* GENERAL DATA
*
*/

int PozyxClass::getDeviceListSize(uint8_t *device_list_size, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_DEVICE_LIST_SIZE, (uint8_t *) device_list_size, 1);  
  }
  else{
    return remoteRegRead(remote_id, POZYX_DEVICE_LIST_SIZE, (uint8_t *) device_list_size, 1);
  }
}

int PozyxClass::getLastNetworkId(uint16_t *network_id, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_RX_NETWORK_ID, (uint8_t *) network_id, 2);
  }
  else{
    return remoteRegRead(remote_id, POZYX_RX_NETWORK_ID, (uint8_t *) network_id, 2);
  } 
}

int PozyxClass::getLastDataLength(uint8_t *data_length, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_RX_DATA_LEN, (uint8_t *) data_length, 1);  
  }
  else{
    return remoteRegRead(remote_id, POZYX_RX_DATA_LEN, (uint8_t *) data_length, 1); 
  }
}

int PozyxClass::getGPIO(int gpio_num, uint8_t *value, uint16_t remote_id)
{
  int gpio_register;
  
  switch(gpio_num){
      case 1  :
         gpio_register = POZYX_GPIO1;
         break; 
      case 2  :
         gpio_register = POZYX_GPIO2;
         break; 
      case 3  :
         gpio_register = POZYX_GPIO3;
         break;
      case 4  :
         gpio_register = POZYX_GPIO4;
         break;  
      default : 
         return -1;
  }
  if(remote_id == NULL){
    return regRead(gpio_register, (uint8_t *) value, 1);
  }
  else{
    return remoteRegRead(remote_id, gpio_register, (uint8_t *) value, 1);
  }
}

int PozyxClass::setGPIO(int gpio_num, uint8_t value, uint16_t remote_id)
{
  int gpio_register;
  int status;
  
  switch(gpio_num){
      case 1  :
         gpio_register = POZYX_GPIO1;
         break; 
      case 2  :
         gpio_register = POZYX_GPIO2;
         break; 
      case 3  :
         gpio_register = POZYX_GPIO3;
         break;
      case 4  :
         gpio_register = POZYX_GPIO4;
         break;  
      default : 
         return -1;
  }

  if(remote_id == NULL){
    status = regWrite(gpio_register, &value, 1);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, gpio_register, &value, 1);
    delay(POZYX_DELAY_REMOTE_WRITE);
  }
  return status;
}

int PozyxClass::setLedConfig(uint8_t config, uint16_t remote_id)
{
  int status;
  if(remote_id == NULL){
    status = regWrite(POZYX_CONFIG_LEDS, &config, 1);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_CONFIG_LEDS, &config, 1);
    delay(POZYX_DELAY_REMOTE_WRITE);
  }

  return status;

}


/**
    *
    * GENERAL FUNCTIONS
    *
    */


void PozyxClass::resetSystem(uint16_t remote_id)
{
  if(remote_id == NULL){
    regFunction(POZYX_RESET_SYS, NULL, 0, NULL, 0); 
  }
  else{
    remoteRegFunction(remote_id, POZYX_RESET_SYS, NULL, 0, NULL, 0); 
  }
  
}
int PozyxClass::setLed(int led_num, boolean state, uint16_t remote_id)
{
  int status = -1;  

  if(led_num >0 && led_num <=4)
  {
    // the 4 MSB indicate which led we wish to control, the 4 LSB indicate the state of the leds
    uint8_t params = (0x1 << (led_num-1+4)) | (((uint8_t)state) << (led_num-1));  
    
    if(remote_id == NULL){
      status = regFunction(POZYX_LED_CTRL, &params, 1, NULL, 0);
      delay(POZYX_DELAY_LOCAL_FUNCTION);
    }
    else{
      status = remoteRegFunction(remote_id, POZYX_LED_CTRL, &params, 1, NULL, 0);
      delay(POZYX_DELAY_REMOTE_FUNCTION);
    }
  }
  return status; 
}

int PozyxClass::sendTXBufferData(uint16_t destination)
{
  int status;

  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x06;    
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
  delay(POZYX_DELAY_LOCAL_FUNCTION);

  return status;
}

int PozyxClass::doRanging(uint16_t destination)
{
  int status;

  status = regFunction(POZYX_DO_RANGING, (uint8_t *) &destination, 2, NULL, 0);  
  if (waitForFlag(POZYX_INT_STATUS_FUNC, POZYX_DELAY_INTERRUPT)){
    return status;
  }
  else{
    return POZYX_FAILURE;
  }
}

int PozyxClass::doRemoteRanging(uint16_t device_from, uint16_t device_to, device_range_t *device_range)
{
  int status;

  status = remoteRegFunction(device_from, POZYX_DO_RANGING, (uint8_t *) device_to, 2, NULL, 0); 

  if (waitForFlag(POZYX_INT_STATUS_RX_DATA , POZYX_DELAY_INTERRUPT)){
    status = readRXBufferData((uint8_t *) device_range, sizeof(device_range_t));
    return status;
  }
  else{
    return POZYX_FAILURE;
  }
}

int PozyxClass::doPositioning()
{
  int status;
  
  status = regFunction(POZYX_DO_POSITIONING, NULL, 0, NULL, 0); 
  
  if (waitForFlag(POZYX_INT_STATUS_POS, POZYX_DELAY_INTERRUPT)){
    return status;
  }
  else{
    return POZYX_FAILURE;
  }
}

int PozyxClass::doRemotePositioning(uint16_t remote_id, coordinates_t *coordinates){
  int status;

  status = remoteRegFunction(remote_id, POZYX_DO_POSITIONING, NULL, 0, NULL, 0); 

  if (waitForFlag(POZYX_INT_STATUS_RX_DATA , POZYX_DELAY_INTERRUPT)){
    status = readRXBufferData((uint8_t *) coordinates, sizeof(coordinates_t));
    return status;
  }
  else{
    return POZYX_FAILURE;
  }
}



int PozyxClass::setAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regFunction(POZYX_POS_SET_ANCHOR_IDS, (uint8_t *) anchors, anchor_num * 2, NULL, 0); 
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_POS_SET_ANCHOR_IDS, (uint8_t *) anchors, anchor_num * 2, NULL, 0); 
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }
  return status;
}


int PozyxClass::getAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id)
{

  int status;
  uint8_t device_list_size = 0;

  if(remote_id == NULL){
    getDeviceListSize(&device_list_size);
    if (anchor_num < device_list_size){
      return POZYX_FAILURE;
    }

    status = regFunction(POZYX_POS_GET_ANCHOR_IDS, NULL, 0, (uint8_t *) anchors, anchor_num * 2); 
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    getDeviceListSize(&device_list_size, remote_id);
    delay(POZYX_DELAY_REMOTE_FUNCTION);
    if (anchor_num < device_list_size){
      return POZYX_FAILURE;
    }
    status = remoteRegFunction(remote_id, POZYX_POS_GET_ANCHOR_IDS, NULL, 0, (uint8_t *) anchors, anchor_num * 2); 
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }
  return status;
}

int PozyxClass::getDeviceIds(uint16_t anchors[],int size, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regFunction(POZYX_DEVICES_GETIDS, NULL, 0, (uint8_t *) anchors, size * 2); 
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_DEVICES_GETIDS, NULL, 0, (uint8_t *) anchors, MAX_ANCHORS_IN_LIST * 2);  
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }
  return status;
}

int PozyxClass::doDiscovery(int slots, int slot_duration)
{
  int status;
  uint8_t params[2];

  params[0] = (uint8_t)slots;
  params[1] = (uint8_t)slot_duration;


  status = regFunction(POZYX_DEVICES_DISCOVER, (uint8_t *)&params, 2, NULL, 0);

  // TODO WAIT FOR FUNCTION FLAG
  if (waitForFlag(POZYX_INT_STATUS_FUNC, POZYX_DELAY_INTERRUPT)){
    return status;
  }
  else{
    return POZYX_FAILURE;
  }
}

int PozyxClass::doAnchorCalibration(int option, int measurements, uint16_t anchors[], int anchor_num)
{
  int status;

  if (anchor_num < 0 || anchor_num > 6){
    return POZYX_FAILURE;
  }


  uint8_t params[2 + anchor_num * sizeof(uint16_t)];
  params[0] = (uint8_t)option;
  params[1] = (uint8_t)measurements;

  if (anchor_num > 0){
    memcpy(params+2, (uint8_t*) anchors, anchor_num * sizeof(uint16_t));
  }


  status = regFunction(POZYX_DEVICES_CALIBRATE, (uint8_t *)&params, 2 + anchor_num * sizeof(uint16_t), NULL, 0);

  // TODO WAIT FOR FUNCTION FLAG
  if (status == POZYX_SUCCESS && waitForFlag(POZYX_INT_STATUS_FUNC, POZYX_DELAY_CALIBRATION * measurements)){
    return status;
  }
  else{
    return POZYX_FAILURE;
  }
}


int PozyxClass::clearDevices(uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regFunction(POZYX_DEVICES_CLEAR, NULL, 0, NULL, 0); 
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_DEVICES_CLEAR, NULL, 0, NULL, 0); 
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }
  return status;
}

int PozyxClass::addDevice(device_coordinates_t device_coordinates, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regFunction(POZYX_DEVICE_ADD, (uint8_t *) &device_coordinates, sizeof(device_coordinates_t), NULL, 0); 
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_DEVICES_CLEAR, NULL, 0, NULL, 0); 
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }
  return status;
}

int PozyxClass::getDeviceInfo(uint16_t device_id, device_info_t *device_info, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regFunction(POZYX_DEVICE_GETINFO, (uint8_t *) &device_id, 1, (uint8_t *) device_info, sizeof(device_info_t));
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_DEVICE_GETINFO, (uint8_t *) &device_id, 1, (uint8_t *) device_info, sizeof(device_info_t));
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }
  return status;
}

int PozyxClass::getDeviceCoordinates(uint16_t device_id, device_coordinates_t *device_coordinates, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regFunction(POZYX_DEVICE_GETCOORDS, (uint8_t *) &device_id, 1, (uint8_t *) device_coordinates, sizeof(device_coordinates_t));
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_DEVICE_GETCOORDS, (uint8_t *) &device_id, 1, (uint8_t *) device_coordinates, sizeof(device_coordinates_t));
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }
  return status;
}

int PozyxClass::getDeviceRangeInfo(uint16_t device_id, device_range_t *device_range, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regFunction(POZYX_DEVICE_GETRANGEINFO, (uint8_t *) &device_id, 2, (uint8_t *) device_range, sizeof(device_range_t));
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_DEVICE_GETRANGEINFO, (uint8_t *) &device_id, 2, (uint8_t *) device_range, sizeof(device_range_t));
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }
  return status;
}
