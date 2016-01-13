/**
* Pozyx_lib.cpp
* -------------
* This file contains the defintion of the general POZYX functions
*
*/

#include "Pozyx.h"
#include <Wire.h>

extern "C" {
  #include "Pozyx_definitions.h"
}


/**
*
* STATUS REGISTERS
*
*/
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

int PozyxClass::getErrorCode(uint8_t *error_code, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_ERRORCODE, error_code, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_ERRORCODE, error_code, 1);
  }
}

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


/**
*
* CONFIGURATION REGISTERS
*
*/
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

int PozyxClass::getConfigModeGPIO(int gpio_num, uint8_t *mode, uint16_t remote_id){
  int status = POZYX_FAILURE;

  if(gpio_num >0 && gpio_num <=4)
  {
    uint8_t gpio_register = POZYX_CONFIG_GPIO1 + (gpio_num-1);

    if(remote_id == NULL){
      status = regRead(gpio_register, mode, 1);
    }
    else{
      status = remoteRegRead(remote_id, gpio_register, mode, 1);
    }
  }
  *mode &= 0x7;
  return status;
}

int PozyxClass::getConfigPullGPIO(int gpio_num, uint8_t *pull, uint16_t remote_id){
  int status = POZYX_FAILURE;

  if(gpio_num >0 && gpio_num <=4)
  {
    uint8_t gpio_register = POZYX_CONFIG_GPIO1 + (gpio_num-1);

    if(remote_id == NULL){
      status = regRead(gpio_register, pull, 1);
    }
    else{
      status = remoteRegRead(remote_id, gpio_register, pull, 1);
    }
  }
  *pull = (*pull & 0x18) >> 3;
  return status;
}

int PozyxClass::setConfigGPIO(int gpio_num, int mode, int pull, uint16_t remote_id)
{

  int status = POZYX_FAILURE;

  if(gpio_num >0 && gpio_num <=4)
  {
    uint8_t gpio_register = POZYX_CONFIG_GPIO1 + (gpio_num-1);
    uint8_t mask = mode + (pull << 3);

    if(remote_id == NULL){
      status = regWrite(gpio_register, &mask, 1);
      delay(POZYX_DELAY_LOCAL_WRITE);
    }
    else{
      status = remoteRegWrite(remote_id, gpio_register, &mask, 1);
      delay(POZYX_DELAY_REMOTE_WRITE);
    }
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

int PozyxClass::getPositionAlgorithm(uint8_t *algorithm, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_POS_ALG, algorithm, 1);
  }
  else{
    status = remoteRegRead(remote_id, POZYX_POS_ALG, algorithm, 1);
  }
  *algorithm &= 0xF;
  return status;
}

int PozyxClass::getPositionDimension(uint8_t *dimension, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_POS_ALG, dimension, 1);
  }
  else{
    status = remoteRegRead(remote_id, POZYX_POS_ALG, dimension, 1);
  }
  *dimension = (*dimension & 0x30) >> 4;;
  return status;
}

int PozyxClass::setPositionAlgorithm(int algorithm, int dimension, uint16_t remote_id)
{
  int status;

  uint8_t params = algorithm + (dimension << 4);

  if(remote_id == NULL){
    status = regWrite(POZYX_POS_ALG, &params, 1);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_POS_ALG, &params, 1);
    delay(POZYX_DELAY_REMOTE_WRITE);
  }
  return status;
}

int PozyxClass::getAnchorMode(uint8_t *mode, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_POS_NUM_ANCHORS, mode, 1);
  }
  else{
    status = remoteRegRead(remote_id, POZYX_POS_NUM_ANCHORS, mode, 1);
  }
  *mode = (*mode & 0x80 >> 7);
  return status;
}

int PozyxClass::getNumberOfAnchors(uint8_t *nr_anchors, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_POS_NUM_ANCHORS, nr_anchors, 1);
  }
  else{
    status = remoteRegRead(remote_id, POZYX_POS_NUM_ANCHORS, nr_anchors, 1);
  }
  *nr_anchors &= 0xF;
  return status;
}

int PozyxClass::setSelectionOfAnchors(int mode, int nr_anchors, uint16_t remote_id)
{
  int status;

  uint8_t params = (mode << 7) + nr_anchors;

  if(remote_id == NULL){
    status = regWrite(POZYX_POS_NUM_ANCHORS, &params, 1);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_POS_NUM_ANCHORS, &params, 1);
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

int PozyxClass::getOperationMode(uint8_t *mode, uint16_t remote_id)
{
  if(remote_id == NULL){
    return regRead(POZYX_OPERATION_MODE, (uint8_t *) mode, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_OPERATION_MODE, (uint8_t *) mode, 1);
  }
}

int PozyxClass::setOperationMode(uint8_t mode, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regWrite(POZYX_OPERATION_MODE, (uint8_t *) &mode, 1);
    delay(POZYX_DELAY_LOCAL_WRITE);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_OPERATION_MODE, (uint8_t *) &mode, 1);
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

/**
*
* POSITION DATA
*
*/
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
   int status = POZYX_FAILURE;

  if(gpio_num >0 && gpio_num <=4)
  {
    uint8_t gpio_register = POZYX_GPIO1 + (gpio_num-1);
    if(remote_id == NULL){
      return regRead(gpio_register, (uint8_t *) value, 1);
    }
    else{
      return remoteRegRead(remote_id, gpio_register, (uint8_t *) value, 1);
    }
  }
  return status;
}

int PozyxClass::setGPIO(int gpio_num, uint8_t value, uint16_t remote_id)
{
  int status = POZYX_FAILURE;

  if(gpio_num >0 && gpio_num <=4)
  {
      uint8_t gpio_register = POZYX_GPIO1 + (gpio_num-1);

    if(remote_id == NULL){
      status = regWrite(gpio_register, &value, 1);
      delay(POZYX_DELAY_LOCAL_WRITE);
    }
    else{
      status = remoteRegWrite(remote_id, gpio_register, &value, 1);
      delay(POZYX_DELAY_REMOTE_WRITE);
    }
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
  int status = POZYX_FAILURE;  

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

int PozyxClass::doRanging(uint16_t destination, device_range_t *range){
  int status;

  status = regFunction(POZYX_DO_RANGING, (uint8_t *) &destination, 2, NULL, 0);  
  if (status == POZYX_SUCCESS && waitForFlag(POZYX_INT_STATUS_FUNC, POZYX_DELAY_INTERRUPT)){
    status = getDeviceRangeInfo(destination, range);
    return status;
  }
  else{
    return POZYX_TIMEOUT;
  }
  return status;
}

int PozyxClass::doRemoteRanging(uint16_t device_from, uint16_t device_to, device_range_t *device_range)
{
  int status;

  status = remoteRegFunction(device_from, POZYX_DO_RANGING, (uint8_t *) device_to, 2, NULL, 0); 

  if (status == POZYX_SUCCESS && waitForFlag(POZYX_INT_STATUS_RX_DATA , POZYX_DELAY_INTERRUPT)){
    status = readRXBufferData((uint8_t *) device_range, sizeof(device_range_t));
    return status;
  }
  else{
    return POZYX_TIMEOUT;
  }
  return status;
}

int PozyxClass::doPositioning(coordinates_t *position)
{
  int status;
  
  status = regFunction(POZYX_DO_POSITIONING, NULL, 0, NULL, 0); 
  
  if (status == POZYX_SUCCESS && waitForFlag(POZYX_INT_STATUS_POS, POZYX_DELAY_INTERRUPT)){
    status = getCoordinates(position);
    return status;
  }
  else{
    return POZYX_TIMEOUT;
  }
  return status;
}

int PozyxClass::doRemotePositioning(uint16_t remote_id, coordinates_t *coordinates){
  int status;

  status = remoteRegFunction(remote_id, POZYX_DO_POSITIONING, NULL, 0, NULL, 0); 

  if (status == POZYX_SUCCESS && waitForFlag(POZYX_INT_STATUS_RX_DATA , POZYX_DELAY_INTERRUPT)){
    status = readRXBufferData((uint8_t *) coordinates, sizeof(coordinates_t));
    return status;
  }
  else{
    return POZYX_TIMEOUT;
  }
  return status;
}



int PozyxClass::setPositioningAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id)
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


int PozyxClass::getPositioningAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id)
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

int PozyxClass::getDeviceIds(uint16_t devices[],int size, uint16_t remote_id)
{
  int status;

  if(remote_id == NULL){
    status = regFunction(POZYX_DEVICES_GETIDS, NULL, 0, (uint8_t *) devices, size * 2); 
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_DEVICES_GETIDS, NULL, 0, (uint8_t *) devices, MAX_ANCHORS_IN_LIST * 2);  
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }
  return status;
}

int PozyxClass::getAnchorIds(uint16_t anchors[],int size, uint16_t remote_id)
{
  int status;
  uint16_t devices[size];

  if(remote_id == NULL){
    status = regFunction(POZYX_DEVICES_GETIDS, NULL, 0, (uint8_t *) devices, size * 2); 
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_DEVICES_GETIDS, NULL, 0, (uint8_t *) devices, MAX_ANCHORS_IN_LIST * 2);  
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }

  if(status == POZYX_SUCCESS){
    for (int i=0; i < size ; i++){
      anchors[i] = 0x0;
    }
    int j = 0;
    for (int i=0; i < size ; i++){
      uint8_t mode = 0x0;
      status &= getOperationMode(&mode, devices[i]);
      if(mode == POZYX_ANCHOR_MODE){
        anchors[j] = devices[i];
        j++;
      }
    }
  }
  return status;
}

int PozyxClass::getTagIds(uint16_t tags[],int size, uint16_t remote_id)
{
  int status;
  uint16_t devices[size];

  if(remote_id == NULL){
    status = regFunction(POZYX_DEVICES_GETIDS, NULL, 0, (uint8_t *) devices, size * 2); 
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_DEVICES_GETIDS, NULL, 0, (uint8_t *) devices, MAX_ANCHORS_IN_LIST * 2);  
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }

  if(status == POZYX_SUCCESS){
    for (int i=0; i < size ; i++){
      tags[i] = 0x0;
    }
    int j = 0;
    for (int i=0; i < size ; i++){
      uint8_t mode = 0x0;
      status &= getOperationMode(&mode, devices[i]);
      if(mode == POZYX_TAG_MODE){
        tags[j] = devices[i];
        j++;
      }
    }
  }
  return status;
}

int PozyxClass::doDiscovery(int type, int slots, int slot_duration)
{
  int status;
  uint8_t params[3];

  params[0] = (uint8_t)type;
  params[1] = (uint8_t)slots;
  params[2] = (uint8_t)slot_duration;


  status = regFunction(POZYX_DEVICES_DISCOVER, (uint8_t *)&params, 3, NULL, 0);
  delay(POZYX_DELAY_LOCAL_FUNCTION);
  if (status == POZYX_SUCCESS && waitForFlag(POZYX_INT_STATUS_FUNC, POZYX_DELAY_INTERRUPT)){
    return status;
  }
  else{
    return POZYX_TIMEOUT;
  }
  return status;
}

int PozyxClass::doAnchorCalibration(int option, uint16_t anchors[], int anchor_num, int measurements)
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
  delay(POZYX_DELAY_LOCAL_FUNCTION);
  if (status == POZYX_SUCCESS && waitForFlag(POZYX_INT_STATUS_FUNC, POZYX_DELAY_CALIBRATION * measurements)){
    return status;
  }
  else{
    return POZYX_FAILURE;
  }
  return status;
}

int PozyxClass::updateRemoteTags(uint16_t tags[], int tags_num)
{
  int status = POZYX_SUCCESS;

  uint16_t internal_tags[MAX_ANCHORS_IN_LIST];

  if( tags == NULL){
    status = getTagIds(internal_tags);
    tags_num = MAX_ANCHORS_IN_LIST;
  }
  else{
    memcpy(internal_tags, (uint8_t*) tags, tags_num*2);
  }

  uint8_t algorithm = 0x0;
  uint8_t selection_of_anchors = 0x0;
  uint16_t anchors[MAX_ANCHORS_IN_LIST];
  uint16_t position_anchors[MAX_ANCHORS_IN_LIST];
  device_coordinates_t anchor_coor[MAX_ANCHORS_IN_LIST];

  status &= regRead(POZYX_POS_ALG, &algorithm, 1);
  status &= regRead(POZYX_POS_NUM_ANCHORS, &selection_of_anchors, 1);
  status &= getAnchorIds(anchors);
  status &= getPositioningAnchorIds(position_anchors, MAX_ANCHORS_IN_LIST); 

  for (int i=0 ; i < MAX_ANCHORS_IN_LIST; i++){
    getDeviceCoordinates(anchors[i], &anchor_coor[i]);
  }

  if (status == POZYX_SUCCESS){
    for(int i=0; i < tags_num; i++){
      status &= clearDevices(internal_tags[i]);

      for (int i=0 ; i < MAX_ANCHORS_IN_LIST; i++){
        addDevice(anchor_coor[i], internal_tags[i]);
      }

      status &= setPositionAlgorithm(algorithm, internal_tags[i]);
      status &= setSelectionOfAnchors(selection_of_anchors, internal_tags[i]);

      status &= setPositioningAnchorIds(position_anchors, internal_tags[i]);
    }
  }
  return status;  
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
