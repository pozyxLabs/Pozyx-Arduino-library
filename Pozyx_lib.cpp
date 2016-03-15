/*
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

int PozyxClass::getWhoAmI(uint8_t *whoami , uint16_t remote_id)
{
  assert(whoami != NULL);

  if (remote_id == NULL){
    return regRead(POZYX_WHO_AM_I, whoami, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_WHO_AM_I, whoami, 1);
  }
}

int PozyxClass::getFirmwareVersion(uint8_t *firmware, uint16_t remote_id)
{
  assert(firmware != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_FIRMWARE_VER, firmware, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_FIRMWARE_VER, firmware, 1);
  }
}

int PozyxClass::getHardwareVersion(uint8_t *hardware, uint16_t remote_id)
{
  assert(hardware != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_HARDWARE_VER, hardware, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_HARDWARE_VER, hardware, 1);
  }
}

int PozyxClass::getSelftest(uint8_t *selftest, uint16_t remote_id)
{
  assert(selftest != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_ST_RESULT, selftest, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_ST_RESULT, selftest, 1);
  }
}

int PozyxClass::getErrorCode(uint8_t *error_code, uint16_t remote_id)
{
  assert(error_code != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_ERRORCODE, error_code, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_ERRORCODE, error_code, 1);
  }
}

int PozyxClass::getInterruptStatus(uint8_t *interrupts, uint16_t remote_id)
{
  assert(interrupts != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_INT_STATUS, interrupts, 1); 
  }
  else{
    return remoteRegRead(remote_id, POZYX_INT_STATUS, interrupts, 1); 
  }  
}

int PozyxClass::getCalibrationStatus(uint8_t *calibration_status, uint16_t remote_id)
{
  assert(calibration_status != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_CALIB_STATUS, calibration_status, 1); 
  }
  else{
    return remoteRegRead(remote_id, POZYX_CALIB_STATUS, calibration_status, 1); 
  }
}

int PozyxClass::getInterruptMask(uint8_t *mask, uint16_t remote_id)
{
  assert(mask != NULL);

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
  assert(ms != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_POS_INTERVAL, (uint8_t *) ms, 2);
  }
  else{
    return remoteRegRead(remote_id, POZYX_POS_INTERVAL, (uint8_t *) ms, 2);
  }
}

int PozyxClass::setUpdateInterval(uint16_t ms, uint16_t remote_id)
{ 
  assert(ms > 100);
  assert(ms <= 60000);

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
  assert(gpio_num > 0);
  assert(gpio_num <= 4);
  assert(mode != NULL);

  int status;
  uint8_t gpio_register = POZYX_CONFIG_GPIO1 + (gpio_num-1);

  if(remote_id == NULL){
    status = regRead(gpio_register, mode, 1);
  }
  else{
    status = remoteRegRead(remote_id, gpio_register, mode, 1);
  }

  *mode &= 0x7;
  return status;
}

int PozyxClass::getConfigPullGPIO(int gpio_num, uint8_t *pull, uint16_t remote_id)
{
  assert(gpio_num > 0);
  assert(gpio_num <= 4);
  assert(pull != NULL);
    

  int status;
  uint8_t gpio_register = POZYX_CONFIG_GPIO1 + (gpio_num-1);

  if(remote_id == NULL){
    status = regRead(gpio_register, pull, 1);
  }
  else{
    status = remoteRegRead(remote_id, gpio_register, pull, 1);
  }

  *pull = (*pull & 0x18) >> 3;
  return status;
}

int PozyxClass::setConfigGPIO(int gpio_num, int mode, int pull, uint16_t remote_id)
{
  assert(gpio_num > 0);
  assert(gpio_num <= 4);
  assert((mode == POZYX_GPIO_DIGITAL_INPUT) || (mode == POZYX_GPIO_PUSHPULL) || (mode == POZYX_GPIO_OPENDRAIN) );
  assert((pull == POZYX_GPIO_NOPULL) || (mode == POZYX_GPIO_PULLUP) || (mode == POZYX_GPIO_PULLDOWN) );
  

  int status;
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
  assert(algorithm != NULL);

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
  assert(dimension != NULL);

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
  assert( (algorithm == POZYX_POS_ALG_UWB_ONLY ) || (algorithm == POZYX_POS_ALG_LS) );
  assert( (dimension == POZYX_3D ) || (dimension == POZYX_2D) || (dimension == POZYX_2_5D) );

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

int PozyxClass::getAnchorSelectionMode(uint8_t *mode, uint16_t remote_id)
{
  assert(mode != NULL);

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
  assert(nr_anchors != NULL);
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
  assert( (mode == POZYX_ANCHOR_SEL_MANUAL ) || (mode == POZYX_ANCHOR_SEL_AUTO ));
  assert( nr_anchors > 2);
  assert( nr_anchors <= 16);

  int status;

  uint8_t params = (mode << 7) + nr_anchors;

  if(remote_id == NULL){
    status = regWrite(POZYX_POS_NUM_ANCHORS, &params, 1);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_POS_NUM_ANCHORS, &params, 1);
  }
  return status;
}

int PozyxClass::getNetworkId(uint16_t *network_id)
{
  assert(network_id != NULL);

  return regRead(POZYX_NETWORK_ID, (uint8_t *) network_id, 2);
}

int PozyxClass::setNetworkId(uint16_t network_id, uint16_t remote_id)
{
  assert(network_id != 0);

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
  assert(UWB_settings != NULL);

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

int PozyxClass::setUWBChannel(int channel_num, uint16_t remote_id)
{
  assert(channel_num >= 1);
  assert(channel_num <= 7);
  assert(channel_num != 6);

  if(remote_id == NULL){
    return regWrite(POZYX_UWB_CHANNEL, (uint8_t *)&channel_num, 1);
  }
  else{
    return remoteRegWrite(remote_id, POZYX_UWB_CHANNEL, (uint8_t *)&channel_num, 1);
  }
}

int PozyxClass::getUWBChannel(int* channel_num, uint16_t remote_id)
{
  assert(channel_num != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_UWB_CHANNEL, (uint8_t *)&channel_num, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_UWB_CHANNEL, (uint8_t *)&channel_num, 1);
  }
}

int PozyxClass::setTxPower(float txgain_dB, uint16_t remote_id)
{
  assert(txgain_dB >= 0.0f);
  assert(txgain_dB <= 35.0f);

  // convert to an int where one unit is 0.5dB
  uint8_t doublegain_dB = (int)(2.0*txgain_dB + 0.5f);

  if(remote_id == NULL){
    return regWrite(POZYX_UWB_GAIN, &doublegain_dB, 1);
  }
  else{
    return remoteRegWrite(remote_id, POZYX_UWB_GAIN, &doublegain_dB, 1);
  }
}

int PozyxClass::getTxPower(float* txgain_dB, uint16_t remote_id)
{
  assert(txgain_dB != NULL);

  uint8_t doublegain_dB = 0;
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_UWB_GAIN, (uint8_t *)&doublegain_dB, 1);
  }
  else{
    status = remoteRegRead(remote_id, POZYX_UWB_GAIN, (uint8_t *)&doublegain_dB, 1);
  }

  *txgain_dB = 0.5f*doublegain_dB;
  return status;
}

int PozyxClass::getOperationMode(uint8_t *mode, uint16_t remote_id)
{
  assert(mode != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_OPERATION_MODE, mode, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_OPERATION_MODE, mode, 1);
  }
}

int PozyxClass::setOperationMode(uint8_t mode, uint16_t remote_id)
{
  assert( (mode == POZYX_ANCHOR_MODE ) || (mode == POZYX_TAG_MODE) );

  int status;

  if(remote_id == NULL){
    return regWrite(POZYX_OPERATION_MODE, (uint8_t *) &mode, 1);
  }
  else{
    return remoteRegWrite(remote_id, POZYX_OPERATION_MODE, (uint8_t *) &mode, 1);
  }

}

int PozyxClass::getSensorMode(uint8_t *sensor_mode, uint16_t remote_id)
{
  assert(sensor_mode != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_SENSORS_MODE, (uint8_t *) sensor_mode, 1);
  }
  else{
    return remoteRegRead(remote_id, POZYX_SENSORS_MODE, (uint8_t *) sensor_mode, 1);
  }
}

int PozyxClass::setSensorMode(uint8_t sensor_mode, uint16_t remote_id)
{
  assert(sensor_mode >= 0);
  assert(sensor_mode <= 12);

  int status;

  if(remote_id == NULL){
    status = regWrite(POZYX_SENSORS_MODE, (uint8_t *) &sensor_mode, 1);
    
    // delay required to switch modes.
    delay(20);
  }
  else{
    status = remoteRegWrite(remote_id, POZYX_SENSORS_MODE, (uint8_t *) &sensor_mode, 1);
  }

  return status;
}

/*
*
* POSITION DATA
*
*/
int PozyxClass::getCoordinates(coordinates_t *coordinates, uint16_t remote_id)
{
  assert(coordinates != NULL);

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
  assert(pos_error != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_POS_ERR_X, (uint8_t *) pos_error, sizeof(pos_error_t));
  }
  else{
    return remoteRegRead(remote_id, POZYX_POS_ERR_X, (uint8_t *) pos_error, sizeof(pos_error_t));
  }
}


/*
 * Function regarding the sensor data
 */

int PozyxClass::getAllSensorData(sensor_data_t *sensor_data, uint16_t remote_id)
{
  assert(sensor_data != NULL);

  sensor_raw_t raw_data;
  int status;

  if(remote_id == NULL){
    status =  regRead(POZYX_PRESSURE, (uint8_t *)&raw_data, sizeof(sensor_raw_t));
  }
  else{
    status =  remoteRegRead(remote_id, POZYX_PRESSURE, (uint8_t *)&raw_data, sizeof(sensor_raw_t));
  }

  // convert all the raw data to meaningfull physical quantities
  sensor_data->pressure = raw_data.pressure / POZYX_PRESS_DIV_PA;

  sensor_data->acceleration.x = raw_data.acceleration[0] / POZYX_ACCEL_DIV_MG;
  sensor_data->acceleration.y = raw_data.acceleration[1] / POZYX_ACCEL_DIV_MG;
  sensor_data->acceleration.z = raw_data.acceleration[2] / POZYX_ACCEL_DIV_MG;

  sensor_data->magnetic.x = raw_data.magnetic[0] / POZYX_MAG_DIV_UT;
  sensor_data->magnetic.y = raw_data.magnetic[1] / POZYX_MAG_DIV_UT;
  sensor_data->magnetic.z = raw_data.magnetic[2] / POZYX_MAG_DIV_UT;

  sensor_data->angular_vel.x = raw_data.angular_vel[0] / POZYX_GYRO_DIV_DPS;
  sensor_data->angular_vel.y = raw_data.angular_vel[1] / POZYX_GYRO_DIV_DPS;
  sensor_data->angular_vel.z = raw_data.angular_vel[2] / POZYX_GYRO_DIV_DPS;
  
  sensor_data->euler_angles.heading = raw_data.euler_angles[0] / POZYX_EULER_DIV_DEG;
  sensor_data->euler_angles.roll  = raw_data.euler_angles[1] / POZYX_EULER_DIV_DEG;
  sensor_data->euler_angles.pitch = raw_data.euler_angles[2] / POZYX_EULER_DIV_DEG;

  sensor_data->quaternion.weight = raw_data.quaternion[0] / POZYX_QUAT_DIV;
  sensor_data->quaternion.x = raw_data.quaternion[1] / POZYX_QUAT_DIV;
  sensor_data->quaternion.y = raw_data.quaternion[2] / POZYX_QUAT_DIV;
  sensor_data->quaternion.z = raw_data.quaternion[3] / POZYX_QUAT_DIV;

  sensor_data->linear_acceleration.x = raw_data.linear_acceleration[0] / POZYX_ACCEL_DIV_MG;
  sensor_data->linear_acceleration.y = raw_data.linear_acceleration[1] / POZYX_ACCEL_DIV_MG;
  sensor_data->linear_acceleration.z = raw_data.linear_acceleration[2] / POZYX_ACCEL_DIV_MG;

  sensor_data->gravity_vector.x = raw_data.gravity_vector[0] / POZYX_ACCEL_DIV_MG;
  sensor_data->gravity_vector.y = raw_data.gravity_vector[1] / POZYX_ACCEL_DIV_MG;
  sensor_data->gravity_vector.z = raw_data.gravity_vector[2] / POZYX_ACCEL_DIV_MG;

  sensor_data->temperature = raw_data.temperature; 

  return status;
}

int PozyxClass::getPressure_Pa(float32_t *pressure, uint16_t remote_id)
{
  assert(pressure != NULL);

  // the raw data is one 32bit value
  uint32_t raw_data;
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_PRESSURE, (uint8_t *)&raw_data, sizeof(uint32_t)); 
  }
  else{
    status = remoteRegRead(remote_id, POZYX_PRESSURE, (uint8_t *)&raw_data, sizeof(uint32_t)); 
  }

  // convert the raw data from the pressure sensor to pressure in Pascal
  *pressure = raw_data / POZYX_PRESS_DIV_PA;

  return status;
}

int PozyxClass::getAcceleration_mg(acceleration_t *acceleration, uint16_t remote_id)
{
  assert(acceleration != NULL);

  // the raw data are three 16bit values
  int16_t raw_data[3] = {0,0,0};
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_ACCEL_X, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  }
  else{
    status = remoteRegRead(remote_id, POZYX_ACCEL_X, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  } 

  // convert the raw data from the accelerometer to acceleration in mg
  acceleration->x = raw_data[0] / POZYX_ACCEL_DIV_MG;
  acceleration->y = raw_data[1] / POZYX_ACCEL_DIV_MG;
  acceleration->z = raw_data[2] / POZYX_ACCEL_DIV_MG;

  return status;
}

int PozyxClass::getMagnetic_uT(magnetic_t *magnetic, uint16_t remote_id)
{
  assert(magnetic != NULL);

  // the raw data are three 16bit values
  int16_t raw_data[3] = {0,0,0};
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_MAGN_X, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  }
  else{
    status = remoteRegRead(remote_id, POZYX_MAGN_X, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  } 

  // convert the raw data from the magnetometer to magnetic field strength in µTesla (ut)
  magnetic->x = raw_data[0] / POZYX_MAG_DIV_UT;
  magnetic->y = raw_data[1] / POZYX_MAG_DIV_UT;
  magnetic->z = raw_data[2] / POZYX_MAG_DIV_UT;

  return status;
}

int PozyxClass::getAngularVelocity_dps(angular_vel_t *angular_vel, uint16_t remote_id)
{
  assert(angular_vel != NULL);

  // the raw data are three 16bit values
  int16_t raw_data[3] = {0,0,0};
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_GYRO_X, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  }
  else{
    status = remoteRegRead(remote_id, POZYX_GYRO_X, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  } 

  // convert the raw data from the gyroscope to angular velocity in degrees per second (dps)
  angular_vel->x = raw_data[0] / POZYX_GYRO_DIV_DPS;
  angular_vel->y = raw_data[1] / POZYX_GYRO_DIV_DPS;
  angular_vel->z = raw_data[2] / POZYX_GYRO_DIV_DPS;

  return status;
}

int PozyxClass::getEulerAngles_deg(euler_angles_t *euler_angles, uint16_t remote_id)
{
  assert(euler_angles != NULL);

  // the raw data are three 16bit values
  int16_t raw_data[3] = {0,0,0};
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_EUL_HEADING, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  }
  else{
    status = remoteRegRead(remote_id, POZYX_EUL_HEADING, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  } 

  // convert the raw data from to euler angles in degrees
  euler_angles->heading = raw_data[0] / POZYX_EULER_DIV_DEG;
  euler_angles->roll  = raw_data[1] / POZYX_EULER_DIV_DEG;
  euler_angles->pitch = raw_data[2] / POZYX_EULER_DIV_DEG;

  return status;
}

int PozyxClass::getQuaternion(quaternion_t *quaternion, uint16_t remote_id)
{ 
  assert(quaternion != NULL);

  // the raw data are four 16bit values
  int16_t raw_data[4] = {0,0,0,0};
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_QUAT_W, (uint8_t *)&raw_data, 4*sizeof(int16_t));
  }
  else{
    status = remoteRegRead(remote_id, POZYX_QUAT_W, (uint8_t *)&raw_data, 4*sizeof(int16_t));
  } 

  // convert the raw data from to a quaternion
  quaternion->weight = raw_data[0] / POZYX_QUAT_DIV;
  quaternion->x = raw_data[1] / POZYX_QUAT_DIV;
  quaternion->y = raw_data[2] / POZYX_QUAT_DIV;
  quaternion->z = raw_data[3] / POZYX_QUAT_DIV;

  return status;
}

int PozyxClass::getLinearAcceleration_mg(linear_acceleration_t *linear_acceleration, uint16_t remote_id)
{
  assert(linear_acceleration != NULL);

  // the raw data are three 16bit values
  int16_t raw_data[3] = {0,0,0};
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_LIA_X, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  }
  else{
    status = remoteRegRead(remote_id, POZYX_LIA_X, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  } 

  // convert the raw data to linear acceleration in mg
  linear_acceleration->x = raw_data[0] / POZYX_ACCEL_DIV_MG;
  linear_acceleration->y = raw_data[1] / POZYX_ACCEL_DIV_MG;
  linear_acceleration->z = raw_data[2] / POZYX_ACCEL_DIV_MG;

  return status;
}

int PozyxClass::getGravityVector_mg(gravity_vector_t *gravity_vector, uint16_t remote_id)
{
  assert(gravity_vector != NULL);

  // the raw data are three 16bit values
  int16_t raw_data[3] = {0,0,0};
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_GRAV_X, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  }
  else{
    status = remoteRegRead(remote_id, POZYX_GRAV_X, (uint8_t *)&raw_data, 3*sizeof(int16_t));
  } 

  // convert the raw data to gravity vector in mg
  gravity_vector->x = raw_data[0] / POZYX_ACCEL_DIV_MG;
  gravity_vector->y = raw_data[1] / POZYX_ACCEL_DIV_MG;
  gravity_vector->z = raw_data[2] / POZYX_ACCEL_DIV_MG;

  return status;
}

int PozyxClass::getTemperature_c(float32_t *temperature, uint16_t remote_id)
{
  assert(temperature != NULL);

  // the raw data is one 8bit value
  uint8_t raw_data = 0;
  int status;

  if(remote_id == NULL){
    status = regRead(POZYX_TEMPERATURE, (uint8_t *)&raw_data, 1); 
  }
  else{
    status = remoteRegRead(remote_id, POZYX_TEMPERATURE, (uint8_t *)&raw_data, 1); 
  }

  // convert the raw data to the temperature in degrees Celcius
  *temperature = raw_data / POZYX_TEMP_DIV_CELSIUS;

  return status;
}

int PozyxClass::getDeviceListSize(uint8_t *device_list_size, uint16_t remote_id)
{
  assert(device_list_size != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_DEVICE_LIST_SIZE, (uint8_t *) device_list_size, 1);  
  }
  else{
    return remoteRegRead(remote_id, POZYX_DEVICE_LIST_SIZE, (uint8_t *) device_list_size, 1);
  }
}

int PozyxClass::getLastNetworkId(uint16_t *network_id, uint16_t remote_id)
{
  assert(network_id != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_RX_NETWORK_ID, (uint8_t *) network_id, 2);
  }
  else{
    return remoteRegRead(remote_id, POZYX_RX_NETWORK_ID, (uint8_t *) network_id, 2);
  } 
}

int PozyxClass::getLastDataLength(uint8_t *data_length, uint16_t remote_id)
{
  assert(data_length != NULL);

  if(remote_id == NULL){
    return regRead(POZYX_RX_DATA_LEN, (uint8_t *) data_length, 1);  
  }
  else{
    return remoteRegRead(remote_id, POZYX_RX_DATA_LEN, (uint8_t *) data_length, 1); 
  }
}



int PozyxClass::getGPIO(int gpio_num, uint8_t *value, uint16_t remote_id)
{
  assert(gpio_num >= 1);
  assert(gpio_num <= 4);
  assert(value != NULL);
   
  uint8_t gpio_register = POZYX_GPIO1 + (gpio_num-1);
  if(remote_id == NULL){
    return regRead(gpio_register, (uint8_t *) value, 1);
  }
  else{
    return remoteRegRead(remote_id, gpio_register, (uint8_t *) value, 1);
  }

}

int PozyxClass::setGPIO(int gpio_num, uint8_t value, uint16_t remote_id)
{
  assert(gpio_num >= 1);
  assert(gpio_num <= 4);
  assert( (value == 0) || (value == 1) );

  int status = POZYX_FAILURE;
  uint8_t gpio_register = POZYX_GPIO1 + (gpio_num-1);

  if(remote_id == NULL){
    return regWrite(gpio_register, &value, 1);
  }
  else{
    return remoteRegWrite(remote_id, gpio_register, &value, 1);
  }

}

void PozyxClass::resetSystem(uint16_t remote_id)
{
  if(remote_id == NULL){
    regFunction(POZYX_RESET_SYS, NULL, 0, NULL, 0); 
  }
  else{
    remoteRegFunction(remote_id, POZYX_RESET_SYS, NULL, 0, NULL, 0); 
  }
  
}


String PozyxClass::getSystemError(uint16_t remote_id)
{
  uint8_t error_code;

  if(remote_id == NULL){
    regRead(POZYX_ERRORCODE, &error_code, 1); 
  }
  else{
    remoteRegRead(remote_id, POZYX_ERRORCODE, &error_code, 1); 
  }

  switch(error_code)
  {
    case POZYX_ERROR_NONE:
      return F("");
    case POZYX_ERROR_I2C_WRITE:
      return F("Error 0x01: Error writing to a register through the I2C bus");
    case POZYX_ERROR_I2C_CMDFULL:
      return F("Error 0x02: Pozyx cannot handle all the I2C commands at once");
    case POZYX_ERROR_ANCHOR_ADD:
      return F("Error 0x03: Cannot add anchor to the internal device list");
    case POZYX_ERROR_COMM_QUEUE_FULL:
      return F("Error 0x04: Communication queue is full, too many UWB messages");
    case POZYX_ERROR_I2C_READ:
      return F("Error 0x05: Error reading from a register from the I2C bus");
    case POZYX_ERROR_UWB_CONFIG:
      return F("Error 0x06: Cannot change the UWB configuration");
    case POZYX_ERROR_OPERATION_QUEUE_FULL:
      return F("Error 0x07: Pozyx cannot handle all the operations at once");
    case POZYX_ERROR_STARTUP_BUSFAULT:
      return F("Error 0x08: Internal bus error");
    case POZYX_ERROR_FLASH_INVALID:
      return F("Error 0x09: Flash memory is corrupted or invalid");
    case POZYX_ERROR_NOT_ENOUGH_ANCHORS:
      return F("Error 0x0A: Not enough anchors available for positioning");
    case POZYX_ERROR_DISCOVERY:
      return F("Error 0x0B: Error during the Discovery process");
    case POZYX_ERROR_CALIBRATION:
      return F("Error 0x0C: Error during the auto calibration process");
    case POZYX_ERROR_FUNC_PARAM:
      return F("Error 0x0D: Invalid function parameters for the register function");
    case POZYX_ERROR_ANCHOR_NOT_FOUND:
      return F("Error 0x0E: The coordinates of an anchor are not found");
    case POZYX_ERROR_GENERAL:
      return F("Error 0xFF: General error");
    default:
      return F("Unknonw error");
  }

}

int PozyxClass::setLed(int led_num, boolean state, uint16_t remote_id)
{
  assert(led_num >= 1);
  assert(led_num <= 4);
  assert( (state == true) || (state == false) );

  int status;
  // the 4 MSB indicate which led we wish to control, the 4 LSB indicate the state of the leds
  uint8_t params = (0x1 << (led_num-1+4)) | (((uint8_t)state) << (led_num-1));  
  
  if(remote_id == NULL){
    return regFunction(POZYX_LED_CTRL, &params, 1, NULL, 0);
  }
  else{
    return remoteRegFunction(remote_id, POZYX_LED_CTRL, &params, 1, NULL, 0);
  }
}

int PozyxClass::doRanging(uint16_t destination, device_range_t *range)
{
  assert(destination != 0);
  assert(range != NULL);
  
  int status;
  // trigger the ranging
  status = regFunction(POZYX_DO_RANGING, (uint8_t *) &destination, 2, NULL, 0);  
  if (status == POZYX_SUCCESS )
  {
    // wait for the result
    if(waitForFlag(POZYX_INT_STATUS_FUNC, POZYX_DELAY_INTERRUPT))
    {
      // read out the ranging results
      return getDeviceRangeInfo(destination, range);
    }else{
      return POZYX_TIMEOUT;
    }
  }else{
    return POZYX_FAILURE;
  }
  
  
}

int PozyxClass::doRemoteRanging(uint16_t device_from, uint16_t device_to, device_range_t *device_range)
{
  assert(device_from != 0);
  assert(device_to != 0);
  assert(device_range != NULL);

  int status;

  // trigger remote ranging between the two devices
  status = remoteRegFunction(device_from, POZYX_DO_RANGING, (uint8_t *) device_to, 2, NULL, 0); 
  if (status == POZYX_SUCCESS)
  {
    // the remote device (device_from) will respond with the ranging result, wait for that to happen
    if(waitForFlag(POZYX_INT_STATUS_RX_DATA , POZYX_DELAY_INTERRUPT))
    {
      // read out the ranging results from the received message
      return readRXBufferData((uint8_t *) device_range, sizeof(device_range_t));
    }else{
      return POZYX_TIMEOUT;
    }    
  }else{
    return POZYX_FAILURE;
  }
 
}

int PozyxClass::doPositioning(coordinates_t *position, uint8_t dimension, int32_t height, uint8_t algorithm)
{
  assert(position != NULL);
  assert( (algorithm == POZYX_POS_ALG_UWB_ONLY ) || (algorithm == POZYX_POS_ALG_LS) );
  assert( (dimension == POZYX_3D ) || (dimension == POZYX_2D) || (dimension == POZYX_2_5D) );
  
  int status;

  // set dimension and algorithm
  uint8_t alg_options = (dimension<<4) | algorithm;
  status = regWrite(POZYX_POS_ALG, &alg_options, 1);

  // in 2.5D mode, we also supply the height
  if(dimension == POZYX_2_5D) {
    status = regWrite(POZYX_POS_Z, (uint8_t*)&height, sizeof(int32_t));
  }
  
  // trigger positioning
  status = regFunction(POZYX_DO_POSITIONING, NULL, 0, NULL, 0); 
  if (status == POZYX_SUCCESS )
  {
    // wait for positioning to finish
    if(waitForFlag(POZYX_INT_STATUS_POS, POZYX_DELAY_INTERRUPT)){
      status = getCoordinates(position);
      return status;
    }else{
      return POZYX_TIMEOUT;
    }    
  }
  else{
    return POZYX_FAILURE;
  }
}

int PozyxClass::doRemotePositioning(uint16_t remote_id, coordinates_t *coordinates, uint8_t dimension, int32_t height, uint8_t algorithm)
{
  assert(remote_id != 0);
  assert(coordinates != NULL);
  assert( (algorithm == POZYX_POS_ALG_UWB_ONLY ) || (algorithm == POZYX_POS_ALG_LS) );
  assert( (dimension == POZYX_3D ) || (dimension == POZYX_2D) || (dimension == POZYX_2_5D) );
  

  int status;
  coordinates->x = 0;
  coordinates->y = 0;
  coordinates->z = 0;

  // set dimension and algorithm
  /*uint8_t alg_options = (dimension<<4) | algorithm;
  status = remoteRegWrite(remote_id, POZYX_POS_ALG, &alg_options, 1);
  delay(5);
  */

  // in 2.5D mode, we also supply the height
  if(dimension == POZYX_2_5D) {
    status = remoteRegWrite(remote_id, POZYX_POS_Z, (uint8_t*)&height, sizeof(int32_t));
    delay(10);
  }

  // trigger remote positioning
  status = remoteRegFunction(remote_id, POZYX_DO_POSITIONING, NULL, 0, NULL, 0); 

  if(status != POZYX_SUCCESS){
    
  }

  if (waitForFlag(POZYX_INT_STATUS_RX_DATA , 500)){

    // we received a response, now get some information about the response
    uint8_t rx_info[3]= {0,0,0};
    regRead(POZYX_RX_NETWORK_ID, rx_info, 3);
    uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
    uint8_t data_len = rx_info[2];
    
    // check if we have received the expected response, i.e., a packet containing the coordinates.
    if( remote_network_id == remote_id && data_len == sizeof(coordinates_t))
    {

      status = readRXBufferData((uint8_t *) coordinates, 12); //sizeof(coordinates_t));
      return status;
    }else{
      /*Serial.println("Wrong response, no coordinates");
      Serial.print("Remote id: ");
      Serial.println(remote_network_id, HEX);
      Serial.print("data length: ");
      Serial.println(data_len);*/
      return POZYX_FAILURE;
    }
  }
  else{
    //Serial.println("timeout");
    return POZYX_TIMEOUT;
  }
  return status;
}

int PozyxClass::setPositioningAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id)
{
  assert(anchor_num > 0);
  assert(anchor_num <= 10);

  int status;

  if(remote_id == NULL){
    status = regFunction(POZYX_POS_SET_ANCHOR_IDS, (uint8_t *) anchors, anchor_num * 2, NULL, 0); 
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_POS_SET_ANCHOR_IDS, (uint8_t *) anchors, anchor_num * 2, NULL, 0); 
  }
  return status;
}


int PozyxClass::getPositioningAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id)
{
  assert(anchor_num > 0);
  assert(anchor_num <= 10);

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
  assert(size > 0);
  assert(size <= 20);

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
  assert(size > 0);
  assert(size <= 20);

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
  assert(size > 0);
  assert(size <= 20);

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
  assert( (type == POZYX_DISCOVERY_ANCHORS_ONLY) || (type == POZYX_DISCOVERY_TAGS_ONLY) || (type == POZYX_DISCOVERY_ALL_DEVICES));
  assert(slots > 1);
  assert(slots < 10);
  assert(slot_duration > 5);

  int status;
  uint8_t params[3];

  params[0] = (uint8_t)type;
  params[1] = (uint8_t)slots;
  params[2] = (uint8_t)slot_duration;


  status = regFunction(POZYX_DEVICES_DISCOVER, (uint8_t *)&params, 3, NULL, 0);
  if (status == POZYX_SUCCESS && waitForFlag(POZYX_INT_STATUS_FUNC, POZYX_DELAY_INTERRUPT)){
    return status;
  }
  else{
    return POZYX_TIMEOUT;
  }
  return status;
}

int PozyxClass::doAnchorCalibration(int dimension, int num_measurements, int num_anchors, uint16_t anchors[],  int32_t heights[])
{
  assert( (dimension == POZYX_2D ) || (dimension == POZYX_2_5D));
  assert( num_measurements > 0);
  assert( num_anchors >= 3);
  assert( num_anchors <= 6);

  int status;

  if (num_anchors < 0 || num_anchors > 6)
    return POZYX_FAILURE;
  if(dimension != POZYX_2D && dimension != POZYX_2_5D)
    return POZYX_FAILURE;

  // in 2.5D mode, we must supply the heights of all the anchors
  if(dimension == POZYX_2_5D){
    device_coordinates_t anchor;
    int i;
    for(i=0; i< num_anchors; i++){
      anchor.network_id = anchors[i];
      anchor.flag = 0x1; 
      anchor.pos.x = 0;
      anchor.pos.y = 0;
      anchor.pos.z = heights[i];
      Pozyx.addDevice(anchor);
    }
  } 

/*
  Serial.println("devices added");
  uint8_t list_size = 0;
  status = Pozyx.getDeviceListSize(&list_size);
  Serial.print("list size: ");
  Serial.println(status*list_size);
*/

  uint8_t params[2 + num_anchors * sizeof(uint16_t)];
  params[0] = (uint8_t)dimension;
  params[1] = (uint8_t)num_measurements;

  if (num_anchors > 0){
    memcpy(params+2, (uint8_t*)anchors, num_anchors * sizeof(uint16_t));
  }

  status = regFunction(POZYX_DEVICES_CALIBRATE, (uint8_t *)&params, 2 + num_anchors * sizeof(uint16_t), NULL, 0);
  Serial.println(status);
  delay(POZYX_DELAY_LOCAL_FUNCTION);
  if (status == POZYX_SUCCESS && waitForFlag(POZYX_INT_STATUS_FUNC, 25000)){
    return POZYX_SUCCESS;
  }
  else{
    return POZYX_TIMEOUT;
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
    status = remoteRegFunction(remote_id, POZYX_DEVICE_ADD, (uint8_t *) &device_coordinates, sizeof(device_coordinates_t), NULL, 0); 
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }
  return status;
}

int PozyxClass::getDeviceCoordinates(uint16_t device_id, coordinates_t *coordinates, uint16_t remote_id)
{
  assert(device_id != 0);
  assert(coordinates != NULL);

  int status;

  if(remote_id == NULL){
    status = regFunction(POZYX_DEVICE_GETCOORDS, (uint8_t *) &device_id, 2, (uint8_t *)coordinates, sizeof(coordinates_t));
    delay(POZYX_DELAY_LOCAL_FUNCTION);
  }
  else{
    status = remoteRegFunction(remote_id, POZYX_DEVICE_GETCOORDS, (uint8_t *) &device_id, 2, (uint8_t *) coordinates, sizeof(coordinates_t));
    delay(POZYX_DELAY_REMOTE_FUNCTION);
  }
  return status;
}

int PozyxClass::getDeviceRangeInfo(uint16_t device_id, device_range_t *device_range, uint16_t remote_id)
{
  assert(device_id != 0);
  assert(device_range != NULL);
  
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

///////////////////////////////////////////////// ASSERTIONS /////////////////////////////////////

#ifdef __ASSERT_USE_STDERR
void __attribute__((weak)) __assert (const char *func, const char *file, int line, const char *failedexpr)
{
    // print out whatever you like here, function name, filename, line#, expression that failed.
  if (Serial){
    Serial.print("Assertion in function : ");
    Serial.println(func);
    Serial.print("Assertion failed : ");
    Serial.println(failedexpr);
    Serial.print("Filename: ");
    Serial.println(file);
    Serial.print("Line number: ");
    Serial.println(line);

    // platform independent delay to allow the string to be printed
    delay(10);
  }

    // halt after outputting information
    abort(); 
}
#else
void __attribute__((weak)) __assert_pozyx (const char *__func, const char *__file, int __lineno)
{
    // print out whatever you like here, function name, filename, line#, expression that failed.
  if (Serial){
    Serial.print("Assertion failed in function : ");
    Serial.println(__func);
    Serial.print("Filename: ");
    Serial.println(__file);
    Serial.print("Line number: ");
    Serial.println(__lineno); 

    // platform independent delay to allow the string to be printed
    delay(10);
  }

  // halt after outputting information
  abort(); 
}

#endif
