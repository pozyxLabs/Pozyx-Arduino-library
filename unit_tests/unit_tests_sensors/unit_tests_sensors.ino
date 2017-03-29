#line 2 "unit_tests_core.ino"
#include <ArduinoUnit.h>

#define NDEBUG

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

/**
 * Unit tests for the Pozyx sensors: accelerometer, gyroscope, magnetometer, pressure sensor, temperature sensor, etc..
 * This test can be run on an Arduino Uno.
 *
 * Author, Samuel Van de Velde, Pozyx Labs
 * 
 * functions tested:
 * - getAcceleration_mg
 * - getAllSensorData
 * - getAngularVelocity_dps
 * - getEulerAngles_deg
 * - getGravityVector_mg
 * - getLinearAcceleration_mg
 * - getMagnetic_uT
 * - getPressure_Pa
 * - getQuaternion
 * - getSensorMode
 * - getTemperature_c
 * - setSensorMode
 *
 * Todo: 
 * -----
 * - better test for the magnetometer
 * - better test for the quaternions
 * - we don't test if the sensor mode actually *does* something
 */
 
 
void setup()
{
  Serial.begin(115200);
  while(!Serial); // for the Arduino Leonardo/Micro only
  
  Wire.begin();

  Serial.println(F("Start testing the Pozyx sensors"));  
  Serial.println(F("This test expects the tag to lay still on a flat surface.\n")); 
  Serial.println(F("---------------------------------------------------------\n"));  
  
  Pozyx.setSensorMode(12);
  delay(500);
}

void loop()
{
  Test::run();
}

/*
 * Test if pozyx produces sensor values at 100Hz, by checking the interrrupt status.
 * We check this by counting the number of new measurements during a fixed interval
 */
test(update_rate)
{
  int counter = 0;
  int timewindow_ms = 2000;    // the clock on the arduino is pretty crappy, so this can be more or less this value
  long chrono_ms = millis();
  uint8_t int_status; 
  
  while(millis() - chrono_ms < timewindow_ms)
  {
    Pozyx.regRead(POZYX_INT_STATUS, &int_status, 1);
    if((int_status & POZYX_INT_STATUS_IMU) == POZYX_INT_STATUS_IMU)
      counter++;
    delay(1);
  }
  
  // 100Hz should be about one measurement per 10ms
  float avg_delay = timewindow_ms/counter;
  assertLess(avg_delay, 12);
  assertMore(avg_delay, 5);    
}

test(getPressure_Pa)
{
  float pressure;
  int result;
  
  result = Pozyx.getPressure_Pa( &pressure);
  assertEqual(result, POZYX_SUCCESS);
  assertLess(pressure, 120000);
  assertMore(pressure, 80000);
}

test(getTemperature_c)
{
  float temp;
  int result;
  
  result = Pozyx.getTemperature_c( &temp);
  assertEqual(result, POZYX_SUCCESS);
  assertLess(temp, 50);
  assertMore(temp, -10);
}

/*
 * Test the acceleration
 */
test(getAcceleration_mg)
{
  acceleration_t acceleration;
  int result;
  
  result = Pozyx.getAcceleration_mg( &acceleration);
  assertEqual(result, POZYX_SUCCESS);
  
  assertLess(abs(acceleration.x), 1200);
  assertLess(abs(acceleration.y), 1200);
  assertLess(abs(acceleration.z), 1200);
  
  float norm = acceleration.x*acceleration.x + acceleration.y*acceleration.y +acceleration.z*acceleration.z;
  norm = sqrt(norm);
  
  assertLess(norm, 1100);
  assertMore(norm, 900);  
}

/*
 * Test the angular velocity
 */
test(getAngularVelocity_dps)
{
  angular_vel_t angular_vel;
  int result;
  
  result = Pozyx.getAngularVelocity_dps( &angular_vel);
  assertEqual(result, POZYX_SUCCESS);
    
  assertLess(abs(angular_vel.x), 1);
  assertLess(abs(angular_vel.y), 1);
  assertLess(abs(angular_vel.z), 1);
}

/*
 * Test the magnetic field sensor
 */
test(getMagnetic_uT)
{
  magnetic_t magn;
  int result;
  
  result = Pozyx.getMagnetic_uT( &magn);
  assertEqual(result, POZYX_SUCCESS);
    
  /*Serial.println(magn.x);  
  Serial.println(magn.y); 
  Serial.println(magn.z); 
  */  
    
  assertLess(abs(magn.x), 500);
  assertLess(abs(magn.y), 500);
  assertLess(abs(magn.z), 500);
  
  float norm = magn.x*magn.x + magn.y*magn.y +magn.z*magn.z;
  norm = sqrt(norm);
  
  //Serial.println(norm);
  
  assertLess(norm, 500);
  assertMore(norm, 10);  
}

test(getLinearAcceleration_mg)
{
  acceleration_t lin_acceleration;
  float acc_treshold_mg = 100;
  int result;
  
  result = Pozyx.getLinearAcceleration_mg( &lin_acceleration);
  assertEqual(result, POZYX_SUCCESS);
  
  assertLess(abs(lin_acceleration.x), acc_treshold_mg);
  assertLess(abs(lin_acceleration.y), acc_treshold_mg);
  assertLess(abs(lin_acceleration.z), acc_treshold_mg);
  
  float norm = lin_acceleration.x*lin_acceleration.x + lin_acceleration.y*lin_acceleration.y +lin_acceleration.z*lin_acceleration.z;
  norm = sqrt(norm);
  
  assertLess(norm, acc_treshold_mg); 
}

test(getGravityVector_mg)
{
  acceleration_t grav;
  float acc_treshold_mg = 1200;
  int result;
  
  result = Pozyx.getGravityVector_mg( &grav);
  assertEqual(result, POZYX_SUCCESS);
  
  assertLess(abs(grav.x), acc_treshold_mg);
  assertLess(abs(grav.y), acc_treshold_mg);
  assertLess(abs(grav.z), acc_treshold_mg);
  
  float norm = grav.x*grav.x + grav.y*grav.y +grav.z*grav.z;
  norm = sqrt(norm);
  
  assertLess(norm, acc_treshold_mg); 
  assertMore(norm, 800); 
}

test(getEulerAngles_deg)
{
  euler_angles_t euler_angles;
  int result;
  
  result = Pozyx.getEulerAngles_deg( &euler_angles);
  assertEqual(result, POZYX_SUCCESS);
  
  assertLess(abs(euler_angles.heading), 361);
  assertLess(abs(euler_angles.roll), 361);
  assertLess(abs(euler_angles.pitch), 361);
}

test(getQuaternion)
{
  quaternion_t quat;
  int result;
  
  result = Pozyx.getQuaternion( &quat);
  assertEqual(result, POZYX_SUCCESS);
  
  // when the device is laying flat, this should be around 1.
  // when the device is upside down, it will be negative
  float gz = quat.weight*quat.weight - quat.x*quat.x - quat.y*quat.y + quat.z*quat.z;
  assertMore(gz, 0.9);
  assertLess(gz, 1.1);
}

test(sensorMode)
{
  uint8_t sensor_modes[13] = {0,1,2,3,4,5,6,7,8,9,10,11,12};
  uint8_t sensor_mode;
  int i, result;  
  
  /*
  sensor_data_t sensor_data;
  sensor_data_t sensor_data_empty;  
  memset(&sensor_data_empty, 0, sizeof(sensor_data_t));
  
  // test MODE_OFF
  result = Pozyx.setSensorMode(1);
  assertEqual(result, POZYX_SUCCESS);  
  delay(5000); 
  result = Pozyx.getSensorMode(&sensor_mode);
  assertEqual(result, POZYX_SUCCESS);
  delay(10);      
  
  memset(&sensor_data, 0, sizeof(sensor_data_t));
  Pozyx.getAllSensorData(&sensor_data);
  result = memcmp(&sensor_data+4, &sensor_data_empty+4, sizeof(sensor_data_t)-4);
  
  for(i=1; i<(sizeof(sensor_data_t)-2)/4; i++)
  {
    Serial.print(*((float32_t*)&sensor_data+i));
    Serial.print(" ");    
  }  
  */  
  
  // test the other modes
  for(i = 0; i<13; i++)
  {
    result = Pozyx.setSensorMode(sensor_modes[i]);
    assertEqual(result, POZYX_SUCCESS);
    delay(10);    
    result = Pozyx.getSensorMode(&sensor_mode);
    assertEqual(result, POZYX_SUCCESS);
    
    assertEqual(sensor_mode, sensor_modes[i]);    
  } 
}
