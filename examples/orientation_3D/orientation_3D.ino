/**
  The pozyx ranging demo (c) Pozyx Labs
  please check out https://www.pozyx.io/Documentation/Tutorials/getting_started/Arduino
  
  This demo requires one (or two) pozyx shields and one Arduino. It demonstrates the 3D orientation and the functionality
  to remotely read register data from a pozyx device. Place one of the pozyx shields on the Arduino and upload this sketch. 
  
  This demo reads the following sensor data: 
  - pressure
  - acceleration
  - magnetic field strength
  - angular velocity
  - the heading, roll and pitch
  - the quaternion rotation describing the 3D orientation of the device. This can be used to transform from the body coordinate system to the world coordinate system.
  - the linear acceleration (the acceleration excluding gravity)
  - the gravitational vector
  
  The data can be viewed in the Processing sketch orientation_3D.pde 
*/

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

boolean remote = false;               // boolean to indicate if we want to read sensor data from the attached pozyx shield (value 0) or from a remote pozyx device (value 1)
uint16_t remote_id = 0x6606;          // the network id of the other pozyx device: fill in the network id of the other device
uint32_t last_millis;                 // used to compute the measurement interval in milliseconds 

////////////////////////////////////////////////

void setup()
{  
  Serial.begin(115200);
    
  if(Pozyx.begin(false, MODE_INTERRUPT, POZYX_INT_MASK_IMU) == POZYX_FAILURE){
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    abort();
  }

  if(!remote)
    remote_id = NULL;
  
  last_millis = millis();
  delay(10);  
}

void loop(){
  sensor_raw_t sensor_raw;
  uint8_t calibration_status = 0;
  int dt;
  int status;
  if(remote){
     status = Pozyx.getRawSensorData(&sensor_raw, remote_id);
     status &= Pozyx.getCalibrationStatus(&calibration_status, remote_id);
    if(status != POZYX_SUCCESS){
      return;
    }
  }else{
    if (Pozyx.waitForFlag(POZYX_INT_STATUS_IMU, 10) == POZYX_SUCCESS){
      Pozyx.getRawSensorData(&sensor_raw);
      Pozyx.getCalibrationStatus(&calibration_status);
    }else{
      uint8_t interrupt_status = 0;
      Pozyx.getInterruptStatus(&interrupt_status);
      return;
    }
  }

  dt = millis() - last_millis;
  last_millis += dt;    
  // print time difference between last measurement in ms, sensor data, and calibration data
  Serial.print(dt, DEC);
  Serial.print(",");
  printRawSensorData(sensor_raw);
  Serial.print(",");
  // will be zeros for remote devices as unavailable remotely.
  printCalibrationStatus(calibration_status);
  Serial.println();
}

void printRawSensorData(sensor_raw_t sensor_raw){
  Serial.print(sensor_raw.pressure);
  Serial.print(",");
  Serial.print(sensor_raw.acceleration[0]);
  Serial.print(",");
  Serial.print(sensor_raw.acceleration[1]);
  Serial.print(",");
  Serial.print(sensor_raw.acceleration[2]);
  Serial.print(",");
  Serial.print(sensor_raw.magnetic[0]);
  Serial.print(",");
  Serial.print(sensor_raw.magnetic[1]);
  Serial.print(",");
  Serial.print(sensor_raw.magnetic[2]);
  Serial.print(",");
  Serial.print(sensor_raw.angular_vel[0]);
  Serial.print(",");
  Serial.print(sensor_raw.angular_vel[1]);
  Serial.print(",");
  Serial.print(sensor_raw.angular_vel[2]);
  Serial.print(",");
  Serial.print(sensor_raw.euler_angles[0]);
  Serial.print(",");
  Serial.print(sensor_raw.euler_angles[1]);
  Serial.print(",");
  Serial.print(sensor_raw.euler_angles[2]);
  Serial.print(",");
  Serial.print(sensor_raw.quaternion[0]);
  Serial.print(",");
  Serial.print(sensor_raw.quaternion[1]);
  Serial.print(",");
  Serial.print(sensor_raw.quaternion[2]);
  Serial.print(",");
  Serial.print(sensor_raw.quaternion[3]);
  Serial.print(",");
  Serial.print(sensor_raw.linear_acceleration[0]);
  Serial.print(",");
  Serial.print(sensor_raw.linear_acceleration[1]);
  Serial.print(",");
  Serial.print(sensor_raw.linear_acceleration[2]);
  Serial.print(",");
  Serial.print(sensor_raw.gravity_vector[0]);
  Serial.print(",");
  Serial.print(sensor_raw.gravity_vector[1]);
  Serial.print(",");
  Serial.print(sensor_raw.gravity_vector[2]);
  Serial.print(",");
  Serial.print(sensor_raw.temperature);
}

void printCalibrationStatus(uint8_t calibration_status){
  Serial.print(calibration_status & 0x03);
  Serial.print(",");
  Serial.print((calibration_status & 0x0C) >> 2);
  Serial.print(",");
  Serial.print((calibration_status & 0x30) >> 4);
  Serial.print(",");
  Serial.print((calibration_status & 0xC0) >> 6);  
}

