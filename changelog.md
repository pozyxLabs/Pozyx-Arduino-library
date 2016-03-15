# Arduino Pozyx library change log

## Version 1.0.1
- Added function documentation for doxygen
- Added defines for parameter descriptions
- Added assertions to all functions for checking API function parameters
- Added function getErrorCode
- Added function setTxPower, getTxPower
- Added function setUWBChannel, getUWBChannel
- Added unit tests for platform specific functions
- The sensor functions now return the sensor values in metric numbers instead of raw numbers
- Removed spurious Serial.print lines 
- Rename function getAcceleration to getAcceleration_mg
- Rename function getGyro to getAngularVelocity_dps
- Rename function getEulerAngles to getEulerAngles_deg
- Rename function getGravity to getGravityVector_mg
- Rename function getLinearAcceleration to getLinearAcceleration_mg
- Rename function getSensorData to getAllSensorData
- Rename function getPressure to getPressure_Pa
- Rename function getMagnetic to getMagnetic_uT
- Rename function getTemperature to getTemperature_c

## Version 1.0.0
- initial release
