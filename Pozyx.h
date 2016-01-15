/**
* Pozyx.h
* -------
* This file is a definition of all structures, classes and functions used in the
* POZYX environment.
*
* Each function is described:
*   - which input parameters are expected
    - what is the behaviour
    - and what is the expected output
*
*/

#ifndef POZYX_h
#define POZYX_h

#include <inttypes.h>

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

extern "C" {
  #include "Pozyx_definitions.h"
}


// SHOULD BE MOVED TO DEFINATIONS !!!
#define MODE_POLLING              0
#define MODE_INTERRUPT            1



/**
* POZYX STRUCTURES
* ----------------
*/

/**
* UMB_settings_t
* --------------
* The UWB settings type defines all attributes needed to set the UWB (communication) parameters
*/
typedef struct __attribute__((packed))_UWB_settings {    
    uint8_t channel;
    unsigned bitrate:6;
    unsigned prf:2;
    uint8_t plen;
    int8_t gain;
    unsigned trim:8;
}UWB_settings_t;

/**
* coordinates_t
* -------------
* The coordinates type defines the coordinates of position result or anchor location
*/
typedef struct __attribute__((packed))_coordinates {    
    int32_t x;
    int32_t y;
    int32_t z;
}coordinates_t;

/**
* pos_error_t
* -----------
* The position error type gives the resulting error covariance for a given position result
*/
typedef struct __attribute__((packed))_pos_error {    
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t xy;
    int16_t xz;
    int16_t yz;
}pos_error_t;

/**
* acceleration_t
* --------------
* The aceleration type can be used to store the acelerometer data
*/
typedef struct __attribute__((packed))_acceleration {    
    int16_t x;
    int16_t y;
    int16_t z;
}acceleration_t;

/**
* magnetic_t
* ----------
* The magnetic type can be used to store the magnetometer data
*/
typedef struct __attribute__((packed))_magnetic {    
    int16_t x;
    int16_t y;
    int16_t z;
}magnetic_t;

/**
* gyro_t
* ------
* The gyro type can be used to store the gyroscope data
*/
typedef struct __attribute__((packed))_gyro {    
    int16_t x;
    int16_t y;
    int16_t z;
}gyro_t;

/**
* euler_angles_t
* --------------
* The euler angels type gives the resulting euler coordinates for the state of the IMU
*/
typedef struct __attribute__((packed))_euler_angles {    
    int16_t heading;
    int16_t roll;
    int16_t pitch;
}euler_angles_t;

/**
* quaternion_t
* ------------
* The quaternion type gives the resulting quaternions for the IMU
*/
typedef struct __attribute__((packed))_quaternion {    
    int16_t weight;
    int16_t x;
    int16_t y;
    int16_t z;
}quaternion_t;

/**
* linear_acceleration_t
* ---------------------
* The linear acceleration type gives the resulting linear accelartion in each direction
*/
typedef struct __attribute__((packed))_linear_acceleration {    
    int16_t x;
    int16_t y;
    int16_t z;
}linear_acceleration_t;

/**
* gravity_vector_t
* ----------------
* The gravity vector type gives the resulting gravity vector for each component
*/
typedef struct __attribute__((packed))_gravity_vector {    
    int16_t x;
    int16_t y;
    int16_t z;
}gravity_vector_t;

/**
* sensor_data_t
* -------------
* The sensor data type allows to read the whole sensor data in one datastructure with one call
*/
typedef struct __attribute__((packed))_sensor_data { 
    uint32_t pressure;
    acceleration_t acceleration;
    magnetic_t magnetic;
    gyro_t gyro;
    euler_angles_t euler_angles;
    quaternion_t quaternion;
    linear_acceleration_t linear_acceleration;
    gravity_vector_t gravity_vector;
    int8_t temprature;   
}sensor_data_t;

/**
* device_coordinates_t
* --------------------
* The device coordinates type is used to easily add a device to the POZYX system
*/
typedef struct __attribute__((packed))_device_coordinates { 
    uint16_t network_id;
    uint8_t flag;
    coordinates_t pos;
}device_coordinates_t;

/**
* device_range_t
* --------------
* The device range type stores all the attributes linked to a range measurement
*/
typedef struct __attribute__((packed))_device_range {       
    uint32_t timestamp;
    uint32_t distance;
    int8_t RSS;
}device_range_t;

/**
* device_info_t
* -------------
* The device info type is used to retrieve all information of a device in the POZYX system
*/
typedef struct __attribute__((packed))_device_info {
    uint16_t network_id;
    uint8_t flag;
    coordinates_t pos;
    device_range_t range;    
}device_info_t;


/**
* POZYX CLASS
* -----------
*/
class PozyxClass
{
private:
    static int _mode;               // the mode of operation, can be MODE_INTERRUPT or MODE_POLLING
    static int _interrupt;          // variable to indicate that an interrupt has occured


    static int _hw_version;         // POZYX harware version 
    static int _sw_version;         // POZYX software (firmware) version. (By updating the firmware on the pozyx device, this value can change)
   

    /**
    * Function: i2cWriteWrite
    * -----------------------
    * Internal function that writes a number of bytes to a specfified POZYX register 
    * This function implements the I2C interface as described in the datasheet on our website
    * Input values:
    *   uint8_t reg_address: POZYX address to write to
    *   uint8_t *pData: reference to the data that needs to be writen
    *   int size: size in byte of data to be written
    *   
    * Output:
    *   POZYX_FAILURE: error occured during the process
    *   POZYX_SUCCESS: successful execution of the function
    */
    static int i2cWriteWrite(const uint8_t reg_address, const uint8_t *pData, int size);
       
    /**
    * Function: void IRQ
    * ------------------
    * Internal function that sets the _interrupt variable on an Arduino interrupt
    */
    static void IRQ(); 
   

public:
    /**
    * Function: i2cWriteRead
    * ----------------------
    * Internal function that writes and reads a number of bytes to call a specfic POZYX register function
    * This function implements the I2C interface as described in the datasheet on our website
    * Input values:
    *   uint8_t *write_data: reference to the data that needs to be writen
    *   int write_len: size in byte of data to be written
    *   uint8_t *read_data: reference to the pointer where the read data should be stored
    *   int read_len: size in byte of data to be read
    *   
    * Output:
    *   POZYX_FAILURE: error occured during the process
    *   POZYX_SUCCESS: successful execution of the function
    */
    static int i2cWriteRead(uint8_t* write_data, int write_len, uint8_t* read_data, int read_len);

    /**
    * Function: boolean waitForFlag
    * -----------------------------
    * Function that waits until the Pozyx shields has raised a specfic event flag until a given timeout
    * Input values:
    *   uint8_t interrupt_flag: the exepected Pozyx interrupt
    *   int timeout_ms: maximum waiting time for flag to occur
    *
    * Output:
    *   true: the interrupt occured in the given timeout window
    *   false: timeout occured
    */
    static boolean waitForFlag(uint8_t interrupt_flag, int timeout_ms);     

    /**
    * Function: boolean begin
    * -----------------------
    * Initaties the POZYX shields 
    * Input values:
    *   boolean print_result (default: false): outputs the result of the function to the Serial output
    *   int mode (default: MODE_INTERRUPT): The modus of the system: MODE_POLLING or MODE_INTERRUPT
    *   int interrupts (default: POZYX_INT_MASK_ALL): defines for MODE_INTERRUPT which interrupts are triggered
    *   int interrupt_pin (default: POZYX_INT_PIN0): POZYX interrupt pin: POZYX_INT_PIN0 or POZYX_INT_PIN1
    *
    * Output:
    *   POZYX_FAILURE: error occured during the process
    *   POZYX_SUCCESS: successful execution of the function
    */
    static int begin(boolean print_result = false, int mode = MODE_INTERRUPT,  int interrupts = POZYX_INT_MASK_ALL, int interrupt_pin = POZYX_INT_PIN0);
   
    /**
    * Functions: int regRead / regWrite / regFunction
    * -----------------------------------------------
    * Functions to read/write the registers of the connected POZYX shield.
    * reg_function allows to call the functions as described in the datasheet.
    * 
    * This functions are used to raw read/write the pozyx device using the register addresses
    * Input values:
    *   uint8_t reg_address: the specific address for read, write or function call
    *   uint8_t *buffer: this is the pointer where the result of the call is stored
    *   uint8_t *pData: this is data that needs to be writen to the Pozyx register
    *   uint8_t *params: this is the pointer to where the parameters are stored
    *   int size: the size of passed variables in byte
    *
    * Output:
    *   POZYX_FAILURE: error occured during the process
    *   POZYX_SUCCESS: successful execution of the function
    */
    static int regRead(uint8_t reg_address, uint8_t *buffer, int size);
    static int regWrite(uint8_t reg_address, const uint8_t *pData, int size);
    static int regFunction(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *buffer, int size);

    /**
    * Functions: int sendData / remoteRegRead / remoteRegWrite / remoteRegDunction
    * ----------------------------------------------------------------------------
    * Functions that allow read/write registers and function calls of a remote Pozyx shield using UWB-signal
    * Note: The remote shield must use the same UWB-settings in order to receive the requests
    * 
    * The send_data is used to write data to the RX-buffer of the destination shield.
    *
    * Input parameters:
    *   uint16_t destination: this is the networkid of the receiving Pozyx tag
    *   uint8_t data[]: data array tobe send
    *   uint8_t reg_address: the specific address for read, write or function call
    *   uint8_t *buffer: this is the pointer where the result of the call is stored
    *   uint8_t *pData: this is data that needs to be writen to the Pozyx register
    *   uint8_t *params: this is the pointer to where the parameters are stored
    *   int size: the size of passed variables in byte
    *
    * The return value is the same as for the reg_write_read function that implements the I2C communciation
    * For reg_function the return value return the status of the called function
    * see below for detailed description of each function
    *
    * Note: It's important to note that in most cases only one master tag can be active in a given
    * UWB-setting as multiple simultanious requests can collide and cause loss of data
    */
    static int sendData(uint16_t destination, uint8_t data[], int size);
    static int remoteRegWrite(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size);
    static int remoteRegRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size);
    static int remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size);
    
    /**
    * Functions: int writeTXBufferData / sendTXBufferData /readRXBufferData
    * ---------------------------------------------------------------------
    * Functions for remote comunication over UWB
    *
    * Usage:
    *   use writeTXBufferData to write data to the buffer that will be send
    *   use sendTXBuferData to transmit the data to destination. By default the message will be broadcasted
    *   to all in range devices
    *   If in interrupt modus the POZYX_INT_STATUS_RX_DATA will be triggered and the data
    *   can be read via readRXBufferData (see datasheet for more information)
    *
    * Input parameters:
    *   uint8 data[]: data array to put on the buffer
    *   int size: size of the data array
    *   int offset default(0): The offset in the memory
    *   uint16_t destination default(0x0): the destination to whom the message should be sent
    *   uint8_t* pData: pointer to data where receiving data will be stored
    */
    static int writeTXBufferData(uint8_t data[], int size, int offset = 0);
    static int sendTXBufferData(uint16_t destination = 0x0);
    static int readRXBufferData(uint8_t* pData, int size);


    /**
    * General functions accessing the POZYX Shield
    *
    * All functions have the following return values:
    * Output:
    *   POZYX_FAILURE: error occured during the process
    *   POZYX_SUCCESS: successful execution of the function
    *   POZYX_TIMEOUT: In case of mode is MODE_INTERRUPT and the function waitForFlag returns false
    *
    * Note: For all get functions a pointer is given as parameter to store the value
    */
    
    /**
    *
    * STATUS REGISTERS
    *
    */
    /**
    * Functions: int getWhoAmI / getFirmwareVersion / getHardwareVersion / getSelftest / getErrorCode / getCalibrationStatus
    * ----------------------------------------------------------------------------------------------------------------------
    * These functions return the status value of the read registers. The values can be interpreted via de datasheet online
    *
    * Input parameters:
    *   uint8_t* 'data': reference to the pointer where the read data should be stored e.g. whoami
    *   uint16_t remote_id: optional parameter that determines the remote device to be read
    */
    static int getWhoAmI(uint8_t *whoami, uint16_t remote_id = NULL);
    static int getFirmwareVersion(uint8_t *firmware, uint16_t remote_id = NULL);
    static int getHardwareVersion(uint8_t *hardware, uint16_t remote_id = NULL);
    static int getSelftest(uint8_t *selftest, uint16_t remote_id = NULL);
    static int getErrorCode(uint8_t *error_code, uint16_t remote_id = NULL);
    static int getInterruptStatus(uint8_t *interrupts, uint16_t remote_id = NULL);
    static int getCalibrationStatus(uint8_t *calibration_status, uint16_t remote_id = NULL);

    /**
    *
    * CONFIGURATION REGISTERS
    *
    */
    /**
    * Functions: int getInterruptMask / setInterruptMask 
    * --------------------------------------------------
    * Function to get or set the interrupt mask. The interrupt mask will be used to determine which interrupts need to be triggered
    *
    * Input parameters:
    *   uint8_t* mask: reference to mask to be written
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getInterruptMask(uint8_t *mask, uint16_t remote_id = NULL);
    static int setInterruptMask(uint8_t mask, uint16_t remote_id = NULL);

    /**
    * Functions: int setInterrupt / unSetInterrupt 
    * --------------------------------------------
    * Function to get or set a specific interrupt. The interrupt will be added/removed from the curren mask
    *
    * Input parameters:
    *   uint8_t* interrupt: reference to interrupt to be written
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int setInterrupt(uint8_t *interrupt,  uint16_t remote_id = NULL);
    static int unSetInterrupt(uint8_t *interrupt,  uint16_t remote_id = NULL);

    /**
    * Functions: int getUpdateInterval / setUpdateInterval 
    * ----------------------------------------------------
    * If this function is used and the interval parameter is set, the system will operate in continuous localisation mode
    *
    * Input parameters:
    *   uint8_t* ms: update interval in milliseconds
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getUpdateInterval(uint16_t *ms, uint16_t remote_id = NULL);
    static int setUpdateInterval(uint16_t ms, uint16_t remote_id = NULL);

    /**
    * Functions: int getConfigModeGPIO / getConfigPullGPIO / setConfigGPIO 
    * --------------------------------------------------------------------
    * Functions to set the configuration modus of the GPIO for the given pin number
    *
    * Input parameters:
    *   int gpio_num: the pin to update
    *   int mode: the mode of GPIO (see datasheet)
    *   int pull: pull configuration of GPIO (see datasheet)
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    * Output:
    *   getConfigModeGPIO -> uint8_t *mode: the mode of GPIO (see datasheet)
    *   getConfigPullGPIO -> uint8_t *pull: pull configuration of GPIO (see datasheet)
    */
    static int getConfigModeGPIO(int gpio_num, uint8_t *mode, uint16_t remote_id = NULL);
    static int getConfigPullGPIO(int gpio_num, uint8_t *pull, uint16_t remote_id = NULL);
    static int setConfigGPIO(int gpio_num, int mode, int pull, uint16_t remote_id = NULL);

    /**
    * Functions: int setLedConfig 
    * ---------------------------
    * Functions the operation modus of the leds
    *
    * Usage:
    *   setLedConfig(POZYX_LED_CTRL_LED3 || POZYX_LED_CTRL_LED2) if you want to control
    *   setLedConfig() to restore the default and give control of the leds to the POZYX system
    *
    * Input parameters:
    *   int gpio_num: the gpio to be set or retrieved
    *   uint8_t config default(0x0): the configuration to be set
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int setLedConfig(uint8_t config = 0x0, uint16_t remote_id = NULL);

    /**
    * Functions: int getPositionAlgorithm / getPositionDimension / setPositionAlgorithm
    * ---------------------------------------------------------------------------------
    * Functions to set the position algorithm
    *
    * Input parameters:
    *   int algorithm default(POZYX_POS_ALG_UWB_ONLY): algorithm used to determine position
    *   int dimension default(2D): dimension used for the algorithm
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    * Output:
    *   getPositionAlgorithm -> uint8_t *algorithm: the algorithm currently used (see datasheet)
    *   getPositionDimension -> uint8_t *dimension: the demension currently used in positioning (see datasheet)
    */
    static int getPositionAlgorithm(uint8_t *algorithm, uint16_t remote_id = NULL);
    static int getPositionDimension(uint8_t *dimension, uint16_t remote_id = NULL);
    static int setPositionAlgorithm(int algorithm = POZYX_POS_ALG_UWB_ONLY, int dimension = 0x0, uint16_t remote_id = NULL);
    
    /**
    * Functions: int getNumberOfAnchors / getAnchorMode / setSelectionOfAnchors
    * -------------------------------------------------------------------------
    * Functions to set the position algorithm
    *
''    * Input parameters:
    *   int mode: the selection mode of the anchors (see datasheet)
    *   int nr_anchors: the number of anchors used for positioning (see datasheet)
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    * Output:
    *   getAnchorMode -> uint8_t *mode: the selection mode of the anchors
    *   getNumberOfAnchors -> uint8_t *nr_anchors: the number of anchors used in positioning
    */
    static int getAnchorMode(uint8_t *mode, uint16_t remote_id = NULL);
    static int getNumberOfAnchors(uint8_t *nr_anchors, uint16_t remote_id = NULL);
    static int setSelectionOfAnchors(int mode, int nr_anchors, uint16_t remote_id = NULL);

    /**
    * Functions: int getNetworkId / setNetworkId
    * ------------------------------------------
    * The network id is determined based on the hardware components. When the system is reset the orignal value is restored
    * This function can be used to manually set thet network id
    *
    * Input parameters:
    *   uint8_t* network_id: reference to the network_id pointer to store the data
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getNetworkId(uint16_t *network_id);
    static int setNetworkId(uint16_t network_id, uint16_t remote_id = NULL);

    /**
    * Functions: int getUWBSettings / setUWBSettings
    * ----------------------------------------------
    * Functions to retrieve and set the UWB settings
    *
    * Input parameters:
    *   uint8_t* UWB_settings: reference to the UWB settings object to store the data
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getUWBSettings(UWB_settings_t *UWB_settings, uint16_t remote_id = NULL);
    static int setUWBSettings(UWB_settings_t UWB_settings, uint16_t remote_id = NULL);

    /**
    * Functions: int getOperationMode / setOperationMode
    * --------------------------------------------------
    * Functions to get/set the operating modus of the device
    *
    * Input parameters:
    *   uint8_t* mode: The mode of operations POZYX_ANCHOR_MODE or POZYX_TAG_MODE
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getOperationMode(uint8_t *mode, uint16_t remote_id = NULL);
    static int setOperationMode(uint8_t mode, uint16_t remote_id = NULL);
    
    /**
    * Functions: int getSensorMode / setSensorMode
    * --------------------------------------------
    * Functions to retrieve and set the IMU sensor modus (see datasheet)
    *
    * Input parameters:
    *   uint8_t* sensor_mode: reference to the pointer for the sensor mode
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getSensorMode(uint8_t *sensor_mode, uint16_t remote_id = NULL);
    static int setSensorMode(uint8_t sensor_mode, uint16_t remote_id = NULL);

    
    /**
    *
    * POSITION DATA
    *
    */
    /**
    * Functions: int getCoordinates / setCoordinates / getPositionError
    * -----------------------------------------------------------------
    * Functions to retrieve the last coordinates of the device or the remote device
    * The function position error returns the estimated covariance
    *
    * The coordinates can be manual set to fix coordinates for a given device
    *
    * Input parameters:
    *   coordinates_t* coordinates: reference to the coordinates pointer object
    *   pos_error_t* pos_error: reference to the pos error object
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getCoordinates(coordinates_t *coordinates, uint16_t remote_id = NULL);
    static int setCoordinates(coordinates_t coordinates, uint16_t remote_id = NULL);
    static int getPositionError(pos_error_t *pos_error, uint16_t remote_id = NULL);


    /**
    *
    * SENSOR DATA
    *
    */
    /**
    * Functions: int getSensorData / getPressure / getAcceleration / getMagnetic / getGyro / getEulerAngles
    *                getQuaternion / getLinearAcceleration / getGravityVector / getTemperature
    * -----------------------------------------------------------------------------------------------------
    * The following functions allow the user to retrieve the sensor data
    * The sensor data is updated upon the POZYX_INT_STATUS_IMU interrupt
    * With the function getSensorData all of the sensor data can be retrieved in one call
    */
    static int getSensorData(sensor_data_t *sensor_data, uint16_t remote_id = NULL);
    static int getPressure(uint32_t *pressure, uint16_t remote_id = NULL);
    static int getAcceleration(acceleration_t *acceleration, uint16_t remote_id = NULL);
    static int getMagnetic(magnetic_t *magnetic, uint16_t remote_id = NULL);
    static int getGyro(gyro_t *gyro, uint16_t remote_id = NULL);
    static int getEulerAngles(euler_angles_t *euler_angles, uint16_t remote_id = NULL);
    static int getQuaternion(quaternion_t *quaternion, uint16_t remote_id = NULL);
    static int getLinearAcceleration(linear_acceleration_t *linear_acceleration, uint16_t remote_id = NULL);
    static int getGravityVector(gravity_vector_t *gravity_vector, uint16_t remote_id = NULL);
    static int getTemperature(int8_t *temperature, uint16_t remote_id = NULL);


    /**
    *
    * GENERAL DATA
    *
    */
    /**
    * Functions: int getDeviceListSize
    * --------------------------------
    * The following functions allow the user to retrieve the number of other devices registered for the given device
    * The sensor data is updated upon the POZYX_INT_STATUS_IMU interrupt
    * With the function getSensorData all of the sensor data can be retrieved in one call
    *
    * Input parameters:
    *   uint8_t *device_list_size: the pointer that stores the device list size
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getDeviceListSize(uint8_t *device_list_size, uint16_t remote_id = NULL);

    /**
    * Functions: int getLastNetworkId / getLastDataLength
    * ---------------------------------------------------
    * These functions allow to retrieve who was the last device that has send data
    * The funtion getLastDataLength returns the length in bytes of the last received UWB-message
    *
    * Input parameters:
    *   uint8_t *device_list_size: the pointer that stores the device list size
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getLastNetworkId(uint16_t *network_id, uint16_t remote_id = NULL);
    static int getLastDataLength(uint8_t *data_length, uint16_t remote_id = NULL);

    /**
    * Functions: int getGPIO / setGPIO
    * --------------------------------
    * Functions to retieve or set the value of the given GPIO on the target device 
    *
    * Input parameters:
    *   int gpio_num: the gpio to be set or retrieved
    *   uint8_t *value: the pointer that stores the value for the GPIO
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getGPIO(int gpio_num, uint8_t *value, uint16_t remote_id = NULL);
    static int setGPIO(int gpio_num, uint8_t value, uint16_t remote_id = NULL);


    /**
    *
    * GENERAL FUNCTIONS
    *
    */
     /**
    * Functions: int resetSystem
    * --------------------------
    * Function that will trigger the reset of the  system
    * Note: this will reset all configuration values to the default values
    *
    * Input parameters:
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static void resetSystem(uint16_t remote_id = NULL);


    /**
    * Functions: int setLed
    * ---------------------
    * Function for turning the leds on and off
    *
    * Input parameters:
    *   int led_num: the led to be controlled
    *   boolean state: the state to set the selected led to
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int setLed(int led_num, boolean state, uint16_t remote_id = NULL);

    /**
    * Functions: int doRanging
    * ------------------------
    * One of the core functions to allow you to do ranging between current device and a destination device
    *
    * Input parameters:
    *   uint16_t destination: the target device to do ranging with 
    *   device_range_t *range: the pointer to where the resulting data will be stored
    */
    static int doRanging(uint16_t destination, device_range_t *range);

    /**
    * Functions: int doRemoteRanging
    * ------------------------------
    * Function allowing to trigger ranging between two remote devices
    *
    * Input parameters:
    *   uint16_t device_from: device that will initiate the ranging call
    *   uint16_t device_to: device to which the ranging will be exectued
    *   device_range_t *range: the pointer to where the resulting data will be stored
    */
    static int doRemoteRanging(uint16_t device_from, uint16_t device_to, device_range_t *device_range);

    /**
    * Functions: int doPositioning
    * ------------------------------
    * Based on the position algorithm set the linked device will determine it's position based on the anchors set
    *
    * Input parameters:
    *   coordinates_t *position: data object to store the result
    */
    static int doPositioning(coordinates_t *position, uint8_t dimension = POZYX_2D, int32_t height = 0, uint8_t algorithm = 0);

    /**
    * Functions: int doRemotePositioning
    * ----------------------------------
    * Triggering the position algorithm on a remote device
    *
    * Input parameters:
    *   uint16_t remote_id: the remote device that will do the positioning
    *   coordinates_t *position: data object to store the result
    */
    static int doRemotePositioning(uint16_t remote_id, coordinates_t *coordinates);

    /**
    * Functions: int setPositioningAnchorIds / getPositioningAnchorIds
    * ----------------------------------------------------------------
    * Functions that determines which anchors to use for positioning based on the parameters set in
    * POZYX_POS_NUM_ANCHORS
    *
    * Input parameters:
    *   uint16_t anchors[]: the array of anchors to be used
    *   int anchor_num: the number of anchors
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int setPositioningAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id = NULL);
    static int getPositioningAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id = NULL);

    /**
    *
    * DEVICE LIST FUNCTIONS
    *
    */
    /**
    * Functions: int getDeviceIds / getAnchorIds / getTagIds
    * ------------------------------------------------------
    * Function to get all the network ids of the devices in the device list
    * Note: it can be unclear how long the device list is
    *
    * Input parameters:
    *   uint16_t devices[]: the array of anchors to be used
    *   int size default(MAX_ANCHORS_IN_LIST): the number of devices
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getDeviceIds(uint16_t devices[], int size = MAX_ANCHORS_IN_LIST, uint16_t remote_id = NULL);
    static int getAnchorIds(uint16_t anchors[], int size = MAX_ANCHORS_IN_LIST, uint16_t remote_id = NULL);
    static int getTagIds(uint16_t tags[], int size = MAX_ANCHORS_IN_LIST, uint16_t remote_id = NULL);

    /**
    * Functions: int doDiscovery
    * --------------------------
    * Function to discover anchors/tags/all POZYX devices in range of the master tag
    *
    * Input parameters:
    *   int type default(0x0): which type of device to discover, default only anchors
    *   int slots default(3): The number of slots to wait for a response of an undiscovered device
    *   int slot_duration default(10): Time duration of an idle slot
    */
    static int doDiscovery(int type = 0x0, int slots = 3, int slot_duration = 10);

    /**
    * Functions: int doAnchorCalibration
    * ----------------------------------
    * Function to auto calibrate the anchors for positioning
    *
    * Input parameters:
    *   int type option(POZYX_2D): type of algorithm for calibration 
    *   uint16_t anchors[] default(NULL): The anchors that determine the axis (see datasheet)
    *   int anchor_num default(0): The number of anchors in the above list
    *   int measurements default(10): The number of measurements per range
    */
    static int doAnchorCalibration(int dimension = POZYX_2D, int num_anchors = 0, int num_measurements = 10, uint16_t anchors[] = NULL,  int32_t heights[] = NULL);
        
    /**
    * Functions: int updateRemoteTags
    * -------------------------------
    * Function to update remote tags with the same positioning parameters as the master tag 
    * This functions allow easy setup of remote tags instead of configuring one by one
    *
    * Input parameters:
    *   uint16_t tags[] default(NULL): The tags that need to be updated, if none are provide all tags in the device list will be updated
    *   int tags_num default(0): The number of tags in the above list
    */
    static int updateRemoteTags(uint16_t tags[] = NULL, int tags_num = 0);

    /**
    * Functions: int clearDevices 
    * ---------------------------
    * This function empties the internal list of devices
    *
    * Input parameters:
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int clearDevices(uint16_t remote_id = NULL);

    /**
    * Functions: int addDevice 
    * ------------------------
    * Manualy adds a devices, this function can be used instead of the doAnchorCalibration or doDiscovery
    *
    * Input parameters:
    *   device_coordinates_t device_coordinates: the device information to be added
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int addDevice(device_coordinates_t device_coordinates, uint16_t remote_id = NULL);

    /**
    * Functions: int getDeviceInfo / getDeviceCoordinates / getDeviceRangeInfo
    * ------------------------------------------------------------------------
    * Functions to retrieve specific device information, localy or remote
    *
    * Input parameters:
    *   uint16_t device_id: device from which the information needs to be retrieved
    *   device_info_t *device_info: data object to store the information
    *   device_coordinates_t *device_coordinates: data object to store the information
    *   device_range_t *device_range: data object to store the information
    *   uint16_t remote_id: optional parameter that determines the remote device to be used
    */
    static int getDeviceInfo(uint16_t device_id, device_info_t *device_info, uint16_t remote_id = NULL);
    static int getDeviceCoordinates(uint16_t device_id, coordinates_t *coordinates, uint16_t remote_id = NULL);
    static int getDeviceRangeInfo(uint16_t device_id, device_range_t *device_range, uint16_t remote_id = NULL);

};
extern PozyxClass Pozyx;


#endif
