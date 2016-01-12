/**
* Pozyx.h
* -------
* This file is a defination of all structures, classes and functions used in the
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
    static int _mode;              // the mode of operation, can be MODE_INTERRUPT or MODE_POLLING
    static int _interrupt;        // variable to indicate that an interrupt has occured


    static int _hw_version;       // pozyx harware version 
    static int _sw_version;       // pozyx software (firmware) version. (By updating the firmware on the pozyx device, this value can change)
   


    /*
    * Function: i2cWriteWrite
    * -----------------------
    * Internal function that writes a number of bytes to a specfified POZYX register 
    * Implements the I2C interface as described in the datasheet on our website
    * Input values:
    *   uint8_t reg_address: to POZYX address to write to
    *   
    * The 
    *     0: SUCCESS
    *     -10/-20: error - wire write transmission error
    *     pos. value: error - wire end transmission error
    *     -11/-21: error - available length on wire not the same as provided 
    */

    static int i2cWriteWrite(const uint8_t reg_address, const uint8_t *pData, int size);

/**
  * Call a register function using I2C with given parameters
  */
/**
  * Writes a number of bytes to the specified pozyx register address using I2C
  */
    
   


    static void IRQ(); 
   /*
    * Function: void IRQ
    * ------------------
    * Internal function that sets the _interrupt variable on an Arduino interrupt
    */


public:

    static int i2cWriteRead(uint8_t* write_data, int write_len, uint8_t* read_data, int read_len);

    static boolean waitForFlag(uint8_t interrupt_flag, int timeout_ms);    
    /*
    * Function: boolean waitForFlag
    * -----------------------------
    * Function that waits until the Pozyx shields has raised a specfic event flag until a given timeout
    *     uint8_t interrupt_flag: the exepected Pozyx interrupt
    *     int timeout_ms: maximum waiting time
    *
    * Returns true if the interrupt was raised in the timeout frame
    */


    static boolean begin(boolean print_result = false, int mode = MODE_INTERRUPT, int interrupts = POZYX_INT_MASK_ALL, int interrupt_pin = 0x0);
   /* Function: boolean begin
    * -----------------------
    * Initaties the Pozyx shields following input values can be given (optional)
    *     boolean: print_result outputs the result of begin to the Serial output
    *     int mode: system can be used in (0) MODE_POLLING or (1) MODE_INTERRUPT (default)
    *     int interrupts: defines for MODE_INTERRUPT which interrupts are enabled (default POZYX_INT_MASK_ALL)
    *     int interrupt_pin: default pin (0), pin (1) is also supported
    *
    * Returns false if the following errors occured:
    *     whoami register is incorrect
    *     selftest incorrect based on the HW-version
    */


    static int regRead(uint8_t reg_address, uint8_t *buffer, int size);
    static int regWrite(uint8_t reg_address, const uint8_t *pData, int size);
    static int regFunction(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *buffer, int size);
    /*
    * Functions: int regRead / regWrite / regFunction
    * -----------------------------------------------
    * Functions to read/write the registers of the connected Pozyx shield.
    * reg_function allows to call the functions as described in the datasheet.
    * 
    * This functions are used to raw read/write the pozyx device using the register addresses
    *
    *     uint8_t reg_address: the specific address for read, write or function call
    *     uint8_t *buffer: this is the pointer where the result of the call is stored
    *     uint8_t *pData: this is data that needs to be writen to the Pozyx register
    *     uint8_t *params: this is the pointer to where the parameters are stored
    *     int size: the size of passed variables in byte
    *
    * The return value is the same as for the reg_write_read function that implements the I2C communciation
    * For reg_function the return value return the status of the called function
    * see below for detailed description of each function.
    */


  
    static int sendData(uint16_t destination, uint8_t data[], int size);
    static int remoteRegWrite(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size);
    static int remoteRegRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size);
    static int remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size);
    /*
    * Functions: int sendData / remoteRegRead / remoteRegWrite / remoteRegDunction
    * -----------------------------------------------------------------------------------
    * Functions that allow read/write registers and function calls of a remote Pozyx shield using UWB-signal
    * Note: The remote shield must use the same UWB-settings in order to receive the requests
    * 
    * The send_data is used to write data to the RX-buffer of the destination shield.
    *
    * Input parameters:
    *     uint16_t destination: this is the networkid of the receiving Pozyx tag
    *     uint8_t data[]: data array tobe send
    *     uint8_t reg_address: the specific address for read, write or function call
    *     uint8_t *buffer: this is the pointer where the result of the call is stored
    *     uint8_t *pData: this is data that needs to be writen to the Pozyx register
    *     uint8_t *params: this is the pointer to where the parameters are stored
    *     int size: the size of passed variables in byte
    *
    * The return value is the same as for the reg_write_read function that implements the I2C communciation
    * For reg_function the return value return the status of the called function
    * see below for detailed description of each function
    *
    * Note: It's important to note that in most cases only one master tag can be active in a given
    * UWB-setting as multiple simultanious requests can collide and cause loss of data
    */


    /**
    * General functions accessing the Pozyx Shield
    *
    * All functions return an integer value, 
    * if the return value is 0 the function is executed succesful
    *
    * For all get functions a pointer is given as parameter to store the value
    *
    */
    
    /**
    *
    * STATUS REGISTERS
    *
    */

    static int getWhoAmI(uint8_t *whoami, uint16_t remote_id = NULL);
    static int getFirmwareVersion(uint8_t *firmware, uint16_t remote_id = NULL);
    static int getHardwareVersion(uint8_t *hardware, uint16_t remote_id = NULL);
    static int getSelftest(uint8_t *selftest, uint16_t remote_id = NULL);

    /*
    static int getStatus();
    static int getErrorCode();
    */

    static int getInterruptStatus(uint8_t *interrupts, uint16_t remote_id = NULL);
    static int getCalibrationStatus(uint8_t *calibration_status, uint16_t remote_id = NULL);

    /**
    *
    * CONFIGURATION REGISTERS
    *
    */

    static int getInterruptMask(uint8_t *mask, uint16_t remote_id = NULL);
    static int setInterruptMask(uint8_t mask, uint16_t remote_id = NULL);

    static int getUpdateInterval(uint16_t *ms, uint16_t remote_id = NULL);
    static int setUpdateInterval(uint16_t ms, uint16_t remote_id = NULL);

    static int getConfigGPIO(int gpio_num, uint8_t *config, uint16_t remote_id = NULL);
    static int convertGPIOMode(int config);
    static int convertGPIOPull(int config);
    static int setConfigGPIO(int gpio_num, int mode, int pull, uint16_t remote_id = NULL);

    /*
    static int getPositionAlg();
    static int setPositionAlg();
    */

    static int getNumberOfAnchors(uint8_t *nr_anchors, uint16_t remote_id = NULL);
    static int convertAnchorMode(int nr_anchors);
    static int convertAnchorNumber(int nr_anchors);
    static int setNumberOfAnchors(uint8_t nr_anchors, uint16_t remote_id = NULL);

    static int getNetworkId(uint16_t *network_id);
    static int setNetworkId(uint16_t network_id, uint16_t remote_id = NULL);

    static int getUWBSettings(UWB_settings_t *UWB_settings, uint16_t remote_id = NULL);
    static int setUWBSettings(UWB_settings_t UWB_settings, uint16_t remote_id = NULL);

    /*
    static int getRangeProtocol(uint8_t *range_protocol);
    static int setRangeProtocol(uint8_t range_protocol);
    */

    static int getSensorMode(uint8_t *sensor_mode, uint16_t remote_id = NULL);
    static int setSensorMode(uint8_t sensor_mode, uint16_t remote_id = NULL);

    
    /**
    *
    * POSITION DATA
    *
    */
    static int getCoordinates(coordinates_t *coordinates, uint16_t remote_id = NULL);
    static int setCoordinates(coordinates_t coordinates, uint16_t remote_id = NULL);

    static int getPositionError(pos_error_t *pos_error, uint16_t remote_id = NULL);


    /**
    *
    * SENSOR DATA
    *
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
    static int getDeviceListSize(uint8_t *device_list_size, uint16_t remote_id = NULL);
    static int getLastNetworkId(uint16_t *network_id, uint16_t remote_id = NULL);
    static int getLastDataLength(uint8_t *data_length, uint16_t remote_id = NULL);

    static int getGPIO(int gpio_num, uint8_t *value, uint16_t remote_id = NULL);
    static int setGPIO(int gpio_num, uint8_t value, uint16_t remote_id = NULL);

    static int setLedConfig(uint8_t config, uint16_t remote_id = NULL);

    /**
    *
    * GENERAL FUNCTIONS
    *
    */
    static void resetSystem(uint16_t remote_id = NULL);
    static int setLed(int led_num, boolean state, uint16_t remote_id = NULL);

    static int writeTXBufferData(uint8_t data[], int size);
    static int writeTXBufferData(uint8_t data[], int offset, int size);
    static int sendTXBufferData(uint16_t destination);
    static int readRXBufferData(uint8_t* pData, int size);

    static int doRanging(uint16_t destination);
    static int doRemoteRanging(uint16_t device_from, uint16_t device_to, device_range_t *device_range);
    static int doPositioning();
    static int doRemotePositioning(uint16_t remote_id, coordinates_t *coordinates);

    static int setAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id = NULL);
    static int getAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id = NULL);

    /**
    *
    * DEVICE LIST FUNCTIONS
    *
    */
    static int getDeviceIds(uint16_t devices[], int size = MAX_ANCHORS_IN_LIST, uint16_t remote_id = NULL);

    static int doDiscovery(int slots = 3, int slot_duration = 10);
    static int doAnchorCalibration(int option = POZYX_2D, int measurements = 10, uint16_t anchors[] = NULL, int anchor_num = 0);

    static int clearDevices(uint16_t remote_id = NULL);
    static int addDevice(device_coordinates_t device_coordinates, uint16_t remote_id = NULL);
    static int getDeviceInfo(uint16_t device_id, device_info_t *device_info, uint16_t remote_id = NULL);
    static int getDeviceCoordinates(uint16_t device_id, device_coordinates_t *device_coordinates, uint16_t remote_id = NULL);
    static int getDeviceRangeInfo(uint16_t device_id, device_range_t *device_range, uint16_t remote_id = NULL);

};

extern PozyxClass Pozyx;


#endif
