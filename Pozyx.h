/**
* Pozyx.h
* -------
* This file is a definition of all structures, classes and functions used in the
* Pozyx environment.
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

#undef NULL
#define NULL 0

///////////////////////////////////////////////// ASSERTIONS /////////////////////////////////////
// assertions will check for wrong use of the library.

// Define NDEBUG to remove the assertions: this will reduce codesize. Do this in your production code.
//#define NDEBUG

// enable printing of function name, filename, linenumber and failed expression. This takes a LOT of memory (too much for the Arduino UNO)
//#define __ASSERT_USE_STDERR
#include <assert.h>

// overwrite the assertions to reduce codesize.
#if ! defined(NDEBUG) && ! defined(__ASSERT_USE_STDERR)
    #undef assert
    #define assert(e)    ((e) ? (void)0 : \
                         __assert_pozyx(__func__, __FILE__, __LINE__))

    extern void __assert_pozyx(const char *__func, const char *__file, int __lineno);
#endif

////////////////////////////////////////// END OF ASSERTIONS /////////////////////////////////////

extern "C" {
  #include "Pozyx_definitions.h"
}

typedef float float32_t;


/**
* The UWB settings type defines all attributes needed to set the UWB (communication) parameters
*/
typedef struct _UWB_settings {
    /** The UWB channel number. Possible values are 1, 2, 3, 4, 5, 7. See the reg:POZYX_UWB_CHANNEL register for more information. */
    uint8_t channel;
    /** The bitrate. Possible values are
    *
    * - 0: 110kbits/s
    * - 1: 850kbits/s
    * - 2: 6.8Mbits/s.
    *
    * See the reg:POZYX_UWB_RATES register for more information */
    uint8_t bitrate;
    /** The UWB pulse repetition frequency (PRF). Possible values are
    *
    * - 1: 16MHz
    * - 2: 64MHz
    *
    * See the reg:POZYX_UWB_RATES register for more information */
    uint8_t prf;
    /** The preabmle length. Possible values are:
    *
    * - 0x0C : 4096 symbols.
    * - 0x28 : 2048 symbols.
    * - 0x18 : 1536 symbols.
    * - 0x08 : 1024 symbols.
    * - 0x34 : 512 symbols.
    * - 0x24 : 256 symbols.
    * - 0x14 : 128 symbols.
    * - 0x04 : 64 symbols.
    *
    * See the reg:POZYX_UWB_PLEN register for more information.
    */
    uint8_t plen;
    /** The transmission gain in dB. Possible values are between 0dB and 33.5dB, with a resolution of 0.5dB. See the reg:POZYX_UWB_GAIN register for more information.*/
    float gain_db;
}UWB_settings_t;

/**
* The coordinates type defines the coordinates of position result or anchor location
*/
typedef struct __attribute__((packed))_coordinates {
    /** The x-coordinate in mm */
    int32_t x;
    /** The y-coordinate in mm */
    int32_t y;
    /** The z-coordinate in mm */
    int32_t z;
}coordinates_t;

/**
 * A structure representing a 3D vector with floating points.
 * This type is used to represent most of the sensor values that have components in 3 dimensions.
 */
typedef struct __attribute__((packed))_v3D_float32 {
    /** The x-coordinate of the vector */
    float32_t x;
    /** The y-coordinate of the vector */
    float32_t y;
    /** The z-coordinate of the vector */
    float32_t z;
}v3D_float32_t;

// some supporting types for specific sensors
typedef v3D_float32_t acceleration_t;
typedef v3D_float32_t magnetic_t;
typedef v3D_float32_t angular_vel_t;
typedef v3D_float32_t linear_acceleration_t;
typedef v3D_float32_t gravity_vector_t;


/**
* The position error type gives the resulting error covariance for a given position result
*/
typedef struct __attribute__((packed))_pos_error {
    /** The variance in the x-coordinate */
    int16_t x;
    /** The variance in the y-coordinate */
    int16_t y;
    /** The variance in the z-coordinate */
    int16_t z;
    /** The covariance of xy */
    int16_t xy;
    /** The covariance of xz */
    int16_t xz;
    /** The covariance of yz */
    int16_t yz;
}pos_error_t;

/**
* The euler angles type holds the absolute orientation of the pozyx device using the Euler angles (yaw, pitch, roll) representation
*/
typedef struct __attribute__((packed))_euler_angles {
    /** The heading (yaw) in degrees. */
    float32_t heading;
    /** The roll in degrees. */
    float32_t roll;
    /** The pitch in degrees. */
    float32_t pitch;
}euler_angles_t;

/**
* The quaternion_t type holds the absolute orientation of the pozyx device using the a quaternion representation
*/
typedef struct __attribute__((packed))_quaternion {
    /** weight of the quaterion. */
    float32_t weight;
    /** x-coordinate of the quaterion. */
    float32_t x;
    /** y-coordinate of the quaterion. */
    float32_t y;
    /** z-coordinate of the quaterion. */
    float32_t z;
}quaternion_t;

/**
* raw sensor data. This follows the ordering of the pozyx registers
*/
typedef struct __attribute__((packed))_sensor_raw {
    uint32_t pressure;
    int16_t acceleration[3];
    int16_t magnetic[3];
    int16_t angular_vel[3];
    int16_t euler_angles[3];
    int16_t quaternion[4];
    int16_t linear_acceleration[3];
    int16_t gravity_vector[3];
    uint8_t temperature;
}sensor_raw_t;

/**
* The sensor data type allows to read the whole sensor data in one datastructure with one call
*/
typedef struct __attribute__((packed))_sensor_data {
    float32_t pressure;
    acceleration_t acceleration;
    magnetic_t magnetic;
    angular_vel_t angular_vel;
    euler_angles_t euler_angles;
    quaternion_t quaternion;
    linear_acceleration_t linear_acceleration;
    gravity_vector_t gravity_vector;
    float32_t temperature;
}sensor_data_t;

/**
* The device_coordinates_t type is used to describe a pozyx device required for the device list
*/
typedef struct __attribute__((packed))_device_coordinates {
    /** the unique 16-bit network id (by default this is the same as on the label of the device) */
    uint16_t network_id;
    /** a flag indicating some aspects of the device such as anchor or tag.
     * Possible values are:
     *
     * - 1 : anchor
     * - 2 : tag
     */
    uint8_t flag;
    /** The coordinates of the device */
    coordinates_t pos;
}device_coordinates_t;

/**
* The device range type stores all the attributes linked to a range measurement
*/
typedef struct __attribute__((packed))_device_range {
    /** The timestamp in ms of the range measurement. */
    uint32_t timestamp;
    /** The distance in mm. */
    uint32_t distance;
    /** The received signal strength in dBm. */
    int16_t RSS;
}device_range_t;

/**
* Provides an interface to an attached Pozyx shield.
* 
*/
class PozyxClass
{
protected:
    static int _mode;               // the mode of operation, can be MODE_INTERRUPT or MODE_POLLING
    static int _interrupt;          // variable to indicate that an interrupt has occured


    static int _hw_version;         // Pozyx harware version
    static int _fw_version;         // Pozyx software (firmware) version. (By updating the firmware on the Pozyx device, this value can change)


    /**
    * Function: i2cWriteWrite
    * -----------------------
    * Internal function that writes a number of bytes to a specfified Pozyx register
    * This function implements the I2C interface as described in the datasheet on our website
    * Input values:
    *   @param reg_address Pozyx address to write to
    *   @param pData reference to the data that needs to be writen
    *   @param size size in byte of data to be written
    *
    * @return
    *   #POZYX_FAILURE: error occured during the process
    *   #POZYX_SUCCESS: successful execution of the function
    */
    static int i2cWriteWrite(const uint8_t reg_address, const uint8_t *pData, int size);

    /**
    * Function: i2cWriteRead
    * ----------------------
    * Internal function that writes and reads a number of bytes to call a specfic Pozyx register function
    * This function implements the I2C interface as described in the datasheet on our website
    * Input values:
    *   @param write_data reference to the data that needs to be writen
    *   @param write_len size in byte of data to be written
    *   @param read_data reference to the pointer where the read data should be stored
    *   @param read_len size in byte of data to be read
    *
    * @return
    *   POZYX_FAILURE: error occured during the process
    *   POZYX_SUCCESS: successful execution of the function
    */
    static int i2cWriteRead(uint8_t* write_data, int write_len, uint8_t* read_data, int read_len);

    /**
    * Function: void IRQ
    * ------------------
    * Internal function that sets the _interrupt variable on an Arduino interrupt
    */
    static void IRQ();

    /**
    * This function calls the waitForFlag function in polling mode. After this, the previous mode is reset.
    *
    *   @param interrupt_flag the exepected Pozyx interrupt. Possible values are #POZYX_INT_STATUS_ERR,
    *   #POZYX_INT_STATUS_POS, #POZYX_INT_STATUS_IMU, #POZYX_INT_STATUS_RX_DATA, #POZYX_INT_STATUS_FUNC, or combinations.
    *   @param timeout_ms maximum waiting time in milliseconds for flag to occur
    *   @param interrupt a pointer that will contain the value of the interrupt status register
    *
    * @retval #true event occured.
    * @retval #false event did not occur, this function timed out.
    */

public:

    static boolean waitForFlag_safe(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt = NULL);

    /**
    * Read from the registers of the connected Pozyx shield.
    *
    *   @param reg_address: the specific register address to start reading from
    *   @param pData: a pointer to the data thas will be read
    *   @param size: the number of bytes to read
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int regRead(uint8_t reg_address, uint8_t *pData, int size);

    /**
    * Write to the registers of the connected Pozyx shield.
    *
    *   @param reg_address: the specific register address to start writing to
    *   @param pData: a pointer to the data thas will be written
    *   @param size: the number of bytes to write
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int regWrite(uint8_t reg_address, uint8_t *pData, int size);

    /**
    * Call a register funcion on the connected Pozyx shield.
    *
    *   @param reg_address: the specific register address of the function
    *   @param params: this is the pointer to a parameter array required for the specific function that is called
    *   @param param_size: the number of bytes in the params array
    *   @param pData: a pointer to the data thas will be read
    *   @param size: the number of bytes that will be stored in pData
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int regFunction(uint8_t reg_address, uint8_t *params=NULL, int param_size=0, uint8_t *pData=NULL, int size=0);

    /**
    * Write to the registers on a remote Pozyx device (anchor or tag).
    *
    *   @param destination: this is the network id of the receiving Pozyx tag
    *   @param reg_address: the specific register address to start writing to
    *   @param pData: a pointer to the data thas will be written
    *   @param size: the number of bytes to write
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int remoteRegWrite(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size);

    /**
    * Read from the registers on a remote Pozyx device (anchor or tag).
    *
    *   @param destination: this is the network id of the receiving Pozyx tag
    *   @param reg_address: the specific register address to start reading from
    *   @param pData: a pointer to the data thas will be read
    *   @param size: the number of bytes to read
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int remoteRegRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size);

    /**
    * Call a register funcion on a remote Pozyx device (anchor or tag).
    *
    *   @param destination: this is the network id of the receiving Pozyx tag
    *   @param reg_address: the specific register address of the function
    *   @param params: this is the pointer to a parameter array required for the specific function that is called
    *   @param param_size: the number of bytes in the params array
    *   @param pData: a pointer to the data thas will be read
    *   @param size: the number of bytes that will be stored in pData
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *params=NULL, int param_size=0, uint8_t *pData=NULL, int size=0);

    /** \addtogroup core
     *  @{
     */

    /**
    * Wait until the Pozyx shields has raised a specfic event flag or until timeout.
    * This functions halts the process flow until a specific event flag was raised by the Pozyx
    * shield. The event flag is checked by reading the contents of the reg:POZYX_INT_STATUS register.
    * This function can work in both polled and interupt mode
    *
    *   @param interrupt_flag the exepected Pozyx interrupt. Possible values are #POZYX_INT_STATUS_ERR,
    *   #POZYX_INT_STATUS_POS, #POZYX_INT_STATUS_IMU, #POZYX_INT_STATUS_RX_DATA, #POZYX_INT_STATUS_FUNC, or combinations.
    *   @param timeout_ms maximum waiting time in milliseconds for flag to occur
    *   @param interrupt a pointer that will contain the value of the interrupt status register
    *
    * @retval #true event occured.
    * @retval #false event did not occur, this function timed out.
    */

    static int remoteRegFunctionWithoutCheck(uint16_t destination, uint8_t reg_address, uint8_t *params=NULL, int param_size=0, uint8_t *pData=NULL, int size=0);

    /** \addtogroup core
     *  @{
     */

    /**
    * Does the same as the remoteRegFunction, but doesn't wait for nearly as long and doesn't care about whether the function worked.
    *
    *   @param interrupt_flag the exepected Pozyx interrupt. Possible values are #POZYX_INT_STATUS_ERR,
    *   #POZYX_INT_STATUS_POS, #POZYX_INT_STATUS_IMU, #POZYX_INT_STATUS_RX_DATA, #POZYX_INT_STATUS_FUNC, or combinations.
    *   @param timeout_ms maximum waiting time in milliseconds for flag to occur
    *   @param interrupt a pointer that will contain the value of the interrupt status register
    *
    * @retval #true event occured.
    * @retval #false event did not occur, this function timed out.
    */

    static boolean waitForFlag(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt = NULL);

    /**
    * Initiates the Pozyx shield. This function initializes the pozyx device.
    * It will verify that the device is functioning correctly by means of the self-test, and it will configure the interrupts.
    * See the register reg:POZYX_INT_MASK for more details about the interrupts.
    *
    * @param print_result outputs the result of the function to the Serial output
    * @param mode The modus of the system: #MODE_POLLING or #MODE_INTERRUPT
    * @param interrupts defines which events trigger interrupts. This field is only required for #MODE_INTERRUPT. Possible
    * values are bit-wise combinations of #POZYX_INT_MASK_ERR, #POZYX_INT_MASK_POS, #POZYX_INT_MASK_IMU, #POZYX_INT_MASK_RX_DATA and #POZYX_INT_MASK_FUNC. Use #POZYX_INT_MASK_ALL to trigger on all events.
    * @param interrupt_pin Pozyx interrupt pin: #POZYX_INT_PIN0 or #POZYX_INT_PIN1. This field is only required for #MODE_INTERRUPT.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int begin(boolean print_result = false, int mode = MODE_INTERRUPT,  int interrupts = POZYX_INT_MASK_ALL, int interrupt_pin = POZYX_INT_PIN0);


    /**
    * Read from the registers on a local or remote Pozyx device (anchor or tag).
    *
    *   @param reg_address: the specific register address to start reading from
    *   @param pData: a pointer to the data thas will be read
    *   @param size: the number of bytes to read
    *   @param remote_id: this is the network id of the remote Pozyx tag, if used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getRead(uint8_t reg_address, uint8_t *pData, int size, uint16_t remote_id=NULL);

    /**
    * Write to the registers of a local remote Pozyx device (anchor or tag).
    *
    *   @param reg_address: the specific register address to start writing to
    *   @param pData: a pointer to the data thas will be written
    *   @param size: the number of bytes to write
    *   @param remote_id: this is the network id of the remote Pozyx tag, if used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int setWrite(uint8_t reg_address, uint8_t *pData, int size, uint16_t remote_id=NULL);

    /**
    * Call a register funcion on a local or remote Pozyx device (anchor or tag).
    *
    *   @param reg_address: the specific register address of the function
    *   @param params: this is the pointer to a parameter array required for the specific function that is called
    *   @param param_size: the number of bytes in the params array
    *   @param pData: a pointer to the data thas will be read
    *   @param size: the number of bytes that will be stored in pData
    *   @param remote_id: this is the network id of the remote Pozyx tag, if used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int useFunction(uint8_t reg_address, uint8_t *params=NULL, int param_size=0, uint8_t *pData=NULL, int size=0, uint16_t remote_id=NULL);

/** @}*/

// currently groupless TODO


/** \addtogroup communication_functions
 *  @{
 */

    /**
    * Wirelessly transmit data to a remote pozyx device.
    * This function combines writeTXBufferData and sendTXBufferData to write data to the transmit buffer and immediately transmit it.
    *
    *   @param destination the network id of the device that should receive the data. A value of 0 will result in a broadcast
    *   @param pData pointer to the data that should be transmitted
    *   @param size number of bytes to transmit
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int sendData(uint16_t destination, uint8_t *pData, int size);

    /**
    * Write data bytes in the transmit buffer.
    * This function writes bytes in the transmit buffer without sending it yet.
    *
    *   @param data[] the array with data bytes
    *   @param size size of the data array
    *   @param offset The offset in the memory
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see sendTXBufferData
    */
    static int writeTXBufferData(uint8_t data[], int size, int offset = 0);

    /**
    * Wirelessly transmit data.
    * Wirelessly transmit the contents of the transmit buffer over UWB.
    *
    *   @param destination the network id of the device that should receive the data. A value of 0 will result in a broadcast.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see writeTXBufferData
    */
    static int sendTXBufferData(uint16_t destination = 0x0);


    /**
    * Read data bytes from receive buffer.
    * This function reads the bytes from the receive buffer from the last received message.
    *
    *   @param pData pointer to where the data will be stored
    *   @param size the number of bytes to read from the receive buffer.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see getLastDataLength getLastNetworkId
    */
    static int readRXBufferData(uint8_t* pData, int size);

    /**
    * Obtain the network id of the last message.
    * This function identifies the source of the last message that was wirelessly received.
    *
    *   @param network_id: the pointer that stores the network_id
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getLastNetworkId(uint16_t *network_id, uint16_t remote_id = NULL);

    /**
    * Obtain the number of bytes received.
    * This function gives the number of bytes in the last message that was wirelessly received.
    *
    *   @param data_length: the pointer that stores the number of received bytes
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getLastDataLength(uint8_t *data_length, uint16_t remote_id = NULL);

    /**
    * Obtain the network id of the connected Pozyx device.
    * The network id is a unique 16bit identifier determined based on the hardware components. When the system is reset the orignal value is restored
    *
    *   @param network_id: reference to the network_id pointer to store the data
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int getNetworkId(uint16_t *network_id);

    /**
    * Overwrite the network id.
    * This function overwrites the network id of the pozyx device either locally or remotely. The network id must be unique within a network.
    * When the system is reset the orignal network id is restored.
    *
    *   @param network_id: the new network id
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int setNetworkId(uint16_t network_id, uint16_t remote_id = NULL);

    /**
    * Obtain the current UWB settings.
    * Functions to retrieve the current UWB settings. In order to communicate, two pozyx devices must have exactly the same UWB settings.
    *
    *   @param UWB_settings reference to the UWB settings object to store the data
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getUWBSettings(UWB_settings_t *UWB_settings, uint16_t remote_id = NULL);

    /**
    * Overwrite the UWB settings.
    * This function overwrites the UWB settings such as UWB channel, gain, PRF, etc.
    * In order to communicate, two pozyx devices must have exactly the same UWB settings.
    * Upon reset, the default UWB settings will be loaded.
    *
    *   @param UWB_settings reference to the new UWB settings. If the gain_db is set to 0, it will automatically load the default gain value for the given uwb paramters.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int setUWBSettings(UWB_settings_t *UWB_settings, uint16_t remote_id = NULL);

    /**
    * Overwrite the UWB settings except the gain.
    * This function overwrites the UWB settings such as UWB channel, PRF, etc.
    * In order to communicate, two pozyx devices must have exactly the same UWB settings.
    * Upon reset, the default UWB settings will be loaded.
    *
    *   @param UWB_settings reference to the new UWB settings. If the gain_db is set to 0, it will automatically load the default gain value for the given uwb paramters.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int setUWBSettingsExceptGain(UWB_settings_t *UWB_settings, uint16_t remote_id=NULL);
    /**
    * Set the Ultra-wideband frequency channel.
    * This function sets the ultra-wideband (UWB) frequency channel used for transmission and reception.
    * For wireless communication, both the receiver and transmitter must be on the same UWB channel.
    * More information can be found in the register reg:POZYX_UWB_CHANNEL.
    *
    *   @param channel_num the channel number, possible values are 1, 2, 3, 4, 5 and 7.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int setUWBChannel(int channel_num, uint16_t remote_id = NULL);

    /**
    * Get the Ultra-wideband frequency channel.
    * This function reads the ultra-wideband (UWB) frequency channel used for transmission and reception.
    * For wireless communication, both the receiver and transmitter must be on the same UWB channel.
    * More information can be found in the register reg:POZYX_UWB_CHANNEL.
    *
    *   @param channel_num the channel number currently set, possible values are 1, 2, 3, 4, 5 and 7.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getUWBChannel(int* channel_num, uint16_t remote_id = NULL);

    /**
    * configure the UWB transmission power.
    *
    * @note setting a large TX gain may cause the system to fall out of regulation. Applications that require regulations must set an appropriate TX gain to meet the spectral mask of your region. This can be verified with a spectrum analyzer.
    *
    * This function configures the UWB transmission power gain. The default value is different for each UWB channel.
    * Setting a larger transmit power will result in a larger range. For increased communication range (which is two-way), both the
    * transmitter and the receiver must set the appropriate transmit power.
    * Changing the UWB channel will reset the power to the default value.
    *
    *   @param txgain_dB the transmission gain in dB. Possible values are between 0dB and 33.5dB, with a resolution of 0.5dB.
    *   @param remote_id optional parameter that determines the remote device to be used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int setTxPower(float txgain_dB, uint16_t remote_id = NULL);

    /**
    * Obtain the UWB transmission power.
    *
    * This function obtains the configured UWB transmission power gain. The default value is different for each UWB channel.
    * Changing the UWB channel will reset the TX power to the default value.
    *
    *   @param txgain_dB the configured transmission gain in dB. Possible values are between 0dB and 33.5dB, with a resolution of 0.5dB.
    *   @param remote_id optional parameter that determines the remote device to be used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getTxPower(float* txgain_dB, uint16_t remote_id = NULL);

/** @}*/


/** \addtogroup system_functions
 *  @{
 */

    /**
    * Obtain the who_am_i value.
    * This function reads the reg:POZYX_WHO_AM_I register.
    *
    *   @param whoami: reference to the pointer where the read data should be stored e.g. whoami
    *   @param remote_id: optional parameter that determines the remote device to be read
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getWhoAmI(uint8_t *whoami, uint16_t remote_id = NULL);

    /**
    * Obtain the firmware version.
    * This function reads the reg:POZYX_FIRMWARE_VER register.
    *
    *   @param firmware: reference to the pointer where the read data should be stored e.g. the firmware version
    *   @param remote_id: optional parameter that determines the remote device to be read
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getFirmwareVersion(uint8_t *firmware, uint16_t remote_id = NULL);

    /**
    * Obtain hte hardware version.
    * This function reads the reg:POZYX_HARDWARE_VER register.
    *
    *   @param data: reference to the pointer where the read data should be stored e.g. hardware version
    *   @param remote_id: optional parameter that determines the remote device to be read
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getHardwareVersion(uint8_t *hardware, uint16_t remote_id = NULL);

    /**
    * Obtain the selftest result.
    * This function reads the reg:POZYX_ST_RESULT register.
    *
    *   @param data: reference to the pointer where the read data should be stored e.g. the selftest result
    *   @param remote_id: optional parameter that determines the remote device to be read
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getSelftest(uint8_t *selftest, uint16_t remote_id = NULL);

    /**
    * Obtain the error code.
    * This function reads the reg:POZYX_ERRORCODE register.
    *
    *   @param data: reference to the pointer where the read data should be stored e.g. the error code
    *   @param remote_id: optional parameter that determines the remote device to be read
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getErrorCode(uint8_t *error_code, uint16_t remote_id = NULL);

    /**
    * Obtain the interrupt status.
    * This function reads the reg:POZYX_INT_STATUS register.
    *
    *   @param data: reference to the pointer where the read data should be stored e.g. the interrupt status
    *   @param remote_id: optional parameter that determines the remote device to be read
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see waitForFlag
    */
    static int getInterruptStatus(uint8_t *interrupts, uint16_t remote_id = NULL);

    /**
    * Obtain the calibration status.
    * This function reads the reg:POZYX_CALIB_STATUS register.
    *
    *   @param data: reference to the pointer where the read data should be stored e.g. calibration status
    *   @param remote_id: optional parameter that determines the remote device to be read
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getCalibrationStatus(uint8_t *calibration_status, uint16_t remote_id = NULL);

    /**
    * Obtain the digital value on one of the GPIO pins.
    * Function to retieve the value of the given General Purpose Input/output pin (GPIO) on the target device
    *
    *   @param gpio_num: the gpio pin to be set or retrieved. Possible values are 1, 2, 3 or 4.
    *   @param value: the pointer that stores the value for the GPIO.
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @note In firmware version v0.9. The GPIO state cannot be read remotely.
    */
    static int getGPIO(int gpio_num, uint8_t *value, uint16_t remote_id = NULL);

    /**
    * Set the digital value on one of the GPIO pins.
    * Function to set or set the value of the given GPIO on the target device
    *
    *   @param gpio_num: the gpio pin to be set or retrieved. Possible values are 1, 2, 3 or 4.
    *   @param value: the value for the GPIO. Can be O (LOW) or 1 (HIGH).
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int setGPIO(int gpio_num, uint8_t value, uint16_t remote_id = NULL);


    /**
    * Trigger a software reset of the Pozyx device.
    * Function that will trigger the reset of the system.
    * This will reload all configurations from flash memory, or to their default values.
    *
    *   @param remote_id: optional parameter that determines the remote device to be used.
    *
    * @see clearConfiguration, saveConfiguration
    */
    static void resetSystem(uint16_t remote_id = NULL);


    /**
    * Function for turning the leds on and off.
    * This function allows you to turn one of the 4 LEDs on the Pozyx board on or off.
    * By default, the LEDs are controlled by the Pozyx system to give status information.
    * This can be changed using the function setLedConfig.
    *
    *   @param led_num: the led number to be controlled, value between 1, 2, 3 or 4.
    *   @param state: the state to set the selected led to. Can be 0 (led is off) or 1 (led is on)
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see setLedConfig
    */
    static int setLed(int led_num, boolean state, uint16_t remote_id = NULL);

    /**
    * Function to obtain the interrupt configuration.
    * This functions obtains the interrupt mask from the register reg:POZYX_INT_MASK.
    * The interrupt mask is used to determine which event trigger an interrupt.
    *
    *   @param mask: the configured interrupt mask
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getInterruptMask(uint8_t *mask, uint16_t remote_id = NULL);

    /**
    * Function to configure the interrupts.
    * Function to configure the interrupts by means of the interrupt mask from the register reg:POZYX_INT_MASK.
    * The interrupt mask is used to determine which event trigger an interrupt.
    *
    *   @param mask: reference to the interrupt mask to be written. Bit-wise combinations of the following flags are allowed: #POZYX_INT_MASK_ERR,
    *   #POZYX_INT_MASK_POS, #POZYX_INT_MASK_IMU, #POZYX_INT_MASK_RX_DATA, #POZYX_INT_MASK_FUNC.
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int setInterruptMask(uint8_t mask, uint16_t remote_id = NULL);


    /**
    * Obtain the pull configuration of a GPIO pin.
    * Function to obtain the configured pin mode of the GPIO for the given pin number. This is performed by reading from
    * the reg:POZYX_CONFIG_GPIO1 up to reg:POZYX_CONFIG_GPIO4 register.
    *
    *   @param gpio_num: the pin to update
    *   @param mode: pointer to the mode of GPIO. Possible values #POZYX_GPIO_DIGITAL_INPUT, #POZYX_GPIO_PUSHPULL, #POZYX_GPIO_OPENDRAIN
    *   @param remote_id: optional parameter that determines the remote device to be used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getConfigModeGPIO(int gpio_num, uint8_t *mode, uint16_t remote_id = NULL);

    /**
    * Obtain the pull configuration of a GPIO pin.
    * Function to obtain the configured pull resistor of the GPIO for the given pin number. This is performed by reading from
    * the reg:POZYX_CONFIG_GPIO1 up to reg:POZYX_CONFIG_GPIO4 register.
    *
    *   @param gpio_num: the pin to update
    *   @param pull: pull configuration of GPIO. Possible values are #POZYX_GPIO_NOPULL, #POZYX_GPIO_PULLUP, #POZYX_GPIO_PULLDOWN.
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getConfigPullGPIO(int gpio_num, uint8_t *pull, uint16_t remote_id = NULL);


    /**
    * Configure the GPIOs.
    * Function to set the configuration mode of the GPIO for the given pin number. This is performed by writing to
    * the reg:POZYX_CONFIG_GPIO1 up to reg:POZYX_CONFIG_GPIO4 register.
    *
    *   @param gpio_num: the GPIO pin to update. Possible values are 1, 2, 3 or 4.
    *   @param mode: the mode of GPIO. Possible values #POZYX_GPIO_DIGITAL_INPUT, #POZYX_GPIO_PUSHPULL, #POZYX_GPIO_OPENDRAIN
    *   @param pull: pull configuration of GPIO. Possible values are #POZYX_GPIO_NOPULL, #POZYX_GPIO_PULLUP, #POZYX_GPIO_PULLDOWN.
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int setConfigGPIO(int gpio_num, int mode, int pull, uint16_t remote_id = NULL);

    /**
    * Configure the LEDs.
    * This function configures the 6 LEDs on the pozyx device by writing to the register reg:POZYX_LED_CTRL.
    * More specifically, the function configures which LEDs give system information. By default all the LEDs
    * will give system information.
    *
    *   @param config default: the configuration to be set. See POZYX_LED_CTRL for details.
    *   @param remote_id: optional parameter that determines the remote device to be used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see setLed
    */
    static int setLedConfig(uint8_t config = 0x0, uint16_t remote_id = NULL);

        /**
     * Configure the interrupt pin.
     *
     * @param  pin          pin id on the pozyx device, can be 1,2,3,4 (or 5 or 6 on the pozyx tag)
     * @param  mode         push-pull or pull-mode
     * @param  bActiveHigh  is the interrupt active level HIGH (i.e. 3.3V)
     * @param  bLatch       should the interrupt be a short pulse or should it latch until the interrupt status register is read
     *
     * @retval #POZYX_SUCCESS success.
     * @retval #POZYX_FAIL function failed.
     */
    static int configInterruptPin(int pin, int mode, int bActiveHigh, int bLatch, uint16_t remote_id=NULL);

    /**
    * Save (part of) the configuration to Flash memory.
    * @version Requires firmware version v1.0
    *
    * This functions stores (parts of) the configurable Pozyx settings in the non-volatile flash memory of the Pozyx device.
    * This function will save the current settings and on the next startup of the Pozyx device these saved settings will be loaded automatically.
    * settings from the flash memory. All registers that are writable, the anchor ids for positioning and the device list (which contains the anchor coordinates) can be saved.
    *
    *   @param type this specifies what should be saved. Possible options are #POZYX_FLASH_REGS, #POZYX_FLASH_ANCHOR_IDS or #POZYX_FLASH_NETWORK.
    *   @param registers an option array that holds all the register addresses for which the value must be saved. All registers that are writable are allowed.
    *   @param num_registers optional parameter that determines the length of the registers array.
    *   @param remote_id optional parameter that determines the remote device to be used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAIL function failed.
    * @retval #POZYX_TIMEOUT function timed out.
    *
    * @see clearConfiguration
    */
    static int saveConfiguration(int type, uint8_t registers[] = NULL, int num_registers = 0, uint16_t remote_id = NULL);

    /**
    * Save registers to the flash memory. See saveConfiguration for more information
    * @version Requires firmware version v1.0
    *
    *   @param registers an option array that holds all the register addresses for which the value must be saved. All registers that are writable are allowed.
    *   @param num_registers optional parameter that determines the length of the registers array.
    *   @param remote_id optional parameter that determines the remote device to be used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAIL function failed.
    * @retval #POZYX_TIMEOUT function timed out.
    * @see clearConfiguration
    */
    static int saveRegisters(uint8_t registers[] = NULL, int num_registers = 0, uint16_t remote_id = NULL);

    /**
    * Save the Pozyx's stored network to its flash memory. See saveConfiguration for more information
    * @version Requires firmware version v1.0
    *
    *   @param remote_id optional parameter that determines the remote device to be used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAIL function failed.
    * @retval #POZYX_TIMEOUT function timed out.
    * @see clearConfiguration
    */
    static int saveNetwork(uint16_t remote_id = NULL);

    /**
    * Save the Pozyx's stored anchor list to its flash memory. See saveConfiguration for more information
    * @version Requires firmware version v1.0
    *
    *   @param remote_id optional parameter that determines the remote device to be used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAIL function failed.
    * @retval #POZYX_TIMEOUT function timed out.
    * @see clearConfiguration
    */
    static int saveAnchorIds(uint16_t remote_id = NULL);

    /**
    * Save the Pozyx's stored UWB settings to its flash memory. See saveConfiguration for more information
    * @version Requires firmware version v1.0
    *
    *   @param remote_id optional parameter that determines the remote device to be used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAIL function failed.
    * @retval #POZYX_TIMEOUT function timed out.
    * @see clearConfiguration
    */
    static int saveUWBSettings(uint16_t remote_id = NULL);

    // static int saveAnchorSelection(int type, uint8_t registers[] = NULL, int num_registers = 0, uint16_t remote_id = NULL);

    /**
    * Clears the configuration.
    * @version Requires firmware version v1.0
    *
    * This function clears (part of) the configuration that was previously stored in the non-volatile Flash memory.
    * The next startup of the Pozyx device will load the default configuration values for the registers, anchor_ids and network list.
    *
    *   @param remote_id optional parameter that determines the remote device to be used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAIL function failed.
    *
    * @see saveConfiguration
    */
    static int clearConfiguration(uint16_t remote_id = NULL);

    /**
     * Verify if a register content is saved in the flash memory.
     * @version Requires firmware version v1.0
     *
     * This function verifies if a given register variable, specified by its address, is saved in flash memory.
     * @param  regAddress the register address to check
     * @param  remote_id optional parameter that determines the remote device to be used.
     *
     * @retval true(1) if the register variable is saved
     * @retval false(0) if the register variable is not saved
     */
    static bool isRegisterSaved(uint8_t regAddress, uint16_t remote_id = NULL);

    /**
     * Return the number of register variables saved in flash memory.
     *
     * @param  remote_id optional parameter that determines the remote device to be used.
     *
     * @return           the number of register variables saved in flash memory.
     */
    static int getNumRegistersSaved(uint16_t remote_id = NULL);

/** @}*/


/** \addtogroup positioning_functions
 *  @{
 */


    /**
    * Obtain the last coordinates of the device.
    * Retrieve the last coordinates of the device or the remote device. Note that this function does not
    * trigger positioning.
    *
    *   @param coordinates reference to the coordinates pointer object.
    *   @param remote_id optional parameter that determines the remote device to be used.
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see doPositioning, doRemotePositioning
    */
    static int getCoordinates(coordinates_t *coordinates, uint16_t remote_id = NULL);

    /**
    * Set the coordinates of the device.
    * Manually set the coordinates of the device or the remote device.
    *
    *   @param coordinates coordinates to be set
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see getCoordinates
    */
    static int setCoordinates(coordinates_t coordinates, uint16_t remote_id = NULL);

    /**
    * Obtain the last estimated position error covariance information.
    * Retrieve the last error covariance of the position for the device or the remote device. Note that this function does not
    * trigger positioning.
    *
    *   @param pos_error reference to the pos error object
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getPositionError(pos_error_t *pos_error, uint16_t remote_id = NULL);

    /**
    * Manually set which anchors to use for positioning.
    * Function to manually set which anchors to use for positioning by calling the register function reg:POZYX_POS_SET_ANCHOR_IDS.
    * Notice that the anchors specified are only used if this is configured with setSelectionOfAnchors. Furthermore, the anchors
    * specified in this functions must be available in the device list with their coordinates.
    *
    *   @param anchors[] an array with network id's of anchors to be used
    *   @param anchor_num the number of anchors write
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see setSelectionOfAnchors, getPositioningAnchorIds
    */
    static int setPositioningAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id = NULL);

    /**
    * Obtain which anchors used for positioning.
    * Function to retrieve the anchors that used for positioning by calling the register function reg:POZYX_POS_GET_ANCHOR_IDS.
    * When in automatic anchor selection mode, the chosen anchors are listed here.
    *
    *   @param anchors[] an array with network id's of anchors manually set
    *   @param anchor_num the number of anchors to read.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see setSelectionOfAnchors, setPositioningAnchorIds
    */
    static int getPositioningAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id = NULL);

    /**
    * Read the update interval continuous positioning.
    * This function reads the configured update interval for continuous positioning from the register reg:POZYX_POS_INTERVAL.
    *
    *   @param ms pointer to the update interval in milliseconds. A value of 0 means that continuous positioning is disabled.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see setUpdateInterval
    */
    static int getUpdateInterval(uint16_t *ms, uint16_t remote_id = NULL);

    /**
    * Configure the udpate interval for continuous positioning.
    * This function configures the update interval by writing to the register reg:POZYX_POS_INTERVAL.
    * By writing a valid value, the system will start continuous positioning which will generate a position estimate
    * on regular intervals. This will generate a #POZYX_INT_STATUS_POS interrupt when interrupts are enabled.
    *
    *   @param ms update interval in milliseconds. The update interval must be larger or equal to 100ms.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see getUpdateInterval
    */
    static int setUpdateInterval(uint16_t ms, uint16_t remote_id = NULL);

    /**
    * Obtain the configured ranging protocol.
    * This function obtains the configured ranging protocol by reading from the reg:POZYX_RANGE_PROTOCOL register.
    *
    *   @param protocol pointer to the variable holding the ranging protocol used when ranging.
    *   Possible values for the ranging protocol are POZYX_RANGE_PROTOCOL_FAST and POZYX_RANGE_PROTOCOL_PRECISION.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see doRanging, setRangingProtocol
    */
    static int getRangingProtocol(uint8_t *protocol, uint16_t remote_id = NULL);

    /**
    * Configure the ranging protocol.
    * This function configures the ranging protocol by writing to the reg:POZYX_RANGE_PROTOCOL register.
    *
    *   @param protocol Ranging protocol used when ranging.
    *   Possible values for the ranging protocol are POZYX_RANGE_PROTOCOL_FAST and POZYX_RANGE_PROTOCOL_PRECISION.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see doRanging, getRangingProtocol
    */
    static int setRangingProtocol(uint8_t protocol, uint16_t remote_id = NULL);

    /** TODO
    * Obtain the configured positioning filter strength.
    * This function obtains the configured positioning filter strength by reading from the reg:POZYX_POS_FILTER register.
    *
    *   @param filter_strength pointer to the variable holding the filter strength used in the built-in filter.
    *   This strength is the amount of previous samples used in positioning.
    *   Possible values for the position filter strength is between 0 and 15 samples.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see getPositionFilterType, getPositionFilter, setPositionFilter, setPositionFilterType, setPositionFilterStrength
    */
    static int getPositionFilterStrength(uint8_t *filter_strength, uint16_t remote_id = NULL);

    /** TODO
    * Obtain the configured positioning algorithm.
    * This function obtains the configured positioning algorithm by reading from the reg:POZYX_POS_ALG register.
    *
    *   @param algorithm pointer to the variable holding the algorithm used to determine position.
    *   Possible values for the positioning algorithm are #POZYX_POS_ALG_UWB_ONLY and #POZYX_POS_ALG_LS.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see getPositionDimension, setPositionAlgorithm
    */
    static int getPositionFilterType(uint8_t *filter_type, uint16_t remote_id = NULL);

    /** TODO
    * Configure the ranging protocol.
    * This function configures the ranging protocol by writing to the reg:POZYX_RANGE_PROTOCOL register.
    *
    *   @param protocol Ranging protocol used when ranging.
    *   Possible values for the ranging protocol are POZYX_RANGE_PROTOCOL_FAST and POZYX_RANGE_PROTOCOL_PRECISION.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see doRanging, getRangingProtocol
    */
    static int setPositionFilter(uint8_t filter_type, uint8_t filter_strength, uint16_t remote_id = NULL);

    /**
    * Obtain the configured positioning algorithm.
    * This function obtains the configured positioning algorithm by reading from the reg:POZYX_POS_ALG register.
    *
    *   @param algorithm pointer to the variable holding the algorithm used to determine position.
    *   Possible values for the positioning algorithm are #POZYX_POS_ALG_UWB_ONLY and #POZYX_POS_ALG_LS.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see getPositionDimension, setPositionAlgorithm
    */
    static int getPositionAlgorithm(uint8_t *algorithm, uint16_t remote_id = NULL);

    /**
    * Obtain the configured positioning dimension.
    * This function obtains the configuration of the physical dimension by reading from the reg:POZYX_POS_ALG register.
    *
    *   @param dimension pointer to physical dimension used for the algorithm. Possible values for the dimension are #POZYX_3D, #POZYX_2D or #POZYX_2_5D.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see getPositionAlgorithm, setPositionAlgorithm
    */
    static int getPositionDimension(uint8_t *dimension, uint16_t remote_id = NULL);


    /**
    * Configure the positioning algorithm.
    * This function configures the positioning algorithm and the desired physical dimension by writing to the
    * register reg:POZYX_POS_ALG.
    *
    *   @param algorithm algorithm used to determine the position. Possible values are #POZYX_POS_ALG_UWB_ONLY and #POZYX_POS_ALG_LS.
    *   @param dimension physical dimension used for the algorithm. Possible values are #POZYX_3D, #POZYX_2D or #POZYX_2_5D.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see getPositionAlgorithm, getPositionDimension
    */
    static int setPositionAlgorithm(int algorithm = POZYX_POS_ALG_UWB_ONLY, int dimension = 0x0, uint16_t remote_id = NULL);

    /**
    * Obtain the anchor selection mode.
    * This function reads the anchor selection mode bit from the register reg:POZYX_POS_NUM_ANCHORS.
    * This bit describes how the anchors are selected for positioning, either manually or automatically.
    *
    *   @param mode reference to the anchor selection mode. Possible results are #POZYX_ANCHOR_SEL_MANUAL for manual anchor selection or #POZYX_ANCHOR_SEL_AUTO for automatic anchor selection.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getAnchorSelectionMode(uint8_t *mode, uint16_t remote_id = NULL);


    /**
    * Obtain the configured number of anchors used for positioning.
    * This functions reads out the reg:POZYX_POS_NUM_ANCHORS register to obtain the
    * number of anchors that are being used for positioning.
    *
    *   @param nr_anchors reference to the number of anchors
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getNumberOfAnchors(uint8_t *nr_anchors, uint16_t remote_id = NULL);


    /**
    * Configure how many anchors are used for positioning and how they are selected.
    * This function configures the number of anchors used for positioning. Theoretically, a larger
    * number of anchors leads to better positioning performance. However, in practice this is not always the case.
    * The more anchors used for positioning, the longer the positioning process will take. Furthermore,
    * it can be chosen which anchors are to be used: either a fixed set given by the user, or an automatic selection
    * between all the anchors in the internal device list.
    *
    *   @param mode describes how the anchors are selected. Possible values are #POZYX_ANCHOR_SEL_MANUAL for manual anchor selection or #POZYX_ANCHOR_SEL_AUTO for automatic anchor selection.
    *   @param nr_anchors the number of anchors to use for positioning. Must be larger than 2 and smaller than 16.
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see setPositioningAnchorIds to set the anchor IDs in manual anchor selection mode.
    */
    static int setSelectionOfAnchors(int mode, int nr_anchors, uint16_t remote_id = NULL);

    /**
    * Obtain the operation mode of the device.
    * This function obtains the operation mode (anchor or tag) by reading from the register reg:POZYX_OPERATION_MODE.
    * This operation mode is independent of the hardware it is on and will have it's effect when performing discovery
    * or auto calibration.
    *
    *   @param mode The mode of operations #POZYX_ANCHOR_MODE or #POZYX_TAG_MODE
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    *
    * @see setOperationMode
    */
    static int getOperationMode(uint8_t *mode, uint16_t remote_id = NULL);


    /**
    * Define if the device operates as a tag or an anchor.
    * This function defines how the device should operate (as an anchor or tag) by writing to the register reg:POZYX_OPERATION_MODE.
    * This operation mode is independent of the hardware it is on and will have it's effect when performing discovery
    * or auto calibration. This function overrules the jumper that is present on the board.
    *
    *   @param mode The mode of operations #POZYX_ANCHOR_MODE or #POZYX_TAG_MODE
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see getOperationMode
    */
    static int setOperationMode(uint8_t mode, uint16_t remote_id = NULL);


    /**
    * Get the textual system error.
    * This function reads out the reg:POZYX_ERRORCODE register and converts the error code to a textual message.
    *
    *   @param remote_id optional parameter that determines the remote device to be used
    *
    * @retval String the textual error
    *
    */
    static String getSystemError(uint16_t remote_id = NULL);


/** @}*/


/** \addtogroup sensor_data
 *  @{
 */

    /**
    * Retrieve the configured sensor mode.
    * This function reads out the register reg:POZYX_SENSORS_MODE which describes the configured sensor mode.
    *
    *   @param sensor_mode: reference to the sensor mode
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getSensorMode(uint8_t *sensor_mode, uint16_t remote_id = NULL);

    /**
    * Configure the sensor mode.
    * This function reads out the register reg:POZYX_SENSORS_MODE which describes the configured sensor mode.
    *
    *   @param sensor_mode: the desired sensor mode.
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int setSensorMode(uint8_t sensor_mode, uint16_t remote_id = NULL);

    /**
    * Obtain all raw sensor data at once as it's stored in the registers.
    * This functions reads out the pressure, acceleration, magnetic field strength, angular velocity, orientation in Euler angles, the orientation as a quaternion,
    * the linear acceleration, the gravity vector and temperature.
    *
    *   @param sensor_raw: reference to the sensor_raw object
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getRawSensorData(sensor_raw_t *sensor_raw, uint16_t remote_id = NULL);

    /**
    * Obtain all sensor data at once.
    * This functions reads out the pressure, acceleration, magnetic field strength, angular velocity, orientation in Euler angles, the orientation as a quaternion,
    * the linear acceleration, the gravity vector and temperature.
    *
    *   @param sensor_data: reference to the sensor_data object
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getAllSensorData(sensor_data_t *sensor_data, uint16_t remote_id = NULL);

    /**
    * Obtain the atmospheric pressure in Pascal.
    * This function reads out the pressure starting from the register POZYX_PRESSURE. The maximal update rate is 10Hz. The units are Pa.
    *
    *   @param pressure: reference to the pressure variable
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getPressure_Pa(float32_t *pressure, uint16_t remote_id = NULL);

    /**
    * Obtain the max linear acceleration
    * This registers functions similarly to the interrupt and error registers in that it clears
    * the register's value upon reading. This is the max linear acceleration since the last read.
    *
    *   @param max_lin_acc: pointer to a variable that will hold the max linear acceleration
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getMaxLinearAcceleration(uint16_t *max_lin_acc, uint16_t remote_id = NULL);

    /**
    * Obtain the 3D acceleration vector in mg.
    * This function reads out the acceleration data starting from the register reg:POZYX_ACCEL_X. The maximal update rate is 100Hz. The units are mg.
    * The vector is expressed in body coordinates (i.e., along axes of the device).
    *
    *   @param acceleration: reference to the acceleration object
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getAcceleration_mg(acceleration_t *acceleration, uint16_t remote_id = NULL);

    /**
    * Obtain the 3D magnetic field strength vector in µTesla.
    * This function reads out the magnetic field strength data starting from the register reg:POZYX_MAGN_X. The maximal update rate is 100Hz.
    * The vector is expressed in body coordinates (i.e., along axes of the device).
    *
    *   @param magnetic: reference to the magnetic object
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getMagnetic_uT(magnetic_t *magnetic, uint16_t remote_id = NULL);

    /**
    * Obtain the 3D angular velocity vector degrees per second.
    * This function reads out the angular velocity from the gyroscope using the register reg:POZYX_GYRO_X. The maximal update rate is 100Hz.
    * The rotations are expressed in body coordinates (i.e., the rotations around the axes of the device).
    *
    *   @param angular_vel: reference to the angular velocity object
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getAngularVelocity_dps(angular_vel_t *angular_vel, uint16_t remote_id = NULL);

    /**
    * Obtain the orientation in Euler angles in degrees.
    * This function reads out the Euleur angles: Yaw, Pitch and Roll that represents the 3D orientation of the device
    *
    *   @param euler_angles: reference to the euler_angles object
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getEulerAngles_deg(euler_angles_t *euler_angles, uint16_t remote_id = NULL);

    /**
    * Obtain the orientation in quaternions.
    * This function reads out the 4 coordinates of the quaternion that represents the 3D orientation of the device.
    * The quaternions are unitless and normalized.
    *
    *   @param quaternion: reference to the quaternion object
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getQuaternion(quaternion_t *quaternion, uint16_t remote_id = NULL);

    /**
    * Obtain the 3D linear acceleration in mg.
    * This function reads out the linear acceleration data starting from the register reg:POZYX_LIA_X.
    * The Linear acceleration is the acceleration compensated for gravity.
    * The maximal update rate is 100Hz. The units are mg.
    * The vector is expressed in body coordinates (i.e., along axes of the device).
    *
    *   @param linear_acceleration: reference to the acceleration object
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getLinearAcceleration_mg(linear_acceleration_t *linear_acceleration, uint16_t remote_id = NULL);

    /**
    * Obtain the 3D gravity vector in mg.
    * This function reads out the gravity vector coordinates starting from the register reg:POZYX_GRAV_X. The maximal update rate is 100Hz. The units are mg.
    * The vector is expressed in body coordinates (i.e., along axes of the device). This vector always points to the ground, regardless of the orientation.
    *
    *   @param gravity_vector: reference to the gravity_vector object
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getGravityVector_mg(gravity_vector_t *gravity_vector, uint16_t remote_id = NULL);

    /**
    * Obtain the temperature in degrees Celcius.
    * This function reads out the temperature from the register reg:POZYX_TEMPERATURE.
    * This function is unsupported in firmware version v0.9.
    *
    *   @param temperature: reference to the temperature variable
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    * @retval #POZYX_TIMEOUT function timed out, no response received.
    */
    static int getTemperature_c(float32_t *temperature, uint16_t remote_id = NULL);

/** @}*/

/** \addtogroup positioning_functions
 *  @{
 */
    
    [[deprecated("doPositioning will no longer set the algorithm in future releases. Use setPositionAlgorithm in setup() instead.")]]
    static int doPositioning(coordinates_t *coordinates, uint8_t dimension, int32_t height, uint8_t algorithm);
    
    /**
    * Obtain the coordinates.
    * This function triggers the positioning algorithm to perform positioning with the given parameters.
    * By default it will automatically select 4 anchors from the internal device list. It will then perform
    * ranging with these anchors and use the results to compute the coordinates.
    * This function requires that the coordinates for the anchors are stored in the internal device list.
    *
    * Please read the tutorial ready to localize to get started with positioning.
    *
    *   @param position: data object to store the result
    *   @param height: optional parameter that is used for #POZYX_2_5D to give the height in mm of the tag
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see doRemotePositioning, doAnchorCalibration, addDevice, setSelectionOfAnchors, setPositionAlgorithm
    */
    static int doPositioning(coordinates_t *position, uint8_t dimension, int32_t height=0);

    [[deprecated("doRemotePositioning will no longer set the algorithm in future releases. Use setPositionAlgorithm in setup() instead.")]]
    static int doRemotePositioning(uint16_t remote_id, coordinates_t *coordinates, uint8_t dimension, int32_t height, uint8_t algorithm);

    /**
    * Obtain the coordinates of a remote device. Don't use with 2.5D!
    *
    * This function triggers the positioning algorithm on a remote pozyx device to perform positioning with the given parameters.
    * By default it will automatically select 4 anchors from the internal device list on the remote device. The device will perform
    * ranging with the anchors and use the results to compute the coordinates.
    * This function requires that the coordinates for the anchors are stored in the internal device list on the remote device.
    * After positioning is completed, the remote device will automatically transmit the result back.
    *
    *   @param remote_id: the remote device that will do the positioning
    *   @param position: data object to store the result
    *   @param height: optional parameter that is used for #POZYX_2_5D to give the height in mm of the tag
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see doPositioning, addDevice, setSelectionOfAnchors, setPositionAlgorithm
    */
    static int doRemotePositioning(uint16_t remote_id, coordinates_t *coordinates, uint8_t dimension, int32_t height=0);

    /**
    * Trigger ranging with a remote device.
    * This function performs ranging with a remote device using the UWB signals.
    *
    *   @param destination: the target device to do ranging with
    *   @param range: the pointer to where the resulting data will be stored
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see doRemoteRanging, getDeviceRangeInfo
    */
    static int doRanging(uint16_t destination, device_range_t *range);

    /**
    * Trigger ranging between two remote devivces.
    * Function allowing to trigger ranging between two remote devices A and B. The ranging data is collected by
    * device A and transmitted back to the local device.
    *
    *   @param device_from: device A that will initiate the range request.
    *   @param device_to: device B that will respond to the range request.
    *   @param range: the pointer to where the resulting data will be stored
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see doRanging, getDeviceRangeInfo
    */
    static int doRemoteRanging(uint16_t device_from, uint16_t device_to, device_range_t *range);

    /**
    * Retrieve stored ranging information.
    * Functions to retrieve the latest ranging information (i.e., the distance, signal strength and timestamp) with
    * respect to a remote device. This function does not trigger ranging.
    *
    *   @param device_id: network id of the device for which range information is requested
    *   @param device_range: data object to store the information
    *   @param remote_id: optional parameter that determines the remote device where this function is called.
    *
    * @see doRanging, doRemoteRanging
    */
    static int getDeviceRangeInfo(uint16_t device_id, device_range_t *device_range, uint16_t remote_id = NULL);

/** @}*/

/** \addtogroup device_list
 *  @{
 */

    /**
    * Obtain the number of devices stored internally.
    * The following function retrieves the number of devices stored in the device list.
    *
    *   @param device_list_size: the pointer that stores the device list size
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see doDiscovery, doAnchorCalibration
    */
    static int getDeviceListSize(uint8_t *device_list_size, uint16_t remote_id = NULL);


    /**
    * Obtain the network IDs from all the devices in the device list.
    * Function to get all the network ids of the devices in the device list
    *
    *   @param devices[]: array that will be filled with the network ids
    *   @param size the number of network IDs to read.
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int getDeviceIds(uint16_t devices[], int size, uint16_t remote_id = NULL);

    /**
    * Obtain the network IDs from all the anchors in the device list.
    * Function to get all the network ids of the anchors in the device list
    *
    *   @param devices[]: array that will be filled with the network ids
    *   @param size the number of network IDs to read.
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int getAnchorIds(uint16_t anchors[], int size, uint16_t remote_id = NULL);

    /**
    * Obtain the network IDs from all the tags in the device list.
    * Function to get all the network ids of the tags in the device list
    *
    *   @param tags[]: array that will be filled with the network ids
    *   @param size the number of network IDs to read.
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int getTagIds(uint16_t tags[], int size, uint16_t remote_id = NULL);

    /**
    * Discover Pozyx devices in range.
    * Function to wirelessly discover anchors/tags/all Pozyx devices in range. The discovered devices are added
    * to the internal device list.
    *
    *   @param type which type of device to discover. Possible values are #POZYX_DISCOVERY_ANCHORS_ONLY: anchors only, #POZYX_DISCOVERY_TAGS_ONLY: tags only or #POZYX_DISCOVERY_ALL_DEVICES: anchors and tags
    *   @param slots The number of slots to wait for a response of an undiscovered device
    *   @param slot_duration Time duration of an idle slot
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    *
    * @see getDeviceListSize, getDeviceIds
    */
    static int doDiscovery(int type = 0x0, int slots = 3, int slot_duration = 10);

    /**
    * Automatically obtain the relative anchor positions.
    * WARNING: This is currently experimental and will be improved in the next firmware version!
    * This function triggers the automatic anchor calibration to obtain the relative coordinates of up to 6
    * pozyx devices in range. This function can be used for quickly setting up the positioning system.
    * The procedure may take several hundres of milliseconds depending on the number of devices in range and
    * the number of range measurements requested. During the calibration proces LED 2 will be turned on.
    * At the end of calibration the corresponding bit in the reg:POZYX_CALIB_STATUS register will be set.
    * The resulting coordinates are stored in the internal device list.
    * \n\n
    * Please read the tutorial Ready to Localize to learn how to use this function.
    *
    *   @param type dimension of the calibration, can be #POZYX_2D or #POZYX_2_5D
    *   @param measurements: The number of measurements per link. Recommended 10. Theoretically, a larger number should result in better calibration accuracy.
    *   @param anchor_num The number of anchors in the anchors[] array
    *   @param anchors[] The anchors that determine the axis (see datasheet)
    *   @param heights The heights in mm of the anchors in the anchors[] array (only used for #POZYX_2_5D)
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int doAnchorCalibration(int dimension = POZYX_2D, int num_measurements = 10, int num_anchors = 0, uint16_t anchors[] = NULL,  int32_t heights[] = NULL);

    /**
    * Empty the internal list of devices.
    * This function empties the internal list of devices.
    *
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int clearDevices(uint16_t remote_id = NULL);

    /**
    * Manualy adds a device to the device list.
    * This function can be used to manually add a device and its coordinates to the device list.
    * Once the device is added, it can be used for positioning.
    *
    *   @param device_coordinates: the device information to be added
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int addDevice(device_coordinates_t device_coordinates, uint16_t remote_id = NULL);

    /**
    * Retrieve the stored coordinates of a device.
    * This function retrieves the device coordinates stored in the internal device list.
    *
    *   @param device_id: device from which the information needs to be retrieved
    *   @param device_coordinates: data object to store the information
    *   @param remote_id: optional parameter that determines the remote device to be used
    *
    * @retval #POZYX_SUCCESS success.
    * @retval #POZYX_FAILURE function failed.
    */
    static int getDeviceCoordinates(uint16_t device_id, coordinates_t *coordinates, uint16_t remote_id = NULL);

/** @}*/

};
extern PozyxClass Pozyx;


#endif
