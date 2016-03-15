/**
  Pozyx_definitions.h - Library for Arduino Pozyx shield.
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

#ifndef POZYX_DEFINITIONS_h
#define POZYX_DEFINITIONS_h

#define _POZYX_LIB_VERSION          1
#define _POZYX_FEET_PER_METER       3.2808399
#define _POZYX_INCH_PER_METER       39.3700787

#define _POZYX_FORMAT_METER
#define _POZYX_FORMAT_FOOT
#define _POZYX_FORMAT_INCH

// maximum number of anchors to be stored in the interal anchor list on the pozyx device
#define MAX_ANCHORS_IN_LIST          20

#define POZYX_I2C_ADDRESS        0x4B

/* Begin auto generated defines */ 

/* POZYX REGISTERS */ 

/* Status registers */ 
#define POZYX_WHO_AM_I          0x0   /* Returns the constant value 0x43. */
#define POZYX_FIRMWARE_VER        0x1   /* Returns the POZYX firmware version. */
#define POZYX_HARDWARE_VER        0x2   /* Returns the POZYX hardware version. */
#define POZYX_ST_RESULT         0x3   /* Returns the self-test result */
#define POZYX_ERRORCODE         0x4   /* Describes a possibly system error. */
#define POZYX_INT_STATUS        0x5   /* Indicates the source of the interrupt. */
#define POZYX_CALIB_STATUS        0x6   /* Returns the calibration status. */

/* Configuration registers */ 
#define POZYX_INT_MASK          0x10    /* Indicates which interrupts are enabled. */
#define POZYX_CONFIG_GPIO1        0x11    /* Configure GPIO pin 1. */
#define POZYX_CONFIG_GPIO2        0x12    /* Configure GPIO pin 2. */
#define POZYX_CONFIG_GPIO3        0x13    /* Configure GPIO pin 3. */
#define POZYX_CONFIG_GPIO4        0x14    /* Configure GPIO pin 4. */
#define POZYX_CONFIG_LEDS       0x15    /* Configure the LEDs */
#define POZYX_POS_ALG         0x16    /* Algorithm used for positioning */
#define POZYX_POS_NUM_ANCHORS     0x17    /* Configure the number of anchors and selection procedure */
#define POZYX_POS_INTERVAL        0x18    /* Defines the update interval in ms in continuous positioning. */
#define POZYX_NETWORK_ID        0x1A    /* The network id.  */
#define POZYX_UWB_CHANNEL       0x1C    /* UWB channel number. */
#define POZYX_UWB_RATES         0x1D    /* Configure the UWB datarate and pulse repetition frequency (PRF) */
#define POZYX_UWB_PLEN          0x1E    /* Configure the UWB preamble length.  */
#define POZYX_UWB_GAIN          0x1F    /* Configure the power gain for the UWB transmitter */
#define POZYX_UWB_XTALTRIM        0x20    /* Trimming value for the uwb crystal. */
#define POZYX_RANGE_PROTOCOL      0x21    /* The ranging protocol */
#define POZYX_OPERATION_MODE      0x22    /* Configure the mode of operation of the pozyx device */
#define POZYX_SENSORS_MODE        0x23    /* Configure the mode of operation of the sensors */

/* Positioning data */ 
#define POZYX_POS_X           0x30    /* x-coordinate of the device in mm. */
#define POZYX_POS_Y           0x34    /* y-coordinate of the device in mm. */
#define POZYX_POS_Z           0x38    /* z-coordinate of the device in mm. */
#define POZYX_POS_ERR_X         0x3C    /* estimated error covariance of x */
#define POZYX_POS_ERR_Y         0x3E    /* estimated error covariance of y */
#define POZYX_POS_ERR_Z         0x40    /* estimated error covariance of z */
#define POZYX_POS_ERR_XY        0x42    /* estimated covariance of xy */
#define POZYX_POS_ERR_XZ        0x44    /* estimated covariance of xz */
#define POZYX_POS_ERR_YZ        0x46    /* estimated covariance of yz */

/* Sensor data */ 
#define POZYX_PRESSURE          0x50    /* Pressure data in mPa */
#define POZYX_ACCEL_X         0x54    /* Accelerometer data (in mg) */
#define POZYX_ACCEL_Y         0x56
#define POZYX_ACCEL_Z         0x58
#define POZYX_MAGN_X          0x5A    /* Magnemtometer data */
#define POZYX_MAGN_Y          0x5C
#define POZYX_MAGN_Z          0x5E
#define POZYX_GYRO_X          0x60    /* Gyroscope data */
#define POZYX_GYRO_Y          0x62
#define POZYX_GYRO_Z          0x64
#define POZYX_EUL_HEADING       0x66    /* Euler angles heading (or yaw)  (1 degree = 16 LSB ) */
#define POZYX_EUL_ROLL          0x68    /* Euler angles roll ( 1 degree = 16 LSB ) */
#define POZYX_EUL_PITCH         0x6A    /* Euler angles pitch ( 1 degree = 16 LSB ) */
#define POZYX_QUAT_W          0x6C    /* Weight of quaternion. */
#define POZYX_QUAT_X          0x6E    /* x of quaternion */
#define POZYX_QUAT_Y          0x70    /* y of quaternion */
#define POZYX_QUAT_Z          0x72    /* z of quaternion */
#define POZYX_LIA_X           0x74    /* Linear acceleration in x-direction */
#define POZYX_LIA_Y           0x76    /* Linear acceleration in y-direction */
#define POZYX_LIA_Z           0x78    /* Linear acceleration in z-direction */
#define POZYX_GRAV_X          0x7A    /* x-component of gravity vector  */
#define POZYX_GRAV_Y          0x7C    /* y-component of gravity vector  */
#define POZYX_GRAV_Z          0x7E    /* z-component of gravity vector  */
#define POZYX_TEMPERATURE       0x80    /* Temperature */

/* General data */ 
#define POZYX_DEVICE_LIST_SIZE      0x81    /* Returns the number of devices stored internally */
#define POZYX_RX_NETWORK_ID       0x82    /* The network id of the latest received message */
#define POZYX_RX_DATA_LEN       0x84    /* The length of the latest received message */
#define POZYX_GPIO1           0x85    /* Value of the GPIO pin 1 */
#define POZYX_GPIO2           0x86    /* Value of the GPIO pin 2 */
#define POZYX_GPIO3           0x87    /* Value of the GPIO pin 3 */
#define POZYX_GPIO4           0x88    /* Value of the GPIO pin 4 */

/* Functions */ 
#define POZYX_RESET_SYS         0xB0    /* Reset the Pozyx device */
#define POZYX_LED_CTRL          0xB1    /* Control LEDS 1 to 4 on the board */
#define POZYX_TX_DATA         0xB2    /* Write data in the UWB transmit (TX) buffer */
#define POZYX_TX_SEND         0xB3    /* Transmit the TX buffer to some other pozyx device */
#define POZYX_RX_DATA         0xB4    /* Read data from the UWB receive (RX) buffer */
#define POZYX_DO_RANGING        0xB5    /* Initiate ranging measurement */
#define POZYX_DO_POSITIONING      0xB6    /* Initiate the positioning process.  */
#define POZYX_POS_SET_ANCHOR_IDS    0xB7    /* Set the list of anchor ID's used for positioning.  */
#define POZYX_POS_GET_ANCHOR_IDS    0xB8    /* Read the list of anchor ID's used for positioning. */

/* Device list functions */ 
#define POZYX_DEVICES_GETIDS      0xB9    /* Get all the network IDs's of devices in the device list. */
#define POZYX_DEVICES_DISCOVER      0xBA    /* Obtain the network ID's of all pozyx devices within range. */
#define POZYX_DEVICES_CALIBRATE     0xBB    /* Obtain the coordinates of the pozyx (anchor) devices within range. */
#define POZYX_DEVICES_CLEAR       0xBC    /* Clear the list of all pozyx devices. */
#define POZYX_DEVICE_ADD        0xBD    /* Add a pozyx device to the devices list */
#define POZYX_DEVICE_GETINFO      0xBE    /* Get the stored device information for a given pozyx device */
#define POZYX_DEVICE_GETCOORDS      0xBF    /* Get the stored coordinates of a given pozyx device */
#define POZYX_DEVICE_GETRANGEINFO   0xC0    /* Get the stored range inforamation of a given pozyx device */
#define POZYX_CIR_DATA          0xC1    /* Get the channel impulse response (CIR) coefficients */


/* Macro's to test if registers are readable/writable */
#define IS_REG_READABLE(x)       ((((x)==0x0)||((x)==0x1)||((x)==0x2)||((x)==0x3)||((x)==0x4)||((x)==0x5)||((x)==0x6)||((x)==0x10)||((x)==0x11)||((x)==0x12)||((x)==0x13)||((x)==0x14)||((x)==0x15)||((x)==0x16)||((x)==0x17)||((x)==0x18)||((x)==0x19)||((x)==0x1A)||((x)==0x1B)||((x)==0x1C)||((x)==0x1D)||((x)==0x1E)||((x)==0x1F)||((x)==0x20)||((x)==0x21)||((x)==0x22)||((x)==0x23)||((x)==0x2D)||((x)==0x2E)||((x)==0x2F)||((x)==0x30)||((x)==0x31)||((x)==0x32)||((x)==0x33)||((x)==0x34)||((x)==0x35)||((x)==0x36)||((x)==0x37)||((x)==0x38)||((x)==0x39)||((x)==0x3A)||((x)==0x3B)||((x)==0x3C)||((x)==0x3D)||((x)==0x3E)||((x)==0x3F)||((x)==0x40)||((x)==0x41)||((x)==0x42)||((x)==0x43)||((x)==0x44)||((x)==0x45)||((x)==0x46)||((x)==0x47)||((x)==0x50)||((x)==0x51)||((x)==0x52)||((x)==0x53)||((x)==0x54)||((x)==0x55)||((x)==0x56)||((x)==0x57)||((x)==0x58)||((x)==0x59)||((x)==0x5A)||((x)==0x5B)||((x)==0x5C)||((x)==0x5D)||((x)==0x5E)||((x)==0x5F)||((x)==0x60)||((x)==0x61)||((x)==0x62)||((x)==0x63)||((x)==0x64)||((x)==0x65)||((x)==0x66)||((x)==0x67)||((x)==0x68)||((x)==0x69)||((x)==0x6A)||((x)==0x6B)||((x)==0x6C)||((x)==0x6D)||((x)==0x6E)||((x)==0x6F)||((x)==0x70)||((x)==0x71)||((x)==0x72)||((x)==0x73)||((x)==0x74)||((x)==0x75)||((x)==0x76)||((x)==0x77)||((x)==0x78)||((x)==0x79)||((x)==0x7A)||((x)==0x7B)||((x)==0x7C)||((x)==0x7D)||((x)==0x7E)||((x)==0x7F)||((x)==0x80)||((x)==0x81)||((x)==0x82)||((x)==0x83)||((x)==0x84)||((x)==0x85)||((x)==0x86)||((x)==0x87)||((x)==0x88))?1:0) 
#define IS_REG_WRITABLE(x)       ((((x)==0x10)||((x)==0x11)||((x)==0x12)||((x)==0x13)||((x)==0x14)||((x)==0x15)||((x)==0x16)||((x)==0x17)||((x)==0x18)||((x)==0x19)||((x)==0x1A)||((x)==0x1B)||((x)==0x1C)||((x)==0x1D)||((x)==0x1E)||((x)==0x1F)||((x)==0x20)||((x)==0x21)||((x)==0x22)||((x)==0x23)||((x)==0x2D)||((x)==0x2E)||((x)==0x2F)||((x)==0x30)||((x)==0x31)||((x)==0x32)||((x)==0x33)||((x)==0x34)||((x)==0x35)||((x)==0x36)||((x)==0x37)||((x)==0x38)||((x)==0x39)||((x)==0x3A)||((x)==0x3B)||((x)==0x85)||((x)==0x86)||((x)==0x87)||((x)==0x88))?1:0) 
#define IS_FUNCTIONCALL(x)       ((((x)==0xB0)||((x)==0xB1)||((x)==0xB2)||((x)==0xB3)||((x)==0xB4)||((x)==0xB5)||((x)==0xB6)||((x)==0xB7)||((x)==0xB8)||((x)==0xB9)||((x)==0xBA)||((x)==0xBB)||((x)==0xBC)||((x)==0xBD)||((x)==0xBE)||((x)==0xBF)||((x)==0xC0)||((x)==0xC1)||((x)==0xE0)||((x)==0xFA)||((x)==0xFB))?1:0) 

/* Bit mask for POZYX_ST_RESULT */
#define POZYX_ST_RESULT_ACC       0x01
#define POZYX_ST_RESULT_MAGN      0x02
#define POZYX_ST_RESULT_GYR       0x04
#define POZYX_ST_RESULT_MCU       0x08
#define POZYX_ST_RESULT_PRES      0x10
#define POZYX_ST_RESULT_UWB       0x20

/* Bit mask for POZYX_INT_STATUS */
#define POZYX_INT_STATUS_ERR      0x01
#define POZYX_INT_STATUS_POS      0x02
#define POZYX_INT_STATUS_IMU      0x04
#define POZYX_INT_STATUS_RX_DATA    0x08
#define POZYX_INT_STATUS_FUNC     0x10

/* Bit mask for POZYX_INT_MASK */
#define POZYX_INT_MASK_ERR        0x01
#define POZYX_INT_MASK_POS        0x02
#define POZYX_INT_MASK_IMU        0x04
#define POZYX_INT_MASK_RX_DATA      0x08
#define POZYX_INT_MASK_FUNC       0x10
#define POZYX_INT_MASK_TDMA       0x40
#define POZYX_INT_MASK_PIN        0x80

/* Bit mask for POZYX_POS_ALG */
#define POZYX_POS_ALG_UWB_ONLY      0x00
#define POZYX_POS_ALG_TRACKING      0x01
#define POZYX_POS_ALG_LS        0x02

/* Bit mask for POZYX_RANGE_PROTOCOL */
#define POZYX_RANGE_PROTOCOL_SDS_TWR  0x00
#define POZYX_RANGE_PROTOCOL_TWR    0x01
#define POZYX_RANGE_PROTOCOL_TEST   0x02

/* Bit mask for POZYX_LED_CTRL */
#define POZYX_LED_CTRL_LED1       0x01
#define POZYX_LED_CTRL_LED2       0x02
#define POZYX_LED_CTRL_LED3       0x04
#define POZYX_LED_CTRL_LED4       0x08

/* End of auto generated defines */ 



#define POZYX_TYPE                                      0xE0   
#define POZYX_ANCHOR                                    0x00
#define POZYX_TAG                                       0x20 

#define MAX_BUF_SIZE                                    100


#define POZYX_INT_MASK_ALL          0x1F
#define POZYX_DELAY_LOCAL_WRITE     1
#define POZYX_DELAY_LOCAL_FUNCTION  5
#define POZYX_DELAY_REMOTE_WRITE    5
#define POZYX_DELAY_REMOTE_FUNCTION 10
#define POZYX_DELAY_INTERRUPT       100
#define POZYX_DELAY_CALIBRATION     1000  
#define POZYX_FAILURE               0x0
#define POZYX_SUCCESS               0x1 
#define POZYX_TIMEOUT               0x8
#define POZYX_3D                    3  
#define POZYX_2D                    2
#define POZYX_2_5D                  1 
#define POZYX_INT_PIN0              0x0
#define POZYX_INT_PIN1              0x1


#define POZYX_LED_CTRL_LEDRX       0x10
#define POZYX_LED_CTRL_LEDTX       0x20

#define POZYX_ANCHOR_MODE         0
#define POZYX_TAG_MODE            1

// The GPIO modes 
#define POZYX_GPIO_DIGITAL_INPUT    0
#define POZYX_GPIO_PUSHPULL         1
#define POZYX_GPIO_OPENDRAIN        1

// The GPIO pull resistor configuration
#define POZYX_GPIO_NOPULL           0
#define POZYX_GPIO_PULLUP           1
#define POZYX_GPIO_PULLDOWN         2

// anchor selection modes
#define POZYX_ANCHOR_SEL_MANUAL     0  
#define POZYX_ANCHOR_SEL_AUTO       1

// discovery options
#define POZYX_DISCOVERY_ANCHORS_ONLY    0
#define POZYX_DISCOVERY_TAGS_ONLY       1
#define POZYX_DISCOVERY_ALL_DEVICES     2


// how to intercept pozyx events: by polling or by interrupts
#define MODE_POLLING              0
#define MODE_INTERRUPT            1

// Division factors for converting the raw register values to meaningful physical quantities
#define POZYX_POS_DIV_MM               1.0f   
#define POZYX_PRESS_DIV_PA             1000.0f
#define POZYX_ACCEL_DIV_MG             1.0f
#define POZYX_GYRO_DIV_DPS             16.0f
#define POZYX_MAG_DIV_UT               16.0f
#define POZYX_EULER_DIV_DEG            16.0f
#define POZYX_QUAT_DIV                 16384.0f
#define POZYX_TEMP_DIV_CELSIUS         1.0f    

// error-code defintions
#define POZYX_ERROR_NONE                  0x00  
#define POZYX_ERROR_I2C_WRITE             0x01  
#define POZYX_ERROR_I2C_CMDFULL           0x02  
#define POZYX_ERROR_ANCHOR_ADD            0x03  
#define POZYX_ERROR_COMM_QUEUE_FULL       0x04  
#define POZYX_ERROR_I2C_READ              0x05  
#define POZYX_ERROR_UWB_CONFIG            0x06  
#define POZYX_ERROR_OPERATION_QUEUE_FULL  0x07  
#define POZYX_ERROR_TDMA                  0xA0  
#define POZYX_ERROR_STARTUP_BUSFAULT      0x08  
#define POZYX_ERROR_FLASH_INVALID         0x09  
#define POZYX_ERROR_NOT_ENOUGH_ANCHORS    0x0A 
#define POZYX_ERROR_DISCOVERY             0X0B
#define POZYX_ERROR_CALIBRATION           0x0C
#define POZYX_ERROR_FUNC_PARAM            0x0D
#define POZYX_ERROR_ANCHOR_NOT_FOUND      0x0E
#define POZYX_ERROR_GENERAL               0xFF  


#endif
