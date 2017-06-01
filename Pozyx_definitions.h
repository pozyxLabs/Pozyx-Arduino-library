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
/* POZYX REGISTERS firmware version v1.0 */

/* Status registers */

#define POZYX_WHO_AM_I          0x0  /* Returns the constant value 0x43. */
#define POZYX_FIRMWARE_VER        0x1  /* Returns the POZYX firmware version. */
#define POZYX_HARDWARE_VER        0x2  /* Returns the POZYX hardware version. */
#define POZYX_ST_RESULT         0x3  /* Returns the self-test result */
#define POZYX_ERRORCODE         0x4  /* Describes a possibly system error. */
#define POZYX_INT_STATUS        0x5  /* Indicates the source of the interrupt. */
#define POZYX_CALIB_STATUS        0x6  /* Returns the calibration status. */

/* Configuration registers */

#define POZYX_INT_MASK          0x10   /* Indicates which interrupts are enabled. */
#define POZYX_INT_CONFIG        0x11   /* Configure the interrupt pin */
#define POZYX_POS_FILTER        0x14   /* Pozyx filter configuration register */
#define POZYX_CONFIG_LEDS       0x15   /* Configure the LEDs */
#define POZYX_POS_ALG         0x16   /* Algorithm used for positioning */
#define POZYX_POS_NUM_ANCHORS       0x17   /* Configure the number of anchors and selection procedure */
#define POZYX_POS_INTERVAL        0x18   /* Defines the update interval in ms in continuous positioning. */
#define POZYX_NETWORK_ID        0x1A   /* The network id.  */
#define POZYX_UWB_CHANNEL       0x1C   /* UWB channel number. */
#define POZYX_UWB_RATES         0x1D   /* Configure the UWB datarate and pulse repetition frequency (PRF) */
#define POZYX_UWB_PLEN          0x1E   /* Configure the UWB preamble length.  */
#define POZYX_UWB_GAIN          0x1F   /* Configure the power gain for the UWB transmitter */
#define POZYX_UWB_XTALTRIM        0x20   /* Trimming value for the uwb crystal. */
#define POZYX_RANGE_PROTOCOL      0x21    /* The ranging protocol */
#define POZYX_OPERATION_MODE        0x22   /* Configure the mode of operation of the pozyx device */
#define POZYX_SENSORS_MODE        0x23   /* Configure the mode of operation of the sensors */
#define POZYX_CONFIG_GPIO1        0x27   /* Configure GPIO pin 1. */
#define POZYX_CONFIG_GPIO2        0x28   /* Configure GPIO pin 2. */
#define POZYX_CONFIG_GPIO3        0x29   /* Configure GPIO pin 3. */
#define POZYX_CONFIG_GPIO4        0x2A   /* Configure GPIO pin 4. */

/* Positioning data */

#define POZYX_POS_X         0x30   /* x-coordinate of the device in mm. */
#define POZYX_POS_Y         0x34   /* y-coordinate of the device in mm. */
#define POZYX_POS_Z         0x38   /* z-coordinate of the device in mm. */
#define POZYX_POS_ERR_X         0x3C   /* estimated error covariance of x */
#define POZYX_POS_ERR_Y         0x3E   /* estimated error covariance of y */
#define POZYX_POS_ERR_Z         0x40   /* estimated error covariance of z */
#define POZYX_POS_ERR_XY        0x42   /* estimated covariance of xy */
#define POZYX_POS_ERR_XZ        0x44   /* estimated covariance of xz */
#define POZYX_POS_ERR_YZ        0x46   /* estimated covariance of yz */

#define POZYX_MAX_LIN_ACC       0x4E   /* maximum linear acceleration since */

/* Sensor data */

#define POZYX_PRESSURE          0x50   /* Pressure data */
#define POZYX_ACCEL_X         0x54   /* Accelerometer data (in mg) */
#define POZYX_ACCEL_Y         0x56   /*  */
#define POZYX_ACCEL_Z         0x58   /*  */
#define POZYX_MAGN_X          0x5A   /* Magnemtometer data */
#define POZYX_MAGN_Y          0x5C   /*  */
#define POZYX_MAGN_Z          0x5E   /*  */
#define POZYX_GYRO_X          0x60   /* Gyroscope data */
#define POZYX_GYRO_Y          0x62   /*  */
#define POZYX_GYRO_Z          0x64   /*  */
#define POZYX_EUL_HEADING       0x66   /* Euler angles heading (or yaw) */
#define POZYX_EUL_ROLL          0x68   /* Euler angles roll */
#define POZYX_EUL_PITCH         0x6A   /* Euler angles pitch */
#define POZYX_QUAT_W          0x6C   /* Weight of quaternion. */
#define POZYX_QUAT_X          0x6E   /* x of quaternion */
#define POZYX_QUAT_Y          0x70   /* y of quaternion */
#define POZYX_QUAT_Z          0x72   /* z of quaternion */
#define POZYX_LIA_X         0x74   /* Linear acceleration in x-direction */
#define POZYX_LIA_Y         0x76   /*  */
#define POZYX_LIA_Z         0x78   /*  */
#define POZYX_GRAV_X          0x7A   /* x-component of gravity vector  */
#define POZYX_GRAV_Y          0x7C   /* y-component of gravity vector  */
#define POZYX_GRAV_Z          0x7E   /* z-component of gravity vector  */
#define POZYX_TEMPERATURE       0x80   /* Temperature */

/* General data */

#define POZYX_DEVICE_LIST_SIZE        0x81   /* Returns the number of devices stored internally */
#define POZYX_RX_NETWORK_ID       0x82   /* The network id of the latest received message */
#define POZYX_RX_DATA_LEN       0x84   /* The length of the latest received message */
#define POZYX_GPIO1         0x85   /* Value of the GPIO pin 1 */
#define POZYX_GPIO2         0x86   /* Value of the GPIO pin 2 */
#define POZYX_GPIO3         0x87   /* Value of the GPIO pin 3 */
#define POZYX_GPIO4         0x88   /* Value of the GPIO pin 4 */

/*Functions*/

#define POZYX_RESET_SYS         0xB0   /* Reset the Pozyx device */
#define POZYX_LED_CTRL          0xB1   /* Control LEDS 1 to 4 on the board */
#define POZYX_TX_DATA         0xB2   /* Write data in the UWB transmit (TX) buffer */
#define POZYX_TX_SEND         0xB3   /* Transmit the TX buffer to some other pozyx device */
#define POZYX_RX_DATA         0xB4   /* Read data from the UWB receive (RX) buffer */
#define POZYX_DO_RANGING        0xB5   /* Initiate ranging measurement */
#define POZYX_DO_POSITIONING        0xB6   /* Initiate the positioning process.  */
#define POZYX_POS_SET_ANCHOR_IDS      0xB7   /* Set the list of anchor ID's used for positioning.  */
#define POZYX_POS_GET_ANCHOR_IDS      0xB8   /* Read the list of anchor ID's used for positioning. */
#define POZYX_FLASH_RESET       0xB9   /* Reset a portion of the configuration in flash memory */
#define POZYX_FLASH_SAVE        0xBA   /* Store a portion of the configuration in flash memory */
#define POZYX_FLASH_DETAILS       0xBB   /* Return information on what is stored in flash */

/*Device list functions*/

#define POZYX_DEVICES_GETIDS        0xC0   /* Get all the network IDs's of devices in the device list. */
#define POZYX_DEVICES_DISCOVER        0xC1   /* Obtain the network ID's of all pozyx devices within range. */
#define POZYX_DEVICES_CALIBRATE       0xC2   /* Obtain the coordinates of the pozyx (anchor) devices within range. */
#define POZYX_DEVICES_CLEAR       0xC3   /* Clear the list of all pozyx devices. */
#define POZYX_DEVICE_ADD        0xC4   /* Add a pozyx device to the devices list */
#define POZYX_DEVICE_GETINFO        0xC5   /* Get the stored device information for a given pozyx device */
#define POZYX_DEVICE_GETCOORDS        0xC6   /* Get the stored coordinates of a given pozyx device */
#define POZYX_DEVICE_GETRANGEINFO     0xC7   /* Get the stored range inforamation of a given pozyx device */
#define POZYX_CIR_DATA          0xC8   /* Get the channel impulse response (CIR) coefficients */

/* Macro's to test if registers are readable/writable */

#define IS_REG_READABLE(x)       (((((x)>=0x0)&&((x)<0x7)) || (((x)>=0x10)&&((x)<0x12)) || (((x)>=0x14)&&((x)<0x24)) || (((x)>=0x27)&&((x)<0x2b)) || (((x)>=0x30)&&((x)<0x48)) || (((x)>=0x50)&&((x)<0x89)))?1:0)
#define IS_REG_WRITABLE(x)       (((((x)>=0x10)&&((x)<0x12)) || (((x)>=0x14)&&((x)<0x24)) || (((x)>=0x27)&&((x)<0x2b)) || (((x)>=0x30)&&((x)<0x3c)) || (((x)>=0x85)&&((x)<0x89)))?1:0)
#define IS_FUNCTIONCALL(x)       (((((x)>=0xb0)&&((x)<0xbc)) || (((x)>=0xc0)&&((x)<0xc9)))?1:0)

/* End of auto generated defines */

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
#define POZYX_POS_ALG_LS            0x00 // deprecated so default to UWB only
#define POZYX_POS_ALG_TRACKING      0x04

/* Bit masks for POZYX_POS_FILTER */
#define FILTER_TYPE_NONE            0x00
#define FILTER_TYPE_FIR             0x01
#define FILTER_TYPE_MOVINGAVERAGE   0x03
#define FILTER_TYPE_MOVINGMEDIAN    0x04

/* Bit mask for POZYX_RANGE_PROTOCOL */
#define POZYX_RANGE_PROTOCOL_PRECISION  0x00
#define POZYX_RANGE_PROTOCOL_FAST    0x01
#define POZYX_RANGE_PROTOCOL_TEST   0x02

/* Bit mask for POZYX_LED_CTRL */
#define POZYX_LED_CTRL_LED1       0x01
#define POZYX_LED_CTRL_LED2       0x02
#define POZYX_LED_CTRL_LED3       0x04
#define POZYX_LED_CTRL_LED4       0x08



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
#define POZYX_ERROR_FLASH         0x0F
#define POZYX_ERROR_MEMORY          0x10
#define POZYX_ERROR_RANGING         0x11
#define POZYX_ERROR_RTIMEOUT1       0x12
#define POZYX_ERROR_RTIMEOUT2       0x13
#define POZYX_ERROR_TXLATE          0x14
#define POZYX_ERROR_UWB_BUSY        0x15
#define POZYX_ERROR_POSALG          0x16
#define POZYX_ERROR_NOACK         0x17
#define POZYX_ERROR_SNIFF_OVERFLOW      0xE0
#define POZYX_ERROR_NO_PPS          0xF0
#define POZYX_ERROR_NEW_TASK        0xF1
#define POZYX_ERROR_UNRECDEV        0xFE
#define POZYX_ERROR_GENERAL              0xFF

// flash configuration types
#define POZYX_FLASH_REGS                  1
#define POZYX_FLASH_ANCHOR_IDS            2
#define POZYX_FLASH_NETWORK               3


// possible interrupt pin configuration settings
#define PIN_MODE_PUSHPULL                 0
#define PIN_ACTIVE_LOW                    0
#define PIN_ACTIVE_HIGH                   1

#endif
