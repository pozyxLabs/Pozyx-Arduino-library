/**
* Pozyx_core.cpp
* --------------
* This file contains the defintion of the core POZYX functions and variables
*
*/

#include "Pozyx.h"
#include <Wire.h>

extern "C" {
  #include "Pozyx_definitions.h"
}

int PozyxClass::_interrupt;
int PozyxClass::_mode;

int PozyxClass::_hw_version;       // pozyx harware version
int PozyxClass::_fw_version;       // pozyx firmware version. (By updating the firmware on the pozyx device, this value can change);

/**
 * The interrupt handler for the pozyx device: keeping it uber short!
 */
void PozyxClass::IRQ()
{
  _interrupt = 1;
}

boolean PozyxClass::waitForFlag(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt)
{
  long timer = millis();
  int status;

  // stay in this loop until the event interrupt flag is set or until the the timer runs out
  while(millis()-timer < timeout_ms)
  {
    // in polling mode, we insert a small delay such that we don't swamp the i2c bus
    if( _mode == MODE_POLLING ){
      delay(1);
    }

    if( (_interrupt == 1) || (_mode == MODE_POLLING))
    {
      _interrupt = 0;

      // Read out the interrupt status register. After reading from this register, pozyx automatically clears the interrupt flags.
      uint8_t interrupt_status = 0;
      status = regRead(POZYX_INT_STATUS, &interrupt_status, 1);
      if((interrupt_status & interrupt_flag) && status == POZYX_SUCCESS)
      {
        // one of the interrupts we were waiting for arrived!
        if(interrupt != NULL)
          *interrupt = interrupt_status;
        return true;
      }
    }
  }
  // too bad, pozyx didn't respond
  // 1) pozyx can select from two pins to generate interrupts, make sure the correct pin is connected with the attachInterrupt() function.
  // 2) make sure the interrupt we are waiting for is enabled in the POZYX_INT_MASK register)
  return false;
}

boolean PozyxClass::waitForFlag_safe(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt)
{
  int tmp = _mode;
  _mode = MODE_POLLING;
  boolean result = waitForFlag(interrupt_flag, timeout_ms, interrupt);
  _mode = tmp;
  return result;
}

int PozyxClass::begin(boolean print_result, int mode, int interrupts, int interrupt_pin){

  int status = POZYX_SUCCESS;

  if(print_result){
    Serial.println("Pozyx Shield");
    Serial.println("------------");
  }

  // check if the mode parameter is valid
  if((mode != MODE_POLLING) && (mode != MODE_INTERRUPT))
    return POZYX_FAILURE;

  // check if the pin is valid
  if((interrupt_pin != 0) && (interrupt_pin != 1))
    return POZYX_FAILURE;

  Wire.begin();

  // wait a bit until the pozyx board is up and running
  delay(250);

  _mode = mode;

  uint8_t whoami, selftest;
  uint8_t regs[3];
  regs[2] = 0x12;

  // we read out the first 3 register values: who_am_i, firmware_version and harware version, respectively.
  if(regRead(POZYX_WHO_AM_I, regs, 3) == POZYX_FAILURE){
    return POZYX_FAILURE;
  }
  whoami = regs[0];
  _fw_version = regs[1];
  _hw_version = regs[2];

  if(print_result){
    Serial.print("WhoAmI: 0x");
    Serial.println(whoami, HEX);
    Serial.print("FW ver.: v");
    Serial.print((_fw_version&0xF0)>>4);
    Serial.print(".");
    Serial.print((_fw_version&0x0F));
    if(_fw_version < 0x10)
      Serial.print(" (please upgrade)");
    Serial.print("\nHW ver.: ");
    Serial.println(_hw_version);
  }
  // verify if the whoami is correct
  if(whoami != 0x43) {
    // possibly the pozyx is not connected right. Also make sure the jumper of the boot pins is present.
    status = POZYX_FAILURE;
  }

  // readout the selftest registers to validate the proper functioning of pozyx
  if(regRead(POZYX_ST_RESULT, &selftest, 1) == POZYX_FAILURE){
    return POZYX_FAILURE;
  }

  if(print_result){
    Serial.print("selftest: 0b");
    Serial.println(selftest, BIN);
  }

  if((_hw_version & POZYX_TYPE) == POZYX_TAG)
  {
    // check if the uwb, pressure sensor, accelerometer, magnetometer and gyroscope are working
    if(selftest != 0b00111111) {
      status = POZYX_FAILURE;
    }
  }else if((_hw_version & POZYX_TYPE) == POZYX_ANCHOR)
  {
    // check if the uwb transceiver and pressure sensor are working
    if(selftest != 0b0011000) {
      status = POZYX_FAILURE;
    }
    return status;
  }

  if(_mode == MODE_INTERRUPT){
    // set the function that must be called upon an interrupt
    // put your main code here, to run repeatedly:
#if defined(__SAMD21G18A__) || defined(__ATSAMD21G18A__)
    // Arduino Tian
    int tian_interrupt_pin = interrupt_pin;
    attachInterrupt(interrupt_pin+2, IRQ, RISING);
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // Arduino UNO, Mega
    attachInterrupt(interrupt_pin, IRQ, RISING);
#else
    Serial.println("This is not a board supported by Pozyx, interrupts may not work");
    attachInterrupt(interrupt_pin, IRQ, RISING);
#endif

    // use interrupt as provided and initiate the interrupt mask
    uint8_t int_mask = interrupts;
    configInterruptPin(5+interrupt_pin, PIN_MODE_PUSHPULL, PIN_ACTIVE_LOW, 0);

    if (regWrite(POZYX_INT_MASK, &int_mask, 1) == POZYX_FAILURE){
      return POZYX_FAILURE;
    }
  }

  // all done
  delay(POZYX_DELAY_LOCAL_WRITE);
  return status;
}

/**
  * Reads a number of bytes from the specified pozyx register address using I2C
  */
int PozyxClass::regRead(uint8_t reg_address, uint8_t *pData, int size)
{
  // BUFFER_LENGTH is defined in wire.h, it limits the maximum amount of bytes that can be transmitted/received with i2c in one go
  // because of this, we may have to split up the i2c reads in smaller chunks

  if(!IS_REG_READABLE(reg_address))
    return POZYX_FAILURE;

  int n_runs = ceil((float)size / BUFFER_LENGTH);
  int i;
  int status = 1;
  uint8_t reg;

  for(i=0; i<n_runs; i++)
  {
    int offset = i*BUFFER_LENGTH;
    reg = reg_address+offset;

    if(i+1 != n_runs){
      status &= i2cWriteRead(&reg, 1, pData+offset, BUFFER_LENGTH);
    }else{
      status &= i2cWriteRead(&reg, 1, pData+offset, size-offset);
    }
  }

  return status;
}

/**
  * Writes a number of bytes to the specified pozyx register address using I2C
  */
int PozyxClass::regWrite(uint8_t reg_address, uint8_t *pData, int size)
{
  // BUFFER_LENGTH is defined in wire.h, it limits the maximum amount of bytes that can be transmitted/received with i2c in one go
  // because of this, we may have to split up the i2c writes in smaller chunks

  if(!IS_REG_WRITABLE(reg_address))
    return POZYX_FAILURE;

  int n_runs = ceil((float)size / BUFFER_LENGTH);
  int i;
  int status = 1;

  for(i=0; i<n_runs; i++)
  {
    int offset = i*BUFFER_LENGTH;
    if(i+1 != n_runs){
      status &= i2cWriteWrite(reg_address+offset, pData+offset, BUFFER_LENGTH);
    }else{
      status &= i2cWriteWrite(reg_address+offset, pData+offset, size-offset);
    }
  }

  return status;
}

/**
  * Call a register function using i2c with given parameters, the data from the function is stored in pData
  */
int PozyxClass::regFunction(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{
  assert(BUFFER_LENGTH >= size+1);           // Arduino-specific code for the i2c
  assert(BUFFER_LENGTH >= param_size+1);     // Arduino-specific code for the i2c

  if(!IS_FUNCTIONCALL(reg_address))
    return POZYX_FAILURE;

  uint8_t status;

  // this feels a bit clumsy with all these memcpy's
  uint8_t write_data[param_size+1];
  write_data[0] = reg_address;
  memcpy(write_data+1, params, param_size);
  uint8_t read_data[size+1];

  // first write some data with i2c and then read some data
  status = i2cWriteRead(write_data, param_size + 1, read_data, size+1);
  if(status == POZYX_FAILURE)
    return status;

  memcpy(pData, read_data+1, size);


  // the first byte that a function returns is always it's success indicator, so we simply pass this through
  return read_data[0];
}


/**
 * Wirelessly write a number of bytes to a specified register address on a remote Pozyx device using UWB.
 */
int PozyxClass::remoteRegWrite(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
  // some checks
  if(!IS_REG_WRITABLE(reg_address))      return POZYX_FAILURE;    // the register is not writable
  if(size > MAX_BUF_SIZE-1)              return POZYX_FAILURE;    // trying to write too much data

  int status = 0;

  // first prepare the packet to send
  uint8_t tmp_data[size+1];
  tmp_data[0] = 0;
  tmp_data[1] = reg_address;              // the first byte is the register address we want to start writing to.
  memcpy(tmp_data+2, pData, size);         // the remaining bytes are the data bytes to be written starting at the register address.
  status = regFunction(POZYX_TX_DATA, (uint8_t *)&tmp_data, size+2, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x04;    // flag to indicate a register write

  uint8_t int_status = 0;
  regRead(POZYX_INT_STATUS, &int_status, 1);      // first clear out the interrupt status register by reading from it
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);

  if (waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, 100, &int_status)){
    if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
    {
      // An error occured during positioning.
      // Please read out the register POZYX_ERRORCODE to obtain more information about the error
      return POZYX_FAILURE;
    }else{
      return POZYX_SUCCESS;
    }
  }else{
    return POZYX_TIMEOUT;
  }

  return status;
}

/**
 * Wirelessly read a number of bytes from a specified register address on a remote Pozyx device using UWB.
 */
int PozyxClass::remoteRegRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
  // some checks
  if(!IS_REG_READABLE(reg_address))      return POZYX_FAILURE;        // the register is not readable
  if(size > MAX_BUF_SIZE)                return POZYX_FAILURE;        // trying to read too much data
  if(destination == 0)                   return POZYX_FAILURE;        // remote read not allowed in broadcast mode

  int status = 0;

  // first prepare the packet to send
  uint8_t tmp_data[3];
  tmp_data[0] = 0;                  // the offset in the TX buffer
  tmp_data[1] = reg_address;        // the first byte is the register address we want to start reading from
  tmp_data[2] = size;               // the number of bytes to read starting from the register address
  status = regFunction(POZYX_TX_DATA, (uint8_t *)&tmp_data, 3, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x02;    // flag to indicate a register read

  uint8_t int_status = 0;
  regRead(POZYX_INT_STATUS, &int_status, 1);      // first clear out the interrupt status register by reading from it
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);

  // stop if POZYX_TX_SEND returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // wait up to x ms to receive a response
  if(waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, 1000, &int_status))
  {
    if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
    {
      // An error occured during positioning.
      // Please read out the register POZYX_ERRORCODE to obtain more information about the error
      return POZYX_FAILURE;
    }else{
      // we received a response, now get some information about the response
      uint8_t rx_info[3]= {0,0,0};
      regRead(POZYX_RX_NETWORK_ID, rx_info, 3);
      uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
      uint8_t data_len = rx_info[2];

      if( remote_network_id == destination && data_len == size)
      {
        status = readRXBufferData(pData, size);
        return status;
      }else{
        return POZYX_FAILURE;
      }
    }

  }else{
    // timeout
    return POZYX_TIMEOUT;
  }
}

/*
 * Wirelessly call a register function with given parameters on a remote Pozyx device using UWB, the data from the function is stored in pData
 */
int PozyxClass::remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{
  // some checks
  if(!IS_FUNCTIONCALL(reg_address))      return POZYX_FAILURE;        // the register is not a function

  int status = 0;

  // first prepare the packet to send
  uint8_t tmp_data[param_size+2];
  tmp_data[0] = 0;
  tmp_data[1] = reg_address;                // the first byte is the function register address we want to call.
  memcpy(tmp_data+2, params, param_size);   // the remaining bytes are the parameter bytes for the function.
  status = regFunction(POZYX_TX_DATA, tmp_data, param_size+2, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
  {
    return status;
  }

  // send the packet
  uint8_t tx_params[3];
  tx_params[0] = (uint8_t)destination;
  tx_params[1] = (uint8_t)(destination>>8);
  tx_params[2] = 0x08;    // flag to indicate a register function call
  uint8_t int_status = 0;
  regRead(POZYX_INT_STATUS, &int_status, 1);      // first clear out the interrupt status register by reading from it
  status = regFunction(POZYX_TX_SEND, tx_params, 3, NULL, 0);

  // stop if POZYX_TX_SEND returned an error.
  if(status == POZYX_FAILURE){
    return status;
  }

  // wait up to x ms to receive a response
  if(waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, 1000, &int_status))
  {
    if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
    {
      return POZYX_FAILURE;
    }else
    {
      // we received a response, now get some information about the response
      uint8_t rx_info[3];
      regRead(POZYX_RX_NETWORK_ID, rx_info, 3);
      uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
      uint8_t data_len = rx_info[2];

      if( remote_network_id == destination && data_len == size+1)
      {
        uint8_t return_data[size+1];

        status = readRXBufferData(return_data, size+1);

        if(status == POZYX_FAILURE){
          // debug information
          return status;
        }

        memcpy(pData, return_data+1, size);

        return return_data[0];
      }else{
        return POZYX_FAILURE;
      }
    }

  }else{
    // timeout
    return POZYX_TIMEOUT;
  }
}

int PozyxClass::writeTXBufferData(uint8_t data[], int size, int offset)
{
  if (offset + size > MAX_BUF_SIZE){
    return POZYX_FAILURE;
  }

  int i, status = 1;
  int max_bytes = BUFFER_LENGTH-2;
  int n_runs = ceil((float)size / max_bytes);
  uint8_t params[BUFFER_LENGTH];

  // read out the received data.
  for(i=0; i<n_runs; i++)
  {
    params[0] = offset + i*max_bytes;      // the offset
    if(i+1 != n_runs){
      memcpy(params+1, data+i*max_bytes, max_bytes);
      status &= regFunction(POZYX_TX_DATA, params, max_bytes + 1, NULL, 0);
    }else{
      memcpy(params+1, data+i*max_bytes, size-i*max_bytes);
      status &= regFunction(POZYX_TX_DATA, params, size-i*max_bytes+1, NULL, 0);
    }
  }

  return status;
}

int PozyxClass::readRXBufferData(uint8_t* pData, int size)
{
  if (size > MAX_BUF_SIZE){
    return POZYX_FAILURE;
  }

  int status;
  int i;
  uint8_t params[2];
  int max_bytes = BUFFER_LENGTH-1;
  int n_runs = ceil((float)size / max_bytes);

  // read out the received data.
  for(i=0; i<n_runs; i++)
  {
    params[0] = i*max_bytes;      // the offset
    if(i+1 != n_runs){
      params[1] = max_bytes;      // the number of bytes to read
    }else{
      params[1] = size - i*max_bytes;      // the number of bytes to read
    }
    status = regFunction(POZYX_RX_DATA, params, 2, pData+params[0], params[1]);
  }

  return status;
}

int PozyxClass::sendTXBufferData(uint16_t destination)
{
  int status;

  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x06;
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
  delay(POZYX_DELAY_LOCAL_FUNCTION);

  return status;
}


/*
 * This function sends some data bytes to the destination
 */
int PozyxClass::sendData(uint16_t destination, uint8_t *pData, int size)
{
  if(size > MAX_BUF_SIZE)          return POZYX_FAILURE;        // trying to send too much data

  uint8_t status = 0;

  uint8_t tmp_data[size+1];
  tmp_data[0] = 0;                        // the first byte is the offset byte.
  memcpy(tmp_data+1, pData, size);

  // set the TX buffer
  status = regFunction(POZYX_TX_DATA, tmp_data, size+1, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x06;    // flag to indicate we're just sending data
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);

  return status;
}

/**
  * Writes a number of bytes to the specified pozyx register address using I2C
  */
int PozyxClass::i2cWriteWrite(const uint8_t reg_address, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(POZYX_I2C_ADDRESS);
  // write the starting register address
  n = Wire.write(reg_address);
  if (n != 1)
    return (POZYX_FAILURE);

  error = Wire.endTransmission(false); // hold the bus for a repeated start
  if (error != 0)
    return (POZYX_FAILURE);

  Wire.beginTransmission(POZYX_I2C_ADDRESS);
  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (POZYX_FAILURE);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (POZYX_FAILURE);

  return (POZYX_SUCCESS);         // return : no error
}

/**
  * Call a register function using I2C with given parameters
  */
int PozyxClass::i2cWriteRead(uint8_t* write_data, int write_len, uint8_t* read_data, int read_len)
{
  int i, n;

  Wire.beginTransmission(POZYX_I2C_ADDRESS);
  for(i=0; i<write_len; i++){
    n = Wire.write(*(write_data+i));  // write parameter bytes
  }

  if (n != 1)
    return (POZYX_FAILURE);

  n = Wire.endTransmission(false);    // hold the I2C-bus for a repeated start

  if (n != 0)
    return (POZYX_FAILURE);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(POZYX_I2C_ADDRESS, read_len, true);
  i = 0;

  while(Wire.available())
  {
    if(i<read_len)
      read_data[i++]=Wire.read();
    else
      Wire.read();
  }

  if ( i != read_len){
    return (POZYX_FAILURE);
  }

  return (POZYX_SUCCESS);  // return : no error
}

PozyxClass Pozyx;
