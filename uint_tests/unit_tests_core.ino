#line 2 "unit_tests_core.ino"
#include <ArduinoUnit.h>

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

// can we read a single byte
test(regRead_singlebyte)
{
  uint8_t whoami = 0;
  int status = Pozyx.regRead(POZYX_WHO_AM_I, &whoami, 1);
  
  assertEqual(status, POZYX_SUCCESS);
  assertEqual(whoami, 0x43);
}

// can we write a single byte
test(regWrite_singlebyte)
{
  uint8_t writebyte = 19;  
  
  // write the value 1 to this register
  int status = Pozyx.regWrite(POZYX_POS_X, &writebyte, 1);
  
  assertEqual(status, POZYX_SUCCESS);
  
  // this delay is necessary because pozyx is still processing the write operation first.
  delay(1);
  
  uint8_t testbyte = 0;
  Pozyx.regRead(POZYX_POS_X, &testbyte, 1);

  assertEqual(testbyte, writebyte);
}

// can we write multiple bytes at once
test(regWrite_multibyte)
{
  uint32_t pos[3] = {1, 2, 3};  
  
  // write the value 1 to this register
  int status = Pozyx.regWrite(POZYX_POS_X, (uint8_t*)&pos, 3*sizeof(uint32_t));
  
  assertEqual(status, POZYX_SUCCESS);
  
  // this delay is necessary because pozyx is still processing the write operation first.
  delay(1);
  
  pos[0] = 0;
  pos[1] = 0;
  pos[2] = 0;
  Pozyx.regRead(POZYX_POS_X, (uint8_t*)&pos, 3*sizeof(uint32_t));

  assertEqual(pos[0], 1);
  assertEqual(pos[1], 2);
  assertEqual(pos[2], 3);
}

// can we write a single byte quickly after one-another.
test(regWrite_quick)
{
  int num_cycles = 10;
  uint8_t i = 0;
  for(i=1; i <= num_cycles; i++){
    int status = Pozyx.regWrite(POZYX_POS_X, (uint8_t*)&i, 1);
    assertEqual(status, POZYX_SUCCESS);
  }
  
  // this delay is necessary because pozyx is still processing the write operation first.
  delay(1);
  
  uint8_t result = 0;
  Pozyx.regRead(POZYX_POS_X, (uint8_t*)&result, 1);

  assertEqual(result, num_cycles);
}

test(regFunction_simple)
{
  uint8_t input = 0x88;  
 
  // call function to write to some buffer
  int status = Pozyx.regFunction( POZYX_LED_CTRL, (uint8_t*)&input, 1, NULL, 0);
  assertEqual(status, POZYX_SUCCESS);
}

// can we call a register function?
test(regFunction)
{
  uint16_t input[3] = {1, 2, 3};  
 
  // call function to write to some buffer
  int status = Pozyx.regFunction( POZYX_POS_SET_ANCHOR_IDS, (uint8_t*)&input, 3*sizeof(uint16_t), NULL, 0);
  assertEqual(status, POZYX_SUCCESS);
  
  // call function to read from some buffer
  uint16_t result[3] = {0,0,0};
  status = Pozyx.regFunction( POZYX_POS_GET_ANCHOR_IDS, NULL, 0, (uint8_t*)&result, 3*sizeof(uint16_t));
  
  assertEqual(result[0], 1);
  assertEqual(result[1], 2);
  assertEqual(result[2], 3);
  
  assertEqual(status, 3);  
}

test(waitForFlag)
{
  Pozyx.begin(false, MODE_INTERRUPT, POZYX_INT_STATUS_ERR);
  //Pozyx.begin(false, MODE_POLLING, POZYX_INT_STATUS_ERR);
  
  // read out the interrupt status register to reset it.
  uint8_t dummy;
  Pozyx.regRead(POZYX_INT_STATUS, &dummy, 1);  
  
  // cause an error interrupt by writing to a read-only register
  uint8_t causes_error = 0x01;
  Pozyx.regWrite(POZYX_WHO_AM_I, &causes_error, 1);

  // if the interrupt was caught, the result is true
  boolean result = Pozyx.waitForFlag(POZYX_INT_STATUS_ERR, 100);
  
  assertTrue(result);
}

void setup()
{
  Serial.begin(115200);
  while(!Serial); // for the Arduino Leonardo/Micro only
  
  Wire.begin();
}

void loop()
{
  Test::run();
}
