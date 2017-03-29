#line 2 "unit_tests_core.ino"
#include <ArduinoUnit.h>

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

/**
 * These unit tests verify that the interrupts work under different settings. 
 * This requires firmware version v1.0
 *
 * Author, Samuel Van de Velde, Pozyx Labs
 *
 * functions tested:
 * - Pozyx.configInterruptPin
 *
 * This tests the registers:
 * - POZYX_INT_CONFIG
 *
 */
void test_pin(int pinnum, int arduino_pinnum);
void test_pinlatch(int pinnum, int arduino_pinnum);
void trigger_interrupt();

void setup()
{
  Serial.begin(115200);
  while(!Serial); // for the Arduino Leonardo/Micro only
  
  Wire.begin();
    
  Serial.println(F("Start testing interrupts"));
  Serial.println(F("This test requires the Arduino Uno."));
  Serial.println(F("Minimum firmware version v1.0.\n"));
  Serial.println(F("---------------------------------------------------------\n"));  
 
}

void loop()
{
  Test::run();
}

// Test pin 1
test(pin1){
  test_pin(1, 9);
}
test(pin1_latch){
  test_pinlatch(1, 9);
}

// Test pin 2
test(pin2){
  test_pin(2, 10);
}
test(pin2_latch){
  test_pinlatch(2, 10);
}

// Test pin 3
test(pin3){
  test_pin(3, 11);
}
test(pin3_latch){
  test_pinlatch(3, 11);
}

// Test pin 4
test(pin4){
  test_pin(4, 12);
}
test(pin4_latch){
  test_pinlatch(4, 12);
}

// Test pin 5
test(pin5){
  test_pin(5, 2);
}
test(pin5_latch){
  test_pinlatch(5, 2);
}

// Test pin 6
test(pin6){
  test_pin(6, 3);
}
test(pin6_latch){
  test_pinlatch(6, 3);
}

void test_pin(int pinnum, int arduino_pinnum)
{     
  uint8_t int_mask = 0;
  Pozyx.regWrite(POZYX_INT_MASK, &int_mask, 1);
  
  // configure pin 1 in push pull active low, no latch
  // verify that the pin give a high signal (not active)
  Pozyx.configInterruptPin(pinnum, PIN_MODE_PUSHPULL, PIN_ACTIVE_LOW, 0);  
  delay(2);
  int level = digitalRead(arduino_pinnum);  
  assertEqual(level, HIGH);
  
  // configure pin 1 in push pull active high, no latch
  // verify that the pin give a high signal (not active)
  Pozyx.configInterruptPin(pinnum, PIN_MODE_PUSHPULL, PIN_ACTIVE_HIGH, 0);  
  delay(2);
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, LOW);  
}

void test_pinlatch(int pinnum, int arduino_pinnum)
{  
  int level;
  uint8_t int_mask;
  uint8_t int_status;
  
  /////////////////////////////////////
  // test for active low
  /////////////////////////////////////
  
  // turn on the interrupt mask for errors
  int_mask = POZYX_INT_MASK_ERR;
  Pozyx.regWrite(POZYX_INT_MASK, &int_mask, 1);
  
  // configure pin 1 in push pull active low, latch
  // verify that the pin give a high signal (not active)
  Pozyx.configInterruptPin(pinnum, PIN_MODE_PUSHPULL, PIN_ACTIVE_LOW, 1);    
  delay(2);
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, HIGH);
  
  // trigger an interrupt, the level should go to low
  trigger_interrupt();  
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, LOW); 
  
  // after reading the interrupt status, the level should be high again
  int_status =0;
  Pozyx.regRead(POZYX_INT_STATUS, &int_status, 1);
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, HIGH); 
  
  /////////////////////////////////////
  // test for active high
  /////////////////////////////////////

  // configure pin 1 in push pull active low, latch
  // verify that the pin give a high signal (not active)
  Pozyx.configInterruptPin(pinnum, PIN_MODE_PUSHPULL, PIN_ACTIVE_HIGH, 1);    
  delay(2);
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, LOW);
  
  // trigger an interrupt, the level should go to low
  trigger_interrupt();  
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, HIGH); 
  
  // after reading the interrupt status, the level should be high again
  int_status =0;
  Pozyx.regRead(POZYX_INT_STATUS, &int_status, 1);
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, LOW); 
}

void trigger_interrupt()
{
  // call this function without parameters: this will result in an error
  Pozyx.regFunction(POZYX_LED_CTRL);  
  delay(1);
}

/*
test(open_drain)
{
  int pinnum = 5;
  int arduino_pinnum = 2;
  int level;
  uint8_t int_status;
  
  /////////////////////////////////////
  // test for active low
  /////////////////////////////////////
  
  // turn on the interrupt mask for errors
  uint8_t int_mask = POZYX_INT_MASK_ERR;
  Pozyx.regWrite(POZYX_INT_MASK, &int_mask, 1);
  
  pinMode(arduino_pinnum, INPUT);           // set pin to input
  digitalWrite(arduino_pinnum, HIGH);       // turn on pullup resistors
  
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, HIGH);
  
  
  // opendrain, active low, latching
  Pozyx.configInterruptPin(pinnum, PIN_MODE_OPENDRAIN, PIN_ACTIVE_LOW, 1);    
  delay(2);
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, HIGH);
  
  // trigger an interrupt, the level should go to low
  trigger_interrupt();  
  delay(2);
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, LOW); 
  
  // after reading the interrupt status, the level should be high again
  int_status =0;
  Pozyx.regRead(POZYX_INT_STATUS, &int_status, 1);
  delay(2);
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, HIGH); 
  
  
  /////////////////////////////////////
  // test for active low
  /////////////////////////////////////
  
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, LOW);
  
  // opendrain, active low, latching
  Pozyx.configInterruptPin(pinnum, PIN_MODE_OPENDRAIN, PIN_ACTIVE_HIGH, 1);    
  delay(5);
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, LOW);
  
  // trigger an interrupt, the level should go high
  trigger_interrupt();  
  delay(2);
  level = digitalRead(arduino_pinnum);  
  assertEqual(level, HIGH); 
  
  // after reading the interrupt status, the level should be low again
  int_status =0;
  Pozyx.regRead(POZYX_INT_STATUS, &int_status, 1);
  delay(2);
  level = digitalRead(arduino_pinnum);    
  assertEqual(level, LOW); 
  
}
*/
