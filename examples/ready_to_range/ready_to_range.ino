/**
  The Pozyx ready to range tutorial (c) Pozyx Labs
  Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_range/Arduino

  This demo requires two Pozyx devices and one Arduino. It demonstrates the ranging capabilities and the functionality to
  to remotely control a Pozyx device. Place one of the Pozyx shields on the Arduino and upload this sketch. Move around
  with the other Pozyx device.

  This demo measures the range between the two devices. The closer the devices are to each other, the more LEDs will
  light up on both devices.
*/

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t destination_id = 0x6069;     // the network id of the other pozyx device: fill in the network id of the other device
signed int range_step_mm = 1000;      // every 1000mm in range, one LED less will be giving light.

uint8_t ranging_protocol = POZYX_RANGE_PROTOCOL_PRECISION; // ranging protocol of the Pozyx.

uint16_t remote_id = 0x605D;          // the network ID of the remote device
bool remote = false;                  // whether to use the given remote device for ranging

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);

  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    abort();
  }
  // setting the remote_id back to NULL will use the local Pozyx
  if (!remote){
    remote_id = NULL;
  }
  Serial.println("------------POZYX RANGING V1.1------------");
  Serial.println("NOTES:");
  Serial.println("- Change the parameters:");
  Serial.println("\tdestination_id (target device)");
  Serial.println("\trange_step (mm)");
  Serial.println();
  Serial.println("- Approach target device to see range and");
  Serial.println("led control");
  Serial.println("------------POZYX RANGING V1.1------------");
  Serial.println();
  Serial.println("START Ranging:");

  // make sure the pozyx system has no control over the LEDs, we're the boss
  uint8_t led_config = 0x0;
  Pozyx.setLedConfig(led_config, remote_id);
  // do the same with the remote device
  Pozyx.setLedConfig(led_config, destination_id);
  // set the ranging protocol
  Pozyx.setRangingProtocol(ranging_protocol, remote_id);
}

void loop(){
  device_range_t range;
  int status = 0;

  // let's perform ranging with the destination
  if(!remote)
    status = Pozyx.doRanging(destination_id, &range);
  else
    status = Pozyx.doRemoteRanging(remote_id, destination_id, &range);

  if (status == POZYX_SUCCESS){
    Serial.print(range.timestamp);
    Serial.print("ms, ");
    Serial.print(range.distance);
    Serial.print("mm, ");
    Serial.print(range.RSS);
    Serial.println("dBm");

    // now control some LEDs; the closer the two devices are, the more LEDs will be lit
    if (ledControl(range.distance) == POZYX_FAILURE){
      Serial.println("ERROR: setting (remote) leds");
    }
  }
  else{
    Serial.println("ERROR: ranging");
  }
}

int ledControl(uint32_t range){
  int status = POZYX_SUCCESS;
  // set the LEDs of the pozyx device
  status &= Pozyx.setLed(4, (range < range_step_mm), remote_id);
  status &= Pozyx.setLed(3, (range < 2*range_step_mm), remote_id);
  status &= Pozyx.setLed(2, (range < 3*range_step_mm), remote_id);
  status &= Pozyx.setLed(1, (range < 4*range_step_mm), remote_id);

  // set the LEDs of the destination pozyx device
  status &= Pozyx.setLed(4, (range < range_step_mm), destination_id);
  status &= Pozyx.setLed(3, (range < 2*range_step_mm), destination_id);
  status &= Pozyx.setLed(2, (range < 3*range_step_mm), destination_id);
  status &= Pozyx.setLed(1, (range < 4*range_step_mm), destination_id);

  // status will be zero if setting the LEDs failed somewhere along the way
  return status;
}
