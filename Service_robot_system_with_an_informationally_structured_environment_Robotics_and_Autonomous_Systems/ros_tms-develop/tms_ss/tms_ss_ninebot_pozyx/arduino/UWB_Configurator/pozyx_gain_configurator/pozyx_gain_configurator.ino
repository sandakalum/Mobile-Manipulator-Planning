/**
 * This sketch sets (and saves if desired) the UWB gain on all entered devices given in
 * the parameter list.
 * Use this to supplement the UWB configurator
 *
 * Example use:
 * ---------------------------------------
 * set the desired gain in the desired_gain variable
 * set whether you want the gain to be saved or not in save_gain
 * set the IDs of the devices you want to choose the gain on in device_ids, set the number too
 *
 * Author, Laurent Van Acker, Pozyx Labs
 *
 */

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

float desired_gain = 33.0f;

boolean save_gain = true;

//0x0000: Local Tag
//const int num_devices = 18;
//uint16_t devices[num_devices] = {0x6e48, 0x0000, 0x6e39, 0x6e22, 0x6050, 0x6044, 0x6037, 0x6023, 0x6e30, 0x6e31, 0x6e58, 0x6e08, 0x6e23, 0x6e49, 0x6e10, 0x6164, 0x6e04, 0x6e28};
const int num_devices = 2;
uint16_t devices[num_devices] = {0x6e69, 0x0000};

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);

  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

  for (int i = 0; i < num_devices; i++){
    int status = Pozyx.setTxPower(desired_gain, devices[i]);
    uint8_t registers[1] = {POZYX_UWB_GAIN};
    if (save_gain){
      status &= Pozyx.saveRegisters(registers, 1, devices[i]);
    }
    if (status == POZYX_SUCCESS){
      printSuccess(devices[i]);
    }
    else{
      printFailure(devices[i]);
    }
  }
}

void printSuccess(uint16_t device_id){
  Serial.print("Configuration of device 0x");
  Serial.print(device_id, 16);
  Serial.print(" was successful!\n");
}

void printFailure(uint16_t device_id){
  Serial.print("Configuration of device 0x");
  Serial.print(device_id, 16);
  Serial.print(" failed!\n");
}

void loop(){
  delay(10000);
}
