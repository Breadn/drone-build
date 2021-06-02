/** FLIGHT CONTROLLER PINS
 * 5v     - MPU | 2x Joystick | NRF24L01 
 * GND    - MPU | 2x Joystick | NRF24L01
 * A0/A1  - 2x Joystick |
 * 
 **/

// Library includes
#include <Wire.h>

// User includes
#include "IMU.h"
#include "ESC.h"

const int PERIPHERAL_COUNT = 2;

const unsigned long DELTA_T = 20; // delta time period in ms
unsigned long delta_r = 0;        // delta time record in ms

  bool IMU_ready;
  bool ESC_ready;
  bool allSystems_ready = true;

void setup() {
  Serial.begin(19200);
  Serial.println("\n      ===      System initalization...");

  beginIMU();
  beginESC();

  IMU_ready = statusIMU();
  ESC_ready = statusESC();

  
  String peripherals[PERIPHERAL_COUNT]{"IMU", "ESCs"};
  bool peripheralReady[PERIPHERAL_COUNT]{IMU_ready, ESC_ready};
  
  for(int i=0; i<PERIPHERAL_COUNT; i++) {
    Serial.print(" * Peripheral: "); Serial.print(peripherals[i]); Serial.print(" | Status: "); (peripheralReady[i] ? Serial.println("OK") : Serial.println("N/OK"));
    if(!peripheralReady[i]) allSystems_ready = false;
  }
  if(allSystems_ready) Serial.println("      ^^^      All systems go!");
  else Serial.println("      !!!      System initialize failed!"); 

  Serial.println("\n      ===      System calibration...");
  if(IMU_ready) calibrateIMU();
  Serial.println("      ***      Calibration complete!");
  
  
}

void loop() {
  if(millis() - delta_r >= DELTA_T && IMU_ready) {
    readIMU();
    readAngle();
    //printIMU();

    delta_r += DELTA_T;
  }

  if(ESC_ready) {
    readThrottle();
    printThrottle();
  }

}
