// Library includes
#include <Wire.h>

// User includes
#include "IMU.h"

const unsigned long DELTA_T = 5; // delta time period in ms
unsigned long delta_r = 0;        // delta time record in ms

void setup() {
  Serial.begin(9600);
  statusIMU();

  beginIMU();
  
  Serial.println("Ready!");
}

void loop() {
  if(millis() - delta_r >= DELTA_T) {
    readIMU();
    readAngle(DELTA_T);

    printIMU();

    delta_r += DELTA_T;
  }

}
