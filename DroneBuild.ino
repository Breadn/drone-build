// Library includes
#include <Wire.h>

// User includes
#include "IMU.h"


void setup() {
  Serial.begin(9600);
  setupIMU();
  //readStatus();
  
  //Serial.print("Ready!");
}

void loop() {
  readIMU();

}
